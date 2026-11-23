# magfit.py - hard- and soft-iron magnetometer calibration (ellipsoid fit) without numpy.
#
# Same method as Controls imu_cal/calibrate_mag.py, rewritten for MicroPython on the Pico H:
#
#   A perfect magnetometer rotated through every direction traces a sphere centred on zero.
#   Real ones trace an ellipsoid that is
#     - shifted   by magnets/steel on the rover      ("hard iron")  -> the OFFSET
#     - stretched by nearby soft metal and the chip  ("soft iron")  -> the 3x3 MATRIX
#   We fit   u' Q u + L . u = 1   (u = normalised sample, Q 3x3 symmetric, L 3-vector: 9 numbers)
#   by least squares, then
#     centre (offset)  c = -0.5 Q^-1 L
#     matrix           M = sqrt(shape of Q), scaled so a calibrated reading has the length of the
#                      average measured field (tesla).
#   Calibrated field = M (raw - offset)  - applied by calibration.apply_mag(), as in imu_cal_live.
#
# Differences from the numpy version, all needed because the RP2040 uses 32-bit floats
# (about 7 significant digits instead of 16):
#   1. Samples are centred on their mean before fitting. Controls fits around zero, which is
#      ill-conditioned (and fails) when the hard-iron offset is larger than the field itself -
#      as happened on the rover. Centring makes the fit well-conditioned in single precision.
#   2. Least squares is solved with the 9x9 normal equations (sum of d d^T) built one sample at
#      a time, so memory use does not grow with the number of samples.
#   3. sqrt(Q) uses a small Jacobi eigen-solver for 3x3 symmetric matrices.
# The output file has the same columns and meaning as Controls, so imu_cal_live can use it.

import math


# ============================================================================ small linear algebra
def solve(a, b):
    """Solve a x = b (a: n x n list of lists, b: list) by Gaussian elimination with partial pivoting.

    Returns x, or None if the matrix is singular (the samples did not cover enough directions).
    `a` and `b` are copied, not changed.
    """
    n = len(b)
    m = [list(a[i]) + [b[i]] for i in range(n)]     # Augmented matrix [a | b]
    biggest = max(abs(m[i][i]) for i in range(n)) or 1.0
    for col in range(n):
        # Pick the row with the largest value in this column (keeps rounding errors small)
        pivot = max(range(col, n), key=lambda r: abs(m[r][col]))
        if abs(m[pivot][col]) < 1e-6 * biggest:
            return None
        m[col], m[pivot] = m[pivot], m[col]
        # Eliminate this column from every row below
        for r in range(col + 1, n):
            factor = m[r][col] / m[col][col]
            if factor:
                row, prow = m[r], m[col]
                for k in range(col, n + 1):
                    row[k] -= factor * prow[k]
    # Back substitution
    x = [0.0] * n
    for r in range(n - 1, -1, -1):
        s = m[r][n] - sum(m[r][k] * x[k] for k in range(r + 1, n))
        x[r] = s / m[r][r]
    return x


def eig_sym3(a, sweeps=50):
    """Eigenvalues and eigenvectors of a 3x3 symmetric matrix (cyclic Jacobi rotations).

    Returns (values, vectors) where vectors[i][k] is component i of eigenvector k
    (eigenvectors are the COLUMNS, like numpy.linalg.eigh).
    """
    a = [list(r) for r in a]
    v = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    for _ in range(sweeps):
        off = a[0][1] ** 2 + a[0][2] ** 2 + a[1][2] ** 2
        if off < 1e-20 * (a[0][0] ** 2 + a[1][1] ** 2 + a[2][2] ** 2 + 1e-30):
            break
        for p, q in ((0, 1), (0, 2), (1, 2)):
            if abs(a[p][q]) < 1e-30:
                continue
            # Rotation angle that zeroes a[p][q]
            theta = (a[q][q] - a[p][p]) / (2.0 * a[p][q])
            t = (1.0 if theta >= 0 else -1.0) / (abs(theta) + math.sqrt(theta * theta + 1.0))
            c = 1.0 / math.sqrt(t * t + 1.0)
            s = t * c
            for k in range(3):                          # a = a J  (columns p, q)
                akp, akq = a[k][p], a[k][q]
                a[k][p], a[k][q] = c * akp - s * akq, s * akp + c * akq
            for k in range(3):                          # a = J^T a (rows p, q)
                apk, aqk = a[p][k], a[q][k]
                a[p][k], a[q][k] = c * apk - s * aqk, s * apk + c * aqk
            for k in range(3):                          # v = v J
                vkp, vkq = v[k][p], v[k][q]
                v[k][p], v[k][q] = c * vkp - s * vkq, s * vkp + c * vkq
    return [a[0][0], a[1][1], a[2][2]], v


def matvec3(m, x):
    return [m[0][0] * x[0] + m[0][1] * x[1] + m[0][2] * x[2],
            m[1][0] * x[0] + m[1][1] * x[1] + m[1][2] * x[2],
            m[2][0] * x[0] + m[2][1] * x[1] + m[2][2] * x[2]]


# ============================================================================ the fit
def thin(xs, ys, zs, min_step):
    """Keep a sample only if it is at least min_step (T) from the last kept one.

    While the IMU is held still (before you start rotating, or a pause) hundreds of samples land
    on one spot; least squares then fits that spot instead of the whole ellipsoid. Thinning makes
    the fit depend on the directions covered, not on how long each was held.
    """
    from array import array
    kx, ky, kz = array("f"), array("f"), array("f")
    step2 = min_step * min_step
    for i in range(len(xs)):
        if kx and (xs[i] - kx[-1]) ** 2 + (ys[i] - ky[-1]) ** 2 + (zs[i] - kz[-1]) ** 2 < step2:
            continue
        kx.append(xs[i])
        ky.append(ys[i])
        kz.append(zs[i])
    return kx, ky, kz


def fit_calibration(xs, ys, zs, max_spread=0.15, min_samples=30):
    """Ellipsoid fit of magnetometer samples (tesla). xs, ys, zs: equal-length sequences.

    Returns (offsets, matrix, report):
      offsets: [ox, oy, oz] in tesla (hard iron)
      matrix : 3x3 list (soft iron), calibrated = matrix (raw - offsets)
      report : dict with field strength, spread and sample count, for printing
    Raises ValueError with an explanation when the data cannot give a good calibration -
    the same checks and messages as Controls calibrate_mag.py.
    """
    n = len(xs)
    if n < min_samples:
        raise ValueError("At least %d finite magnetometer samples are required (got %d)." % (min_samples, n))

    # --- Centre and scale: u = (raw - mean) / scale, so the numbers are all about +-1.
    # The scale is the mean distance from the mean, summed in a loop: a list of every distance
    # (as Controls' median uses) needs one 16-byte float object per sample, and after a 60 s
    # recording the Pico's fragmented RAM could not fit it (MemoryError). The scale only keeps the
    # numbers near 1, so the mean works as well as the median.
    mean = [sum(xs) / n, sum(ys) / n, sum(zs) / n]
    scale = 0.0
    for i in range(n):
        scale += math.sqrt((xs[i] - mean[0]) ** 2 + (ys[i] - mean[1]) ** 2 + (zs[i] - mean[2]) ** 2)
    scale /= n
    if not scale > 0.0:
        raise ValueError("Magnetometer samples have no usable magnitude (did the sensor move?).")

    # --- Least squares for p = [A B C D E F G H I] in
    #     A x^2 + B y^2 + C z^2 + 2D xy + 2E xz + 2F yz + G x + H y + I z = 1
    # Normal equations: (sum d d^T) p = sum d, with d the 9 terms of one sample.
    ata = [[0.0] * 9 for _ in range(9)]
    atb = [0.0] * 9
    for i in range(n):
        x = (xs[i] - mean[0]) / scale
        y = (ys[i] - mean[1]) / scale
        z = (zs[i] - mean[2]) / scale
        d = (x * x, y * y, z * z, 2 * x * y, 2 * x * z, 2 * y * z, x, y, z)
        for r in range(9):
            dr = d[r]
            atb[r] += dr
            row = ata[r]
            for c in range(r, 9):                       # Upper triangle only; mirrored below
                row[c] += dr * d[c]
    for r in range(9):
        for c in range(r):
            ata[r][c] = ata[c][r]

    p = solve(ata, atb)
    if p is None:
        raise ValueError("Magnetometer samples do not cover enough directions for a 3D fit.")
    a, b, c, d, e, f, g, h, i = p
    quad = [[a, d, e], [d, b, f], [e, f, c]]
    lin = [g, h, i]

    # --- Centre of the ellipsoid (in u units): c = -0.5 Q^-1 L
    cu = solve(quad, [-0.5 * lin[0], -0.5 * lin[1], -0.5 * lin[2]])
    if cu is None:
        raise ValueError("Could not fit a bounded ellipsoid. Rotate the sensor through "
                         "all orientations and record a new dataset.")
    # Completing the square: (u - c)' Q (u - c) = 1 + c' Q c = r2
    qc = matvec3(quad, cu)
    r2 = 1.0 + cu[0] * qc[0] + cu[1] * qc[1] + cu[2] * qc[2]
    if not r2 > 0.0:
        raise ValueError("The fitted magnetometer ellipsoid is invalid.")
    shape = [[quad[r][k] / r2 for k in range(3)] for r in range(3)]     # (u-c)' shape (u-c) = 1

    values, vectors = eig_sym3(shape)
    if min(values) <= 0.0:
        raise ValueError("Could not fit a bounded ellipsoid. Rotate the sensor through "
                         "all orientations and record a new dataset.")

    # --- Matrix: sqrt(shape) maps the ellipsoid to a unit sphere (in u units). Multiplying by the
    #     geometric mean of the ellipsoid's semi-axes keeps the field length in u units, and the
    #     scale cancels when it is applied to raw tesla values: |M (raw - offset)| = field (T).
    semi_axes_mean = 1.0
    for v in values:
        semi_axes_mean *= 1.0 / math.sqrt(v)
    semi_axes_mean = semi_axes_mean ** (1.0 / 3.0)
    matrix = [[sum(vectors[r][k] * math.sqrt(values[k]) * vectors[col][k] for k in range(3)) * semi_axes_mean
               for col in range(3)] for r in range(3)]
    offsets = [mean[k] + scale * cu[k] for k in range(3)]

    # --- Quality check (same rule as Controls): calibrated lengths must be nearly constant
    total = total_sq = 0.0
    for idx in range(n):
        v = matvec3(matrix, [xs[idx] - offsets[0], ys[idx] - offsets[1], zs[idx] - offsets[2]])
        length = math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
        total += length
        total_sq += length * length
    mean_length = total / n
    spread = math.sqrt(max(total_sq / n - mean_length * mean_length, 0.0)) / mean_length if mean_length > 0 else 1.0
    if mean_length <= 0.0 or spread > max_spread:
        raise ValueError("The ellipsoid fit is poor (spread %.2f > %.2f). Record a slower, more "
                         "complete rotation through all orientations." % (spread, max_spread))

    report = {"samples": n, "field_uT": mean_length * 1e6, "spread": spread,
              "semi_axes_uT": [scale * semi * 1e6 for semi in (1.0 / math.sqrt(v) for v in values)]}
    return offsets, matrix, report
