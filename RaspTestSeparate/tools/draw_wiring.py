#!/usr/bin/env python3
"""Draws docs/wiring_power_bank.png: Pico H + Pimoroni ICM-20948 + 100 ohm dummy load + power bank."""
import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, Rectangle

LEFT = ["GP0", "GP1", "GND", "GP2", "GP3", "GP4", "GP5", "GND", "GP6", "GP7",
        "GP8", "GP9", "GND", "GP10", "GP11", "GP12", "GP13", "GND", "GP14", "GP15"]          # pins 1-20
RIGHT = ["VBUS", "VSYS", "GND", "3V3_EN", "3V3(OUT)", "ADC_VREF", "GP28", "GND", "GP27", "GP26",
         "RUN", "GP22", "GND", "GP21", "GP20", "GP19", "GP18", "GND", "GP17", "GP16"]        # pins 40-21

fig, ax = plt.subplots(figsize=(14, 12.5))
ax.set_xlim(0, 14)
ax.set_ylim(0, 13)
ax.axis("off")

# --- Pico H board (USB at the top)
bx, by, bw, bh = 5.0, 2.0, 3.8, 8.6
ax.add_patch(FancyBboxPatch((bx, by), bw, bh, boxstyle="round,pad=0.05", fc="#2e7d32", ec="black", lw=1.5))
ax.add_patch(Rectangle((bx + bw / 2 - 0.45, by + bh - 0.05), 0.9, 0.45, fc="#b0b0b0", ec="black"))
ax.text(bx + bw / 2, by + bh / 2, "Raspberry Pi Pico H", ha="center", va="center", color="white",
        fontsize=15, fontweight="bold", rotation=90, alpha=0.9)
ax.text(bx + bw / 2, by + bh + 0.55, "micro-USB", ha="center", fontsize=9)
pitch = (bh - 0.6) / 19
pin_y = lambda i: by + bh - 0.3 - i * pitch          # i = 0..19 from the top
left_x, right_x = bx + 0.18, bx + bw - 0.18
used = {3: "#000000", 4: "#1565c0", 5: "#f9a825", 36: "#d32f2f", 38: "#000000", 40: "#d32f2f"}
for i, name in enumerate(LEFT):
    n = i + 1
    hl = n in used
    ax.plot(left_x, pin_y(i), "o", ms=7, color="gold" if hl else "#c8c8c8", mec="black")
    ax.text(left_x + 0.2, pin_y(i), "%d %s" % (n, name), va="center", fontsize=8,
            color="white", fontweight="bold" if hl else "normal")
for i, name in enumerate(RIGHT):
    n = 40 - i
    hl = n in used
    ax.plot(right_x, pin_y(i), "o", ms=7, color="gold" if hl else "#c8c8c8", mec="black")
    ax.text(right_x - 0.2, pin_y(i), "%s %d" % (name, n), va="center", ha="right", fontsize=8,
            color="white", fontweight="bold" if hl else "normal")

# --- Pimoroni ICM-20948 breakout (left)
ix, iy, iw, ih = 0.4, 6.8, 2.4, 3.0
ax.add_patch(FancyBboxPatch((ix, iy), iw, ih, boxstyle="round,pad=0.05", fc="#212121", ec="black", lw=1.5))
ax.text(ix + iw / 2, iy + ih - 0.35, "Pimoroni\nICM-20948", ha="center", va="top", color="white",
        fontsize=11, fontweight="bold")
imu_pins = [("3-6V", "#d32f2f", 36), ("SDA", "#1565c0", 4), ("SCL", "#f9a825", 5), ("INT", None, None),
            ("GND", "#000000", 3)]
for k, (name, colour, pico_pin) in enumerate(imu_pins):
    y = iy + 1.45 - k * 0.3
    px = ix + iw - 0.15
    ax.plot(px, y, "o", ms=7, color="gold" if colour else "#777777", mec="black")
    ax.text(px - 0.2, y, name, ha="right", va="center", color="white", fontsize=9)
    if not colour:
        ax.text(px + 0.25, y, "not used", va="center", fontsize=8, color="gray")
        continue
    target_y = pin_y(pico_pin - 1) if pico_pin <= 20 else pin_y(40 - pico_pin)
    if pico_pin <= 20:
        mid = 3.6 + 0.25 * k
        ax.plot([px, mid, mid, left_x], [y, y, target_y, target_y], color=colour, lw=2.5)
    else:   # 3V3(OUT) pin 36 on the right side: route over the top of the board
        top = by + bh + 1.2
        ax.plot([px, 3.3, 3.3, right_x + 1.0, right_x + 1.0, right_x],
                [y, y, top, top, target_y, target_y], color=colour, lw=2.5)

# --- Dummy load: 100 ohm between VBUS (40) and GND (38), drawn as a zigzag on the right
y40, y38 = pin_y(0), pin_y(2)
rx = right_x + 2.0
ax.plot([right_x, rx], [y40, y40], color="#d32f2f", lw=2.5)
ax.plot([right_x, rx], [y38, y38], color="black", lw=2.5)
import numpy as np
zz_y = np.linspace(y40 - 0.12, y38 + 0.12, 9)
zz_x = rx + np.array([0, 0.15, -0.15, 0.15, -0.15, 0.15, -0.15, 0.15, 0])
ax.plot([rx, rx], [y40, y40 - 0.12], color="#6d4c41", lw=2.5)
ax.plot(zz_x, zz_y, color="#6d4c41", lw=2.5)
ax.plot([rx, rx], [y38 + 0.12, y38], color="#6d4c41", lw=2.5)
ax.text(rx + 0.35, (y40 + y38) / 2, "100 Ω, ≥ 0.5 W\n(dummy load, ~50 mA)\nVBUS 40 ↔ GND 38",
        va="center", fontsize=10, color="#6d4c41", fontweight="bold")

# --- Power bank + USB cable
px0, py0 = 9.8, 11.0
ax.add_patch(FancyBboxPatch((px0, py0), 3.6, 1.4, boxstyle="round,pad=0.08", fc="#455a64", ec="black", lw=1.5))
ax.text(px0 + 1.8, py0 + 0.7, "USB power bank", ha="center", va="center", color="white", fontsize=12,
        fontweight="bold")
ax.plot([px0, bx + bw / 2, bx + bw / 2], [py0 + 0.7, py0 + 0.7, by + bh + 0.4], color="#555555", lw=4)
ax.text((px0 + bx + bw / 2) / 2 + 0.6, py0 + 0.85, "USB cable (5 V)", ha="center", fontsize=9)

# --- Legend / notes
notes = ("IMU 3-6V  → 3V3(OUT) pin 36      IMU SDA → GP2 pin 4      IMU SCL → GP3 pin 5\n"
         "IMU GND   → GND pin 3 (any GND pin works: 3, 8, 13, 18, 23, 28, 33, 38)      IMU INT: not connected\n"
         "Resistor only if the power bank switches off by itself: VBUS pin 40 ↔ GND pin 38. "
         "It gets warm (0.25 W); use 68 Ω (≥ 0.5 W) if 100 Ω is not enough.\n"
         "Not needed if the power bank has an 'always on' / low-current mode, or the LED still blinks after 2 minutes.")
ax.text(0.2, 0.05, notes, fontsize=9, va="bottom", family="monospace",
        bbox=dict(boxstyle="round", fc="#fffde7", ec="#bbbbbb"))
ax.set_title("Pico H IMU logger on a power bank: wiring with the dummy-load resistor", fontsize=14,
             fontweight="bold")
out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "docs", "wiring_power_bank.png")
fig.savefig(out, dpi=120, bbox_inches="tight")
print(os.path.abspath(out))
