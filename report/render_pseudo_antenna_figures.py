#!/usr/bin/env python3
"""Render pseudo-plot antenna figures for the Galuboy vertical-comm report.

Galuboy uses:
  - TX (both sides) : regular omnidirectional antenna (broad, ~2 dBi)
  - RX ground       : 8-element patch array
  - RX air          : 16-element patch array
  Each RX array can aim a main lobe at any user. The main-lobe peak gain
  as a function of steering angle is a smoothed rectangular window:
    flat in the FOV, raised-cosine taper at the edge, low floor outside.

Produces:
  report/figures/fig5_antenna_patterns.png        (3-panel: omni TX, 8-patch, 16-patch)
  report/figures/fig6_mrc_window_vs_steering.png  (window family vs FOV/taper)
"""
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

FIGS = Path("/Users/lev/galuboy-sim/report/figures")
FIGS.mkdir(parents=True, exist_ok=True)


def patch_main_lobe(theta_deg, N, fov_deg=60.0, taper_deg=20.0, floor_db=-20.0):
    """Peak main-lobe gain as a function of steering angle from broadside.

    Flat at G_max inside the FOV, raised-cosine taper at the edge, low
    floor outside. G_max = 10 log10 N is the coherent N-element gain
    over a single element.
    """
    G_max = 10.0 * np.log10(N)
    abs_th = np.abs(theta_deg)
    g = np.full_like(theta_deg, floor_db, dtype=float)

    flat_edge = fov_deg - taper_deg / 2.0
    roll_edge = fov_deg + taper_deg / 2.0

    mask_flat = abs_th <= flat_edge
    g[mask_flat] = G_max

    mask_taper = (abs_th > flat_edge) & (abs_th <= roll_edge)
    t = (abs_th[mask_taper] - flat_edge) / taper_deg
    g[mask_taper] = floor_db + (G_max - floor_db) * 0.5 * (1.0 + np.cos(np.pi * t))

    return g


def omni_pattern(el_deg, peak_dbi=2.0, floor_db=-3.0):
    """Regular omnidirectional antenna pattern (very broad, mild taper)."""
    # Mild cosine taper around horizon-symmetric pattern
    g = peak_dbi - 1.0 * np.sin(np.radians(el_deg)) ** 2
    return np.maximum(g, floor_db)


# =========================================================================
# Figure 5 — three-panel antenna gain summary
# =========================================================================

fig, axes = plt.subplots(1, 3, figsize=(15, 5), constrained_layout=True)
fig.suptitle(
    "Antenna gain patterns — Galuboy vertical-comm architecture",
    fontsize=15, fontweight="normal",
)

# (a) TX omni (both sides use the same physical antenna)
ax = axes[0]
el = np.linspace(-180, 180, 1441)
g_omni = omni_pattern(el, peak_dbi=2.0, floor_db=-3.0)
ax.plot(el, g_omni, lw=2.0, color="#1f77b4")
ax.set_xlim(-180, 180)
ax.set_ylim(-5, 5)
ax.set_xlabel("Elevation θ (deg)", fontsize=11)
ax.set_ylabel("Gain (dBi)", fontsize=11)
ax.set_title("(a) Omnidirectional TX antenna  —  peak ≈ 2 dBi", fontsize=12)
ax.grid(True, alpha=0.3)
ax.set_xticks(np.arange(-180, 181, 60))

# (b) Ground RX: 8-element patch main-lobe peak gain vs steering angle
ax = axes[1]
theta = np.linspace(-90, 90, 721)
g_8 = patch_main_lobe(theta, N=8, fov_deg=60.0, taper_deg=20.0)
ax.plot(theta, g_8, lw=2.0, color="#d7642d")
ax.axhline(10 * np.log10(8), ls=":", color="#666", lw=0.8)
ax.text(-85, 10 * np.log10(8) + 0.4, f"G_max = 10 log10(N) = {10*np.log10(8):.1f} dB",
        fontsize=9, color="#444")
ax.set_xlim(-90, 90)
ax.set_ylim(-22, 12)
ax.set_xlabel("Steering angle from broadside φ (deg)", fontsize=11)
ax.set_ylabel("Main-lobe peak gain (dB over single element)", fontsize=11)
ax.set_title("(b) Ground RX  —  8-element patch", fontsize=12)
ax.grid(True, alpha=0.3)
ax.set_xticks(np.arange(-90, 91, 30))

# (c) Air RX: 16-element patch
ax = axes[2]
g_16 = patch_main_lobe(theta, N=16, fov_deg=60.0, taper_deg=20.0)
ax.plot(theta, g_16, lw=2.0, color="#d7642d")
ax.axhline(10 * np.log10(16), ls=":", color="#666", lw=0.8)
ax.text(-85, 10 * np.log10(16) + 0.4, f"G_max = 10 log10(N) = {10*np.log10(16):.1f} dB",
        fontsize=9, color="#444")
ax.set_xlim(-90, 90)
ax.set_ylim(-22, 14)
ax.set_xlabel("Steering angle from broadside φ (deg)", fontsize=11)
ax.set_ylabel("Main-lobe peak gain (dB over single element)", fontsize=11)
ax.set_title("(c) Air RX  —  16-element patch", fontsize=12)
ax.grid(True, alpha=0.3)
ax.set_xticks(np.arange(-90, 91, 30))

fig.savefig(FIGS / "fig5_antenna_patterns.png", dpi=300, bbox_inches="tight",
            facecolor="white")
plt.close(fig)
print("wrote", FIGS / "fig5_antenna_patterns.png")


# =========================================================================
# Figure 6 — main-lobe gain family vs FOV/taper
# =========================================================================

fig, ax = plt.subplots(figsize=(11, 5.5), constrained_layout=True)
theta = np.linspace(-90, 90, 1801)
for fov, taper, label, color in [
    (45, 15, "N=8,  FOV=±45°, taper 15°",  "#1f77b4"),
    (60, 20, "N=8,  FOV=±60°, taper 20°",  "#d7642d"),
    (60, 20, "N=16, FOV=±60°, taper 20°",  "#2ca02c"),
    (75, 25, "N=16, FOV=±75°, taper 25°",  "#9467bd"),
]:
    N = 16 if "N=16" in label else 8
    g = patch_main_lobe(theta, N=N, fov_deg=fov, taper_deg=taper)
    ax.plot(theta, g, lw=2.0, color=color, label=label)

ax.set_xlim(-90, 90)
ax.set_ylim(-22, 14)
ax.set_xlabel("Steering angle from array broadside φ (deg)", fontsize=12)
ax.set_ylabel("Main-lobe peak gain (dB over single element)", fontsize=12)
ax.set_title(
    "Patch-array main-lobe gain as a function of steering direction "
    "— smoothed-rectangular-window model",
    fontsize=13, fontweight="normal",
)
ax.set_xticks(np.arange(-90, 91, 30))
ax.grid(True, alpha=0.3)
ax.legend(loc="lower center", fontsize=10, frameon=False, ncol=2)

fig.savefig(FIGS / "fig6_mrc_window_vs_steering.png", dpi=300, bbox_inches="tight",
            facecolor="white")
plt.close(fig)
print("wrote", FIGS / "fig6_mrc_window_vs_steering.png")
