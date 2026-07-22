#!/usr/bin/env python3
# Plot 2-hop sweep results (N=joiners 10..25) for thesis Ch5.
#   Inputs : repo-root dsme-beacon-slot-selection-{fixed-0dBm,fixed-5dBm,PC-schemeA}-2hop.txt
#   Outputs: plots/*_2hop.pdf + plots/*_2hop.png
#
# Style matches the reference 2x2 figure:
#   - Baseline 0 dBm : black,  circle,   solid
#   - Fixed -5 dBm   : gray,   square,   dashed
#   - Power Control  : blue,   triangle, dash-dot
#   - No error bars (clean lines + markers), serif fonts, dotted grid.
# Panel order (like reference): (TL) p_coll  (TR) s_coll  (BL) eta  (BR) Ptx

import os
import numpy as np
import matplotlib.pyplot as plt

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PLOT_DIR = os.path.join(ROOT, "plots")
os.makedirs(PLOT_DIR, exist_ok=True)

FILES = {
    "Baseline 0 dBm":  os.path.join(ROOT, "dsme-beacon-slot-selection-fixed-0dBm-2hop.txt"),
    "Fixed −5 dBm": os.path.join(ROOT, "dsme-beacon-slot-selection-fixed-5dBm-2hop.txt"),
    "Power Control":   os.path.join(ROOT, "dsme-beacon-slot-selection-PC-schemeA-2hop.txt"),
}

STYLE = {
    "Baseline 0 dBm":  dict(color="#000000", marker="o", linestyle="-",  linewidth=1.6, markersize=5),
    "Fixed −5 dBm": dict(color="#7f7f7f", marker="s", linestyle="--", linewidth=1.6, markersize=5),
    "Power Control":   dict(color="#1f77b4", marker="^", linestyle="-.", linewidth=1.7, markersize=6),
}

# cols: N  p_mean p_std  s_mean s_std  eta_mean eta_std  Ptx_mean Ptx_std
DATA = {label: np.loadtxt(p) for label, p in FILES.items()}

XLABEL = r"Number of joiners $N$"   # column 1 is joiners; total nodes = N + 1
XTICKS = range(10, 26, 5)           # 10, 15, 20, 25

plt.rcParams.update({
    "font.family": "serif",
    "font.size": 11,
    "axes.labelsize": 13,
    "legend.fontsize": 10,
    "xtick.labelsize": 11,
    "ytick.labelsize": 11,
    "axes.grid": True,
    "grid.alpha": 0.35,
    "grid.linestyle": ":",
})

def plot_metric(ax, ycol, only=None):
    for label in FILES:
        if only is not None and label not in only:
            continue
        d = DATA[label]
        ax.plot(d[:, 0], d[:, ycol], label=label, **STYLE[label])

def save(fig, stem):
    for ext in ("pdf", "png"):
        fig.savefig(os.path.join(PLOT_DIR, f"{stem}.{ext}"),
                    dpi=600 if ext == "png" else None, bbox_inches="tight")
    print(f"  -> plots/{stem}.pdf, plots/{stem}.png")

# ---------------- 2x2 summary panel (main deliverable) ----------------
fig, ((axA, axB), (axC, axD)) = plt.subplots(2, 2, figsize=(11.0, 8.0))

# (a) p_coll
plot_metric(axA, 1)
axA.set_ylabel(r"$p_{\mathrm{coll}}$")
axA.set_ylim(bottom=0)
axA.legend(loc="upper left", framealpha=0.95)

# (b) s_coll
plot_metric(axB, 3)
axB.set_ylabel(r"$\bar{s}_{\mathrm{coll}}$")
axB.set_ylim(bottom=0)
axB.legend(loc="upper left", framealpha=0.95)

# (c) eta
plot_metric(axC, 5)
axC.set_ylabel(r"$\eta$")
axC.set_ylim(0.3, 1.0)
axC.legend(loc="lower right", framealpha=0.95)

# (d) Ptx  (baseline/fixed as flat reference lines, PC as adaptive curve)
plot_metric(axD, 7, only=["Power Control"])
axD.axhline(0.0,  color="#000000", linestyle="-",  linewidth=1.2, marker="o", markersize=0)
axD.axhline(-5.0, color="#7f7f7f", linestyle="--", linewidth=1.2)
# proxy handles so the legend shows all three, matching the other panels
from matplotlib.lines import Line2D
handles = [
    Line2D([0], [0], **STYLE["Baseline 0 dBm"]),
    Line2D([0], [0], **STYLE["Fixed −5 dBm"]),
    Line2D([0], [0], **STYLE["Power Control"]),
]
axD.legend(handles, list(FILES.keys()), loc="center right", framealpha=0.95)
axD.set_ylabel(r"$\bar{P}_{\mathrm{tx}}$ (dBm)")
axD.set_ylim(-5.6, 0.5)

for ax in (axA, axB, axC, axD):
    ax.set_xlabel(XLABEL)
    ax.set_xticks(XTICKS)
    ax.set_xlim(10, 25)

fig.tight_layout()
save(fig, "fig_summary_2x2_2hop")
plt.close(fig)

# ---------------- individual single-panel figures ----------------
def single(stem, ycol, ylabel, legend_loc, ylim=None, only=None, ptx=False):
    fig, ax = plt.subplots(figsize=(6.2, 4.2))
    if ptx:
        plot_metric(ax, ycol, only=["Power Control"])
        ax.axhline(0.0,  color="#000000", linestyle="-",  linewidth=1.2)
        ax.axhline(-5.0, color="#7f7f7f", linestyle="--", linewidth=1.2)
        from matplotlib.lines import Line2D
        handles = [Line2D([0], [0], **STYLE[k]) for k in FILES]
        ax.legend(handles, list(FILES.keys()), loc=legend_loc, framealpha=0.95)
    else:
        plot_metric(ax, ycol, only=only)
        ax.legend(loc=legend_loc, framealpha=0.95)
    ax.set_xlabel(XLABEL)
    ax.set_ylabel(ylabel)
    ax.set_xticks(XTICKS)
    ax.set_xlim(10, 25)
    if ylim is not None:
        ax.set_ylim(*ylim)
    fig.tight_layout()
    save(fig, stem)
    plt.close(fig)

single("fig_pcoll_2hop",  1, r"$p_{\mathrm{coll}}$",              "upper left",  ylim=(0, None))
single("fig_scoll_2hop",  3, r"$\bar{s}_{\mathrm{coll}}$",        "upper left",  ylim=(0, None))
single("fig_eta_2hop",    5, r"$\eta$",                           "lower right", ylim=(0.3, 1.0))
single("fig_ptx_2hop",    7, r"$\bar{P}_{\mathrm{tx}}$ (dBm)",    "center right", ylim=(-5.6, 0.5), ptx=True)

# ---------------- bonus: collision-free admitted coordinators ----------------
# joined_nocoll = eta * joiners * (1 - p_coll) : single number combining eta & p_coll
fig, ax = plt.subplots(figsize=(6.2, 4.2))
for label in FILES:
    d = DATA[label]
    y = d[:, 5] * d[:, 0] * (1.0 - d[:, 1])
    ax.plot(d[:, 0], y, label=label, **STYLE[label])
ax.set_xlabel(XLABEL)
ax.set_ylabel(r"Collision-free admitted coordinators")
ax.set_xticks(XTICKS)
ax.set_xlim(10, 25)
ax.legend(loc="upper left", framealpha=0.95)
fig.tight_layout()
save(fig, "fig_collisionfree_2hop")
plt.close(fig)

print("\nAll 2-hop figures written to:", PLOT_DIR)
