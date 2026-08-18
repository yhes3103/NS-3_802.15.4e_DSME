#!/usr/bin/env python3
# Plot sweep results for thesis Ch5
#   Inputs : repo-root *.txt produced by sweep-*.sh
#   Outputs: plots/*.pdf + plots/*.png
#
# Layout choices for thesis-quality figures:
#   - Vector PDF + 600 dpi PNG (so they survive both print and screen)
#   - B/W-friendly: distinct line styles + markers per curve
#   - Error bars = 1 std (capsize 3, alpha 0.7)
#   - Larger axis labels (12pt) + tick labels (11pt) + legend (11pt)
#   - Tight layout, no chartjunk

import os
import numpy as np
import matplotlib.pyplot as plt

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PLOT_DIR = os.path.join(ROOT, "plots")
os.makedirs(PLOT_DIR, exist_ok=True)

FILES = {
    "Baseline (0 dBm)":  os.path.join(ROOT, "dsme-beacon-slot-selection-fixed-0dBm.txt"),
    "Fixed (-5 dBm)":    os.path.join(ROOT, "dsme-beacon-slot-selection-fixed-5dBm.txt"),
    "PC (GPS-based)":    os.path.join(ROOT, "dsme-beacon-slot-selection-PC-schemeA.txt"),
}

# B/W-friendly: line style + marker + color all differ
STYLE = {
    "Baseline (0 dBm)":  dict(color="#1f77b4", marker="o", linestyle="-",  linewidth=1.6, markersize=6),
    "Fixed (-5 dBm)":    dict(color="#2ca02c", marker="s", linestyle="--", linewidth=1.6, markersize=6),
    "PC (GPS-based)":    dict(color="#d62728", marker="^", linestyle="-.", linewidth=1.8, markersize=7),
}

def load(path):
    # cols: N  p_mean p_std  s_mean s_std  eta_mean eta_std  Ptx_mean Ptx_std
    return np.loadtxt(path)

DATA = {label: load(p) for label, p in FILES.items()}

plt.rcParams.update({
    "font.family": "serif",
    "font.size": 11,
    "axes.labelsize": 12,
    "axes.titlesize": 12,
    "legend.fontsize": 11,
    "xtick.labelsize": 11,
    "ytick.labelsize": 11,
    "axes.grid": True,
    "grid.alpha": 0.3,
    "grid.linestyle": ":",
})

def errbar(ax, label, ycol_mean, ycol_std):
    d = DATA[label]
    ax.errorbar(d[:, 0], d[:, ycol_mean], yerr=d[:, ycol_std],
                label=label, capsize=3, alpha=0.9, **STYLE[label])

def save(fig, stem):
    for ext in ("pdf", "png"):
        fig.savefig(os.path.join(PLOT_DIR, f"{stem}.{ext}"),
                    dpi=600 if ext == "png" else None,
                    bbox_inches="tight")
    print(f"  -> {stem}.pdf, {stem}.png")

# ---------- Figure 1: p_coll vs N ----------
fig, ax = plt.subplots(figsize=(6.0, 4.0))
for label in FILES:
    errbar(ax, label, 1, 2)
ax.set_xlabel("Number of joiners $N$")
ax.set_ylabel(r"Collision probability $p_{\mathrm{coll}}$")
ax.set_xticks(range(10, 51, 5))
ax.set_ylim(bottom=0)
ax.legend(loc="upper left", framealpha=0.95)
fig.tight_layout()
save(fig, "fig_pcoll_vs_n")
plt.close(fig)

# ---------- Figure 2: eta vs N ----------
fig, ax = plt.subplots(figsize=(6.0, 4.0))
for label in FILES:
    errbar(ax, label, 5, 6)
ax.set_xlabel("Number of joiners $N$")
ax.set_ylabel(r"Join success rate $\eta$")
ax.set_xticks(range(10, 51, 5))
ax.set_ylim(0, 1.05)
ax.legend(loc="lower right", framealpha=0.95)
fig.tight_layout()
save(fig, "fig_eta_vs_n")
plt.close(fig)

# ---------- Figure 3: PC P_tx vs N ----------
fig, ax = plt.subplots(figsize=(6.0, 4.0))
d = DATA["PC (GPS-based)"]
ax.errorbar(d[:, 0], d[:, 7], yerr=d[:, 8],
            label="PC (GPS-based)", capsize=3, **STYLE["PC (GPS-based)"])
# reference lines
ax.axhline(0.0,  color="#1f77b4", linestyle="-",  linewidth=1.0, alpha=0.7,
           label="Baseline (0 dBm)")
ax.axhline(-5.0, color="#2ca02c", linestyle="--", linewidth=1.0, alpha=0.7,
           label="Fixed (-5 dBm)")
ax.set_xlabel("Number of joiners $N$")
ax.set_ylabel(r"Average TX power $\bar{P}_{\mathrm{tx}}$ (dBm)")
ax.set_xticks(range(10, 51, 5))
ax.legend(loc="upper right", framealpha=0.95)
fig.tight_layout()
save(fig, "fig_ptx_vs_n")
plt.close(fig)

# ---------- Figure 4: s_coll vs N (appendix) ----------
fig, ax = plt.subplots(figsize=(6.0, 4.0))
for label in FILES:
    errbar(ax, label, 3, 4)
ax.set_xlabel("Number of joiners $N$")
ax.set_ylabel(r"Collision severity $\bar{s}_{\mathrm{coll}}$")
ax.set_xticks(range(10, 51, 5))
ax.set_ylim(bottom=0)
ax.legend(loc="upper left", framealpha=0.95)
fig.tight_layout()
save(fig, "fig_scoll_vs_n")
plt.close(fig)

# ---------- Figure 5: 2x2 panel summary ----------
fig, axes = plt.subplots(2, 2, figsize=(11.0, 7.5))
((axA, axB), (axC, axD)) = axes

for label in FILES:
    d = DATA[label]
    axA.errorbar(d[:, 0], d[:, 1], yerr=d[:, 2], label=label, capsize=3, **STYLE[label])
    axB.errorbar(d[:, 0], d[:, 5], yerr=d[:, 6], label=label, capsize=3, **STYLE[label])
    axC.errorbar(d[:, 0], d[:, 3], yerr=d[:, 4], label=label, capsize=3, **STYLE[label])

dPC = DATA["PC (GPS-based)"]
axD.errorbar(dPC[:, 0], dPC[:, 7], yerr=dPC[:, 8],
             label="PC (GPS-based)", capsize=3, **STYLE["PC (GPS-based)"])
axD.axhline(0.0,  color="#1f77b4", linestyle="-",  linewidth=1.0, alpha=0.7, label="Baseline (0 dBm)")
axD.axhline(-5.0, color="#2ca02c", linestyle="--", linewidth=1.0, alpha=0.7, label="Fixed (-5 dBm)")

for ax in (axA, axB, axC, axD):
    ax.set_xticks(range(10, 51, 5))
    ax.set_xlabel("Number of joiners $N$")

axA.set_ylabel(r"$p_{\mathrm{coll}}$");          axA.set_title("(a) Collision probability");        axA.set_ylim(bottom=0); axA.legend(loc="upper left")
axB.set_ylabel(r"$\eta$");                       axB.set_title("(b) Join success rate");            axB.set_ylim(0, 1.05);  axB.legend(loc="lower right")
axC.set_ylabel(r"$\bar{s}_{\mathrm{coll}}$");    axC.set_title("(c) Collision severity");           axC.set_ylim(bottom=0); axC.legend(loc="upper left")
axD.set_ylabel(r"$\bar{P}_{\mathrm{tx}}$ (dBm)");axD.set_title("(d) Average TX power (PC adapts)"); axD.legend(loc="upper right")

fig.tight_layout()
save(fig, "fig_summary_2x2")
plt.close(fig)

print("\nAll figures written to:", PLOT_DIR)
