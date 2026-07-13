"""
Landing accuracy bar chart for 8-drone test run.
Replace the `errors` dict with your actual measurements from Table 6-3.
"""

import matplotlib.pyplot as plt
import numpy as np

# ============================================================
# REPLACE THESE WITH YOUR ACTUAL VALUES FROM TABLE 6-3
# ============================================================
errors = {
    'Drone 19-20': 2,
    'Drone 20-21': 14,
    'Drone 21-22': 4,
    'Drone 22-23': 0,
    'Drone 23-24': 1,
    'Drone 24-25': 0,
    'Drone 25-26': 3,
}
# ============================================================

# Slide colour palette (matches your deck)
BG_COLOR = '#0a3d6b'        # dark blue background
BAR_COLOR = '#5ba3d8'       # light blue bars
ACCENT = '#f5b942'          # yellow accent
ALERT = '#e74c3c'           # red for max
TEXT = '#ffffff'            # white text
GRID = '#1f5a8a'            # subtle grid

drones = list(errors.keys())
values = list(errors.values())

mean_err = np.mean(values)
max_err = np.max(values)
max_drone = drones[np.argmax(values)]

fig, ax = plt.subplots(figsize=(11, 6))
fig.patch.set_facecolor(BG_COLOR)
ax.set_facecolor(BG_COLOR)

# Bars - highlight max in red
bar_colors = [ALERT if v == max_err else BAR_COLOR for v in values]
bars = ax.bar(drones, values, color=bar_colors, edgecolor=TEXT, linewidth=0.8, zorder=3)

# Mean line
ax.axhline(mean_err, color=ACCENT, linestyle='--', linewidth=2, zorder=2,
           label=f'Mean: {mean_err:.1f} cm')

# Value labels on top of each bar
for bar, v in zip(bars, values):
    ax.text(bar.get_x() + bar.get_width() / 2, v + 0.2, f'{v:.1f}',
            ha='center', va='bottom', color=TEXT, fontsize=10, fontweight='bold')

# Mean label
ax.text(len(drones) - 0.5, mean_err + 0.3, f'Mean = {mean_err:.1f} cm',
        ha='right', va='bottom', color=ACCENT, fontsize=11, fontweight='bold')

# Max callout
ax.annotate(f'Max: {max_err:.1f} cm',
            xy=(max_drone, max_err),
            xytext=(max_drone, max_err + 2.5),
            ha='center', color=ALERT, fontsize=11, fontweight='bold',
            arrowprops=dict(arrowstyle='->', color=ALERT, lw=1.5))

# Styling
ax.set_ylabel('Landing Error (cm)', color=TEXT, fontsize=12, fontweight='bold')
ax.set_xlabel('')
ax.set_title('Landing Accuracy — 8-Drone Test', color=TEXT, fontsize=14,
             fontweight='bold', pad=15)

# Tick colours
ax.tick_params(axis='x', colors=TEXT, labelsize=10)
ax.tick_params(axis='y', colors=TEXT, labelsize=10)

# Grid
ax.yaxis.grid(True, color=GRID, linestyle='-', linewidth=0.5, zorder=1)
ax.set_axisbelow(True)

# Spines
for spine in ax.spines.values():
    spine.set_color(TEXT)
    spine.set_linewidth(0.8)
ax.spines['top'].set_visible(False)
ax.spines['right'].set_visible(False)

# Y-axis from 0 with headroom for max callout
ax.set_ylim(0, max_err + 5)

# Legend
legend = ax.legend(loc='upper left', frameon=False, fontsize=10,
                   labelcolor=TEXT)

plt.tight_layout()
plt.savefig('/home/claude/landing_accuracy.png', dpi=200,
            facecolor=BG_COLOR, bbox_inches='tight')
plt.savefig('/home/claude/landing_accuracy.pdf',
            facecolor=BG_COLOR, bbox_inches='tight')
print(f'Mean error: {mean_err:.2f} cm')
print(f'Max error: {max_err:.2f} cm ({max_drone})')
print(f'Min error: {min(values):.2f} cm')
print('Saved: landing_accuracy.png and landing_accuracy.pdf')