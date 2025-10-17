"""
uwb_simulation_fixed.py

Simulated UWB indoor localization animation using 4 anchors.
- Blue dot: true drone position
- Green dot: estimated position from noisy UWB ranges
- Red Xs: anchors

This version uses FigureCanvasAgg.buffer_rgba() to extract images from matplotlib
so it doesn't call tostring_rgb() on a backend that doesn't provide it.

If imageio is available the script saves 'uwb_simulation.gif'. If imageio is NOT
available, the script will show an interactive matplotlib animation window instead.
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas

# --- Try to import imageio for GIF saving; handle gracefully if missing ---
try:
    import imageio
    HAVE_IMAGEIO = True
except Exception:
    HAVE_IMAGEIO = False

# ----------------- SIMULATION PARAMETERS -----------------
np.random.seed(42)
num_steps = 200            # number of animation frames
noise_std = 0.05           # UWB range noise (meters)
anchors = np.array([
    [0.0, 0.0],
    [6.0, 0.5],
    [6.0, 5.5],
    [0.0, 6.0]
])                         # anchors positions in meters

# Drone path (parametric, smooth looping)
t = np.linspace(0, 2 * np.pi, num_steps)
true_path = np.vstack((
    3.0 + 2.2 * np.cos(1.0 * t) + 0.6 * np.cos(3.0 * t),
    3.0 + 1.6 * np.sin(1.2 * t) + 0.5 * np.sin(2.5 * t)
)).T

out_gif = "uwb_simulation.gif"
fps = 20
# -------------------------------------------------------

def estimate_position_from_ranges(anchors, ranges):
    """
    Linearized multilateration (use anchor 0 as reference).
    Returns estimated (x, y).
    """
    x0, y0 = anchors[0]
    r0 = ranges[0]
    A = []
    b = []
    for i in range(1, len(anchors)):
        xi, yi = anchors[i]
        ri = ranges[i]
        A.append([2 * (x0 - xi), 2 * (y0 - yi)])
        b.append((x0**2 + y0**2 - r0**2) - (xi**2 + yi**2 - ri**2))
    A = np.array(A)
    b = np.array(b)
    sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    return float(sol[0]), float(sol[1])

# --- Build frames by rendering each matplotlib figure to a numpy array ---
frames = []
fig_w, fig_h = 6, 6
xlims = [-1, 7]
ylims = [-1, 7]
est_history = []

for k in range(num_steps):
    true_pos = true_path[k]
    true_ranges = np.linalg.norm(anchors - true_pos, axis=1)
    measured_ranges = true_ranges + np.random.normal(0, noise_std, size=true_ranges.shape)
    est_x, est_y = estimate_position_from_ranges(anchors, measured_ranges)
    est_history.append((est_x, est_y))

    # create figure
    fig, ax = plt.subplots(figsize=(fig_w, fig_h), dpi=100)
    ax.set_xlim(xlims)
    ax.set_ylim(ylims)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.set_title(f"UWB Localization — step {k+1}/{num_steps}  noise={noise_std:.3f} m")

    # anchors (red X)
    ax.scatter(anchors[:, 0], anchors[:, 1], marker='x', s=100, color='red', label='Anchors')
    for i, (axx, ayy) in enumerate(anchors):
        ax.text(axx + 0.06, ayy + 0.06, f"A{i+1}", fontsize=9)

    # true drone (blue)
    ax.plot(true_path[:k+1, 0], true_path[:k+1, 1], '-', linewidth=1, alpha=0.25)
    ax.scatter([true_pos[0]], [true_pos[1]], color='blue', s=60, label='True drone')

    # estimated (green)
    valid = [p for p in est_history if not (np.isnan(p[0]) or np.isnan(p[1]))]
    if valid:
        ax.plot([p[0] for p in valid], [p[1] for p in valid], '--', linewidth=1, alpha=0.6)
        ax.scatter([est_x], [est_y], color='green', s=60, label='Estimated')

    # measured ranges as small text near anchors
    for i, (axx, ayy) in enumerate(anchors):
        ax.text(axx + 0.08, ayy - 0.12, f"d{i+1}={measured_ranges[i]:.2f} m", fontsize=7)

    ax.legend(loc='upper right', fontsize=8)

    # --- Render to Agg canvas and grab pixel buffer (robust) ---
    canvas = FigureCanvas(fig)    # Agg canvas for this figure
    canvas.draw()
    # buffer_rgba returns an object that we can convert to ndarray reliably
    rgba = np.asarray(canvas.buffer_rgba())  # shape: (h, w, 4), dtype=uint8
    # convert to RGB (drop alpha). .copy() to ensure contiguous memory for imageio.
    image = rgba[:, :, :3].copy()
    frames.append(image)

    plt.close(fig)

# --- Save or display ---
if HAVE_IMAGEIO:
    try:
        imageio.mimsave(out_gif, frames, fps=fps)
        print(f"Saved GIF to {out_gif} ({len(frames)} frames, {fps} fps).")
    except Exception as e:
        print("Failed to save GIF with imageio:", e)
        print("You can still view frames by making a simple animation (see fallback).")
        HAVE_IMAGEIO = False

if not HAVE_IMAGEIO:
    # fallback: display a simple animation using matplotlib FuncAnimation
    print("imageio not available — showing interactive animation instead.")
    import matplotlib.animation as animation

    fig2, ax2 = plt.subplots(figsize=(fig_w, fig_h), dpi=100)
    ax2.set_xlim(xlims)
    ax2.set_ylim(ylims)
    ax2.set_aspect('equal')
    scat_true = ax2.scatter([], [], color='blue', s=60)
    scat_est = ax2.scatter([], [], color='green', s=60)
    ax2.scatter(anchors[:, 0], anchors[:, 1], marker='x', s=100, color='red')
    for i, (axx, ayy) in enumerate(anchors):
        ax2.text(axx + 0.06, ayy + 0.06, f"A{i+1}", fontsize=9)

    def init():
        scat_true.set_offsets([])
        scat_est.set_offsets([])
        return scat_true, scat_est

    def update_frame(i):
        true_p = true_path[i]
        est_p = est_history[i]
        scat_true.set_offsets([true_p])
        scat_est.set_offsets([est_p])
        ax2.set_title(f"UWB Localization — step {i+1}/{num_steps}")
        return scat_true, scat_est

    ani = animation.FuncAnimation(fig2, update_frame, frames=num_steps, init_func=init, blit=True, interval=1000/fps)
    plt.show()
