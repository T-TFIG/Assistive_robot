#!/usr/bin/env python3
import numpy as np
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from scipy.spatial import KDTree

# -----------------------------
# Shape — parallelogram
# -----------------------------
P = np.array([
    [0.0, 0.0],
    [2.0, 0.0],
    [3.0, 1.0],
    [1.0, 1.0]
])
theta = 0.2
R_true = np.array([
    [np.cos(theta), -np.sin(theta)],
    [np.sin(theta),  np.cos(theta)]
])
t_true = np.array([1.0, 0.5])
Q = (R_true @ P.T).T + t_true

# -----------------------------
# Helper functions
# -----------------------------
def find_closest_points(P, Q):
    tree = KDTree(Q)
    dist, idx = tree.query(P)
    return Q[idx], dist

def centroid(points):
    return np.mean(points, axis=0)

def center(points, c):
    return points - c

def covariance(Pc, Qc):
    return Pc.T @ Qc

def compute_error(P, Q):
    return np.mean(np.sum((P - Q) ** 2, axis=1))

# -----------------------------
# Draw helpers
# -----------------------------
def draw_shape(ax, pts, style, label):
    closed = np.vstack([pts, pts[0]])
    ax.plot(closed[:, 0], closed[:, 1], style, linewidth=2.5, label=label)

def render_frame(ax_left, ax_right,
                 P_orig, P_curr, Q, Q_match,
                 start_k, n_starts, it, error,
                 best_P, best_error, done=False):

    # ── Left ───────────────────────────────────────────────────
    ax_left.clear()
    draw_shape(ax_left, P_orig, 'b-',  "Original P")
    draw_shape(ax_left, Q,      'r-',  "Target Q")
    draw_shape(ax_left, P_curr, 'g--', "Aligned P")

    ax_left.scatter(P_orig[:, 0], P_orig[:, 1], color='blue',  s=40, alpha=0.35, zorder=5)
    ax_left.scatter(Q[:, 0],      Q[:, 1],      color='red',   s=40, zorder=5)
    ax_left.scatter(P_curr[:, 0], P_curr[:, 1], color='green', s=40, zorder=5)

    for i in range(len(P_curr)):
        ax_left.plot(
            [P_curr[i, 0], Q_match[i, 0]],
            [P_curr[i, 1], Q_match[i, 1]],
            'y-', linewidth=0.7, alpha=0.5
        )

    if done:
        ax_left.set_title(f"DONE — best error: {best_error:.6f}", fontweight='bold')
    else:
        ax_left.set_title(
            f"Start {start_k + 1}/{n_starts}  —  iter {it + 1}\n"
            f"Error: {error:.6f}"
        )
    ax_left.legend(loc='upper left')
    ax_left.axis('equal')

    # ── Right ──────────────────────────────────────────────────
    ax_right.clear()
    draw_shape(ax_right, Q, 'r-', "Target Q")
    ax_right.scatter(Q[:, 0], Q[:, 1], color='red', s=40, zorder=5)

    if best_P is not None:
        draw_shape(ax_right, best_P, 'b--', "Best aligned")
        ax_right.scatter(best_P[:, 0], best_P[:, 1], color='blue', s=40, zorder=5)

    title = f"Final best error: {best_error:.6f}" if done else f"Best error so far: {best_error:.6f}"
    ax_right.set_title(title, fontweight='bold' if done else 'normal')
    ax_right.legend(loc='upper left')
    ax_right.axis('equal')

# -----------------------------
# Kabsch
# -----------------------------
def kabsch(P_used, Q_used, p_c, q_c):
    Pc = center(P_used, p_c)
    Qc = center(Q_used, q_c)
    H  = covariance(Pc, Qc)
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T
    t = q_c - R @ p_c
    return R, t

# -----------------------------
# Run all ICP starts and collect frames
# -----------------------------
def run_all(P, Q, n_starts=30, max_iter=30, tol=0.001):
    best_error     = float('inf')
    best_R         = None
    best_t         = None
    best_P_aligned = None
    frames         = []
    it             = 0

    for i in range(n_starts):
        angle  = np.random.uniform(0, 2 * np.pi)
        R_init = np.array([
            [np.cos(angle), -np.sin(angle)],
            [np.sin(angle),  np.cos(angle)]
        ])
        t_init    = centroid(Q) - centroid(P)
        P_current = (R_init @ P.T).T + t_init
        R_total   = R_init.copy()
        t_total   = t_init.copy()
        prev_error = float('inf')

        for it in range(max_iter):
            q_star, dist = find_closest_points(P_current, Q)

            idx  = np.argsort(dist)
            keep = idx[:int(0.7 * len(idx))]
            P_used = P_current[keep]
            Q_used = q_star[keep]
            error  = np.mean(dist)

            frames.append({
                'P_curr':     P_current.copy(),
                'Q_match':    q_star.copy(),
                'start_k':    i,
                'it':         it,
                'error':      error,
                'best_P':     best_P_aligned.copy() if best_P_aligned is not None else None,
                'best_error': best_error,
                'done':       False,
            })

            if abs(prev_error - error) < 1e-6:
                break
            prev_error = error

            p_c = centroid(P_used)
            q_c = centroid(Q_used)
            R, t = kabsch(P_used, Q_used, p_c, q_c)

            P_current = (R @ P_current.T).T + t
            R_total   = R @ R_total
            t_total   = R @ t_total + t

        final_error = compute_error(P_current, q_star)
        print(f"Start {i + 1:02d} → error: {final_error:.6f}")

        # FIX 1: update best BEFORE building the end-of-start frame
        if final_error < best_error:
            best_error     = final_error
            best_R         = R_total
            best_t         = t_total
            best_P_aligned = P_current.copy()

        # FIX 2: one extra frame per start that captures the updated best
        frames.append({
            'P_curr':     P_current.copy(),
            'Q_match':    q_star.copy(),
            'start_k':    i,
            'it':         it,
            'error':      final_error,
            'best_P':     best_P_aligned.copy(),
            'best_error': best_error,
            'done':       False,
        })

        if final_error <= tol:
            print("Tolerance reached — stopping early.")
            break

    # FIX 3: done frame appended AFTER the full loop so best_P_aligned
    #         is always the definitive final best
    frames.append({
        'P_curr':     best_P_aligned.copy(),
        'Q_match':    find_closest_points(best_P_aligned, Q)[0],
        'start_k':    n_starts - 1,
        'it':         0,
        'error':      best_error,
        'best_P':     best_P_aligned.copy(),
        'best_error': best_error,
        'done':       True,
    })

    return frames, best_R, best_t, best_error

# -----------------------------
# Main: display + record
# -----------------------------
def icp_visual_record(P, Q, n_starts=30, tol=0.001,
                      output_file="icp_recording.mp4", fps=15):

    print("Running ICP (collecting frames)...")
    frames, best_R, best_t, best_error = run_all(P, Q, n_starts=n_starts, tol=tol)
    print(f"Total frames collected: {len(frames)}")

    fig, (ax_left, ax_right) = plt.subplots(1, 2, figsize=(13, 5))
    fig.tight_layout(pad=3)

    # ── Live display ───────────────────────────────────────────
    plt.show(block=False)
    plt.ion()

    for f in frames:
        render_frame(ax_left, ax_right,
                     P, f['P_curr'], Q, f['Q_match'],
                     f['start_k'], n_starts, f['it'], f['error'],
                     f['best_P'], f['best_error'], done=f['done'])
        plt.gcf().canvas.draw()
        plt.gcf().canvas.flush_events()
        plt.pause(0.05)

    plt.ioff()

    # ── Video recording ────────────────────────────────────────
    print(f"\nSaving video to '{output_file}' ...")

    fig2, (ax2_left, ax2_right) = plt.subplots(1, 2, figsize=(13, 5))
    fig2.tight_layout(pad=3)

    def animate(idx):
        f = frames[idx]
        render_frame(ax2_left, ax2_right,
                     P, f['P_curr'], Q, f['Q_match'],
                     f['start_k'], n_starts, f['it'], f['error'],
                     f['best_P'], f['best_error'], done=f['done'])

    ani = animation.FuncAnimation(
        fig2, animate,
        frames=len(frames),
        interval=1000 // fps,
        repeat=False
    )

    try:
        writer = animation.FFMpegWriter(fps=fps, bitrate=1800)
        ani.save(output_file, writer=writer)
        print(f"Saved: {output_file}")
    except Exception as e:
        print(f"ffmpeg not available ({e}), saving as GIF instead...")
        gif_file = output_file.replace(".mp4", ".gif")
        ani.save(gif_file, writer="pillow", fps=fps)
        print(f"Saved: {gif_file}")

    plt.show()

    # ── Summary ────────────────────────────────────────────────
    print("\n--- Ground Truth ---")
    print("R:\n", R_true)
    print("t:\n", t_true)
    print("\n--- Estimated ---")
    print("R:\n", best_R)
    print("t:\n", best_t)
    print("Final error:", best_error)

# -----------------------------
# Run
# -----------------------------
if __name__ == "__main__":
    icp_visual_record(P, Q, n_starts=30, output_file="icp_recording.mp4", fps=15)