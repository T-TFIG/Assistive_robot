#!/usr/bin/env python3
import numpy as np
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from scipy.spatial import KDTree

# -----------------------------
# Shape
# -----------------------------
P = np.array([
    [0.0, 0.0],
    [2.0, 0.0],
    [3.0, 1.0],
    [1.0, 1.0]
])

theta = 0.5
R_true = np.array([
    [np.cos(theta), -np.sin(theta)],
    [np.sin(theta),  np.cos(theta)]
])
t_true = np.array([1.0, 0.5])

Q = (R_true @ P.T).T + t_true


# -----------------------------
# Cost
# -----------------------------
def compute_cost(P, Q):
    return np.mean(np.sum((P - Q) ** 2, axis=1))


# -----------------------------
# Draw
# -----------------------------
def draw_shape(ax, pts, style, label):
    closed = np.vstack([pts, pts[0]])
    ax.plot(closed[:, 0], closed[:, 1], style, linewidth=3, label=label)


def plot_step(ax, P_orig, P_curr, Q, Q_match, it, cost, mode):
    ax.clear()

    draw_shape(ax, P_orig, 'b-', "Original P")
    draw_shape(ax, Q, 'r-', "Target Q")
    draw_shape(ax, P_curr, 'g--', "Aligned P")

    ax.scatter(P_orig[:,0], P_orig[:,1], color='blue', alpha=0.3)
    ax.scatter(Q[:,0], Q[:,1], color='red')
    ax.scatter(P_curr[:,0], P_curr[:,1], color='green')

    if mode == "multi":
        for i in range(len(P_curr)):
            ax.plot(
                [P_curr[i,0], Q_match[i,0]],
                [P_curr[i,1], Q_match[i,1]],
                'y--', alpha=0.6
            )

    elif mode == "single":
        i = 0
        ax.plot(
            [P_curr[i,0], Q_match[i,0]],
            [P_curr[i,1], Q_match[i,1]],
            'y-', linewidth=3
        )
        ax.scatter(P_curr[i,0], P_curr[i,1], color='lime', s=120)
        ax.scatter(Q_match[i,0], Q_match[i,1], color='orange', s=120)

    ax.set_title(f"{mode.upper()} ICP | Iter {it+1} | Cost {cost:.6f}")
    ax.axis("equal")
    ax.legend()

    plt.pause(0.6)


# -----------------------------
# Kabsch
# -----------------------------
def kabsch(P_pts, Q_pts):
    cP = np.mean(P_pts, axis=0)
    cQ = np.mean(Q_pts, axis=0)

    P_c = P_pts - cP
    Q_c = Q_pts - cQ

    H = P_c.T @ Q_c

    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T

    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T

    t = cQ - R @ cP
    return R, t


# -----------------------------
# ICP
# -----------------------------
def icp_visual(P, Q, mode="multi", max_iter=20):
    P_aligned = P.copy()
    P_orig = P.copy()

    plt.ion()
    fig, ax = plt.subplots()

    for it in range(max_iter):

        tree = KDTree(Q)
        _, idx = tree.query(P_aligned)
        Q_matched = Q[idx]

        # -----------------------------
        # 🔥 UPDATE FIRST (FIXED)
        # -----------------------------
        if mode == "single":
            # ONLY translation (correct for 1 point)
            i = 0
            t = Q_matched[i] - P_aligned[i]
            R = np.eye(2)
        else:
            R, t = kabsch(P_aligned, Q_matched)

        P_aligned = (R @ P_aligned.T).T + t

        # -----------------------------
        # cost AFTER update
        # -----------------------------
        cost = compute_cost(P_aligned, Q_matched)

        # -----------------------------
        # plot AFTER update
        # -----------------------------
        plot_step(ax, P_orig, P_aligned, Q, Q_matched, it, cost, mode)

    plt.ioff()
    plt.show()


# -----------------------------
# Run
# -----------------------------
if __name__ == "__main__":
    print("MULTI ICP")
    icp_visual(P, Q, mode="multi")

    print("SINGLE ICP")
    icp_visual(P, Q, mode="single")