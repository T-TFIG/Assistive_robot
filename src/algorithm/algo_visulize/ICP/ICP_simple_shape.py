#!/usr/bin/env python3
import numpy as np
from scipy.spatial import KDTree

# the guessing point
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

# helper function 
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
    return np.mean(np.sum((P - Q)**2, axis=1))


# main ICP code 
def ICP_once(P, Q, R_init, t_init, max_iter=30):

    P_current = (R_init @ P.T).T + t_init

    R_total = R_init.copy()
    t_total = t_init.copy()

    prev_error = float('inf')

    for i in range(max_iter):

        q_star, dist = find_closest_points(P_current, Q)

        # keep best 70% matches
        idx = np.argsort(dist)
        keep = idx[:int(0.7 * len(idx))]

        P_used = P_current[keep]
        Q_used = q_star[keep]

        error = np.mean(dist)

        if abs(prev_error - error) < 1e-6:
            break
        prev_error = error

        # centroids
        p_c = centroid(P_used)
        q_c = centroid(Q_used)

        # center
        Pc = center(P_used, p_c)
        Qc = center(Q_used, q_c)

        # covariance
        H = covariance(Pc, Qc)

        # SVD
        U, S, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T

        if np.linalg.det(R) < 0:
            Vt[-1, :] *= -1
            R = Vt.T @ U.T

        t = q_c - R @ p_c

        # update
        P_current = (R @ P_current.T).T + t

        R_total = R @ R_total
        t_total = R @ t_total + t

    final_error = compute_error(P_current, q_star)
    return R_total, t_total, final_error


# multi point starter
def ICP_multi_start(P, Q, n_starts=20, tol=0.001):

    best_error = float('inf')
    best_R = None
    best_t = None

    for i in range(n_starts):

        # random rotation
        angle = np.random.uniform(0, 2*np.pi)
        R_init = np.array([
            [np.cos(angle), -np.sin(angle)],
            [np.sin(angle),  np.cos(angle)]
        ])

        # centroid alignment for translation
        t_init = centroid(Q) - centroid(P)

        R, t, err = ICP_once(P, Q, R_init, t_init)


        print(f"Start {i+1:02d} → error: {err:.6f}")

        if err < best_error:
            best_error = err
            best_R = R
            best_t = t

            if err <= tol:
                break


    return best_R, best_t, best_error

# -----------------------------
# Run
# -----------------------------
R_est, t_est, err = ICP_multi_start(P, Q, n_starts=30)

print("\n--- Ground Truth ---")
print("R:\n", R_true)
print("t:\n", t_true)

print("\n--- Estimated ---")
print("R:\n", R_est)
print("t:\n", t_est)
print("Final error:", err)