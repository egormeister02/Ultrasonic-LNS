"""
Simulation core functions for ultrasonic localization.
"""
import numpy as np
import itertools
from scipy.optimize import least_squares


def add_noise(value, noise_width):
    return value + (np.random.rand() - 0.5) * noise_width


def analytic_solution_three(P1, P2, P3, r1, r2, r3):
    A = np.array([
        2 * (P2 - P1)[:2],
        2 * (P3 - P1)[:2]
    ])
    b = np.array([
        r1**2 - r2**2 + np.dot(P2[:2], P2[:2]) - np.dot(P1[:2], P1[:2]),
        r1**2 - r3**2 + np.dot(P3[:2], P3[:2]) - np.dot(P1[:2], P1[:2]),
    ])
    xy = np.linalg.lstsq(A, b, rcond=None)[0]
    x, y = xy
    z_sq = r1**2 - (x - P1[0])**2 - (y - P1[1])**2
    if z_sq < 0:
        return None
    return np.array([x, y, np.sqrt(z_sq)])


def residuals(x, sensors, distances):
    return np.linalg.norm(x - sensors, axis=1) - distances


def gauss_newton_manual(sensors, distances, x0, max_iter=50, tol=1e-6):
    x = x0.copy()
    for _ in range(max_iter):
        r = np.linalg.norm(x - sensors, axis=1) - distances
        diff = x - sensors
        norms = np.linalg.norm(diff, axis=1, keepdims=True)
        J = diff / norms
        try:
            delta = np.linalg.lstsq(J, r, rcond=None)[0]
        except:
            break
        x = x - delta
        if np.linalg.norm(delta) < tol:
            break
    return x


SENSORS_4 = np.array([
    [-1, -1, 0], [-1, 1, 0], [1, -1, 0], [1, 1, 0],
])

SENSORS_5 = np.array([
    [-1, -1, 0], [-1, 1, 0], [1, -1, 0], [1, 1, 0], [0, 0, 0],
])


def run_trial(sensors, H=25, bias_std=0.05, variance_std=0.1):
    true_pos = np.array([
        add_noise(0, 4), add_noise(0, 4), add_noise(H, 4)
    ])
    true_dist = np.array([np.linalg.norm(true_pos - s) for s in sensors])
    
    bias = np.random.normal(0, bias_std)
    variance = np.random.normal(0, variance_std, size=len(sensors))
    noisy_dist = true_dist * (1 + bias) + variance

    tris = list(itertools.combinations(range(len(sensors)), 3))
    sols = []
    for i, j, k in tris:
        sol = analytic_solution_three(sensors[i], sensors[j], sensors[k],
                                      noisy_dist[i], noisy_dist[j], noisy_dist[k])
        if sol is not None and sol[2] > 0:
            sols.append(sol)

    analytic = np.mean(sols, axis=0) if sols else None
    x0 = sols[0] if sols else np.array([0, 0, H])

    gn = gauss_newton_manual(sensors, noisy_dist, x0)

    return true_pos, analytic, gn


def run_experiment(sensors, n=50, bias_std=0.05, variance_std=0.1):
    results = []
    for _ in range(n):
        true_pos, analytic, gn = run_trial(
            sensors, H=20, bias_std=bias_std, variance_std=variance_std
        )
        if analytic is None:
            continue
        err_an = np.linalg.norm(analytic - true_pos)
        err_gn = np.linalg.norm(gn - true_pos)
        results.append([err_an, err_gn])
    return np.array(results)