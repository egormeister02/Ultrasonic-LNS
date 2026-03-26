import numpy as np
import itertools
import pandas as pd
import matplotlib.pyplot as plt
from scipy.optimize import least_squares

np.random.seed(47)


# -------------------------------------------------------
# ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ
# -------------------------------------------------------

def add_noise(value, noise_width):
    """Равномерный шум [-w/2, +w/2]"""
    return value + (np.random.rand() - 0.5) * noise_width


def analytic_solution_three(P1, P2, P3, r1, r2, r3):
    """
    Решение по 3 точкам (триангуляция)
    Возвращает один вариант решения — тот, где z > 0
    """
    A = np.array([
        2 * (P2 - P1),
        2 * (P3 - P1)
    ])
    b = np.array([
        r1**2 - r2**2 + np.dot(P2, P2) - np.dot(P1, P1),
        r1**2 - r3**2 + np.dot(P3, P3) - np.dot(P1, P1),
    ])
    xy = np.linalg.lstsq(A[:, :2], b, rcond=None)[0]  # x,y

    x, y = xy
    z_sq = r1**2 - (x - P1[0])**2 - (y - P1[1])**2
    if z_sq < 0:
        return None
    return np.array([x, y, np.sqrt(z_sq)])



# -------------------------------------------------------
# НОВЫЙ ГАУСС–НЬЮТОН ЧЕРЕЗ SciPy LM
# -------------------------------------------------------

def residuals(x, sensors, distances):
    """Функция невязок: ||x - sensor_i|| - dist_i"""
    return np.linalg.norm(x - sensors, axis=1) - distances


def gauss_newton_library(sensors, distances, x0):
    """
    Levenberg–Marquardt (метод='lm') — устойчивый ГН
    """
    res = least_squares(
        residuals,
        x0,
        args=(sensors, distances),
        method='lm',
        max_nfev=50
    )
    return res.x
from scipy.optimize import least_squares
import numpy as np

def residuals_weighted(x, sensors, distances, weights):
    # вектор невязок, умноженный на sqrt(weights) для корректного взвешивания
    r = np.linalg.norm(x - sensors, axis=1) - distances
    return np.sqrt(weights) * r

def solve_with_scipy(sensors, distances, analytic_avg, sigma_r=0.1):
    """
    sensors: (n,3)
    distances: (n,)
    analytic_avg: усреднённый аналитический estimate (может быть None)
    sigma_r: оценка std шума по диапазону (м)
    """
    # 1) начальное приближение — аналитическое усреднение, если есть
    if analytic_avg is None:
        x0 = np.array([sensors[:,0].mean(), sensors[:,1].mean(), 10.0])
    else:
        x0 = analytic_avg.copy()

    # 2) веса: если шум равномерный, можно оценить эквивалентную дисперсию; тут используем 1/sigma^2
    w = np.ones(len(distances)) / (sigma_r**2)

    # 3) используем робастную loss и method='trf'
    res = least_squares(
        residuals_weighted, x0,
        args=(sensors, distances, w),
        method='trf',           # trf более гибкий, позволяет bounds
        loss='huber',           # уменьшает влияние выбросов
        f_scale=0.1,            # порог Хьюбера, подберите экспериментально
        max_nfev=200
    )
    return res.x, res.cost  # cost = 0.5 * sum(residuals**2)

# Пример использования внутри run_trial:
# analytic_avg = np.mean(valid_triple_solutions, axis=0)  # как раньше
# x_sol, cost = solve_with_scipy(sensors, noisy_dist, analytic_avg, sigma_r=0.1)


# -------------------------------------------------------
# АЛЬТЕРНАТИВНЫЕ МЕТОДЫ ОПТИМИЗАЦИИ
# -------------------------------------------------------

def residuals(x, sensors, distances):
    """Вектор невязок: ||x - s_i|| - d_i"""
    return np.linalg.norm(x - sensors, axis=1) - distances


# 1. Soft L1 Loss — менее агрессивно штрафует большие ошибки
def solve_soft_l1(sensors, distances, x0):
    res = least_squares(
        residuals, x0,
        args=(sensors, distances),
        method='trf',
        loss='soft_l1',  # отличие от LM
        f_scale=0.1,
        max_nfev=200
    )
    return res.x, res.cost


# 2. Linear Loss (L1) — минимизирует сумму абсолютных невязок
def solve_linear_loss(sensors, distances, x0):
    res = least_squares(
        residuals, x0,
        args=(sensors, distances),
        method='trf',
        loss='linear',  # L1 loss
        max_nfev=200
    )
    return res.x, res.cost


# 3. BFGS — квази-ньютоновский метод
from scipy.optimize import minimize

def objective_bfgs(x, sensors, distances):
    r = np.linalg.norm(x - sensors, axis=1) - distances
    return 0.5 * np.sum(r**2)

def solve_bfgs(sensors, distances, x0):
    res = minimize(
        objective_bfgs, x0,
        args=(sensors, distances),
        method='BFGS',
        options={'maxiter': 200}
    )
    return res.x, res.fun


# -------------------------------------------------------
# ОДНО ИЗМЕРЕНИЕ
# -------------------------------------------------------

def run_trial(sensors, H=20):
    # истинное положение с шумом ±2 м
    true_pos = np.array([
        add_noise(0, 4),
        add_noise(0, 4),
        add_noise(H, 4)
    ])

    # истинные расстояния
    true_dist = np.array([np.linalg.norm(true_pos - s) for s in sensors])

    # шум расстояний ±0.2 м
    noisy_dist = np.array([add_noise(d, 0.2) for d in true_dist])

    # ------- АНАЛИТИКА ПО ТРОЙКАМ -------
    tris = list(itertools.combinations(range(len(sensors)), 3))
    sols = []
    for i, j, k in tris:
        sol = analytic_solution_three(sensors[i], sensors[j], sensors[k],
                                      noisy_dist[i], noisy_dist[j], noisy_dist[k])
        if sol is not None and sol[2] > 0:
            sols.append(sol)

    analytic = None
    if sols:
        analytic = np.mean(sols, axis=0)

    # начальное приближение
    if sols:
        x0 = sols[0]
    else:
        x0 = np.array([0, 0, H])

    # ------- ВСЕ МЕТОДЫ ОПТИМИЗАЦИИ -------
    # 1. Текущий LM с Huber (как раньше)
    gn, _ = solve_with_scipy(sensors, noisy_dist, analytic, sigma_r=0.1)
    
    # 2. Soft L1
    soft_l1, _ = solve_soft_l1(sensors, noisy_dist, x0)
    
    # 3. Linear (L1)
    linear, _ = solve_linear_loss(sensors, noisy_dist, x0)
    
    # 4. BFGS
    bfgs, _ = solve_bfgs(sensors, noisy_dist, x0)

    return true_pos, analytic, gn, soft_l1, linear, bfgs


# -------------------------------------------------------
# СЕРИЯ ИЗМЕРЕНИЙ
# -------------------------------------------------------

def run_experiment(sensors, n=20):
    results = []
    for _ in range(n):
        true_pos, analytic, gn, soft_l1, linear, bfgs = run_trial(sensors)
        if analytic is None:
            continue

        err_an = np.linalg.norm(analytic - true_pos)
        err_gn = np.linalg.norm(gn - true_pos)
        err_soft = np.linalg.norm(soft_l1 - true_pos)
        err_lin = np.linalg.norm(linear - true_pos)
        err_bfgs = np.linalg.norm(bfgs - true_pos)

        results.append([err_an, err_gn, err_soft, err_lin, err_bfgs])
    return np.array(results)



# -------------------------------------------------------
# КОНФИГУРАЦИИ СЕНСОРОВ
# -------------------------------------------------------

sensors4 = np.array([
    [-1, -1, 0],
    [-1,  1, 0],
    [ 1, -1, 0],
    [ 1,  1, 0],
])

sensors5 = np.array([
    [-1, -1, 0],
    [-1,  1, 0],
    [ 1, -1, 0],
    [ 1,  1, 0],
    [ 0,  0, 0],   # центр
])



# -------------------------------------------------------
# ЗАПУСК ЭКСПЕРИМЕНТОВ
# -------------------------------------------------------

res4 = run_experiment(sensors4, n=50)
res5 = run_experiment(sensors5, n=50)

df4 = pd.DataFrame(res4, columns=["analytic", "lm_huber", "soft_l1", "linear", "bfgs"])
df5 = pd.DataFrame(res5, columns=["analytic", "lm_huber", "soft_l1", "linear", "bfgs"])

print("=" * 60)
print("4 датчика:")
print("=" * 60)
print(df4.describe().loc[['mean', 'std', 'min', 'max']])
print()

print("=" * 60)
print("5 датчиков:")
print("=" * 60)
print(df5.describe().loc[['mean', 'std', 'min', 'max']])