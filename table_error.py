"""
Генерация таблиц ошибки для разных значений variance.
"""
import numpy as np
from simulation import run_experiment, SENSORS_4, SENSORS_5

VARIANCE_VALUES = [0.002, 0.005, 0.01, 0.05, 0.1]
N_TRIALS = 500
BIAS_STD = 0.0012


def generate_table():
    """Генерирует таблицы для обоих методов."""
    print("=" * 70)
    print("Analytic метод")
    print("=" * 70)
    print(f"{'Variance':<12} {'4 anchors (cm)':<18} {'5 anchors (cm)':<18}")
    print("-" * 70)
    
    analytic_4 = []
    analytic_5 = []
    
    for var in VARIANCE_VALUES:
        res4 = run_experiment(SENSORS_4, n=N_TRIALS, bias_std=BIAS_STD, variance_std=var)
        res5 = run_experiment(SENSORS_5, n=N_TRIALS, bias_std=BIAS_STD, variance_std=var)
        
        err_4 = res4[:, 0].mean() * 100
        err_5 = res5[:, 0].mean() * 100
        analytic_4.append(err_4)
        analytic_5.append(err_5)
        
        print(f"{var:<12} {err_4:<18.2f} {err_5:<18.2f}")
    
    print("\n" + "=" * 70)
    print("Gauss-Newton метод")
    print("=" * 70)
    print(f"{'Variance':<12} {'4 anchors (cm)':<18} {'5 anchors (cm)':<18}")
    print("-" * 70)
    
    gn_4 = []
    gn_5 = []
    
    for var in VARIANCE_VALUES:
        res4 = run_experiment(SENSORS_4, n=N_TRIALS, bias_std=BIAS_STD, variance_std=var)
        res5 = run_experiment(SENSORS_5, n=N_TRIALS, bias_std=BIAS_STD, variance_std=var)
        
        err_4 = res4[:, 1].mean() * 100
        err_5 = res5[:, 1].mean() * 100
        gn_4.append(err_4)
        gn_5.append(err_5)
        
        print(f"{var:<12} {err_4:<18.2f} {err_5:<18.2f}")
    
    return (analytic_4, analytic_5), (gn_4, gn_5)


if __name__ == '__main__':
    generate_table()