"""
Построение графиков ошибки от variance для разного количества якорей.
"""
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from simulation import run_experiment, SENSORS_4, SENSORS_5


def sweep_variance(sensors, variance_values, n_measurements=50, bias_std=0.0012):
    results = {'analytic': [], 'gauss_newton': []}
    for var in variance_values:
        data = run_experiment(sensors, n=n_measurements, bias_std=bias_std, variance_std=var)
        results['analytic'].append(data[:, 0].mean())
        results['gauss_newton'].append(data[:, 1].mean())
    return results


def plot_error_vs_variance():
    variance_values = np.linspace(0.001, 0.2, 20)  # 1мм - 20см
    
    print("Расчёт для 4 якорей...")
    results_4 = sweep_variance(SENSORS_4, variance_values)
    
    print("Расчёт для 5 якорей...")
    results_5 = sweep_variance(SENSORS_5, variance_values)
    
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    x = variance_values * 1000  # в мм
    
    # 4 якоря
    ax1 = axes[0]
    y_an = np.array(results_4['analytic']) * 100
    y_gn = np.array(results_4['gauss_newton']) * 100
    ax1.plot(x, y_an, 'o-', label='Analytic', linewidth=2, markersize=5)
    ax1.plot(x, y_gn, 's-', label='Gauss-Newton', linewidth=2, markersize=5)
    ax1.set_xlabel('Variance, мм', fontsize=12)
    ax1.set_ylabel('Ошибка, см', fontsize=12)
    ax1.set_title('4 якоря', fontsize=14)
    ax1.legend(fontsize=10)
    ax1.grid(True, alpha=0.3)
    
    # 5 якорей
    ax2 = axes[1]
    y2_an = np.array(results_5['analytic']) * 100
    y2_gn = np.array(results_5['gauss_newton']) * 100
    ax2.plot(x, y2_an, 'o-', label='Analytic', linewidth=2, markersize=5)
    ax2.plot(x, y2_gn, 's-', label='Gauss-Newton', linewidth=2, markersize=5)
    ax2.set_xlabel('Variance, мм', fontsize=12)
    ax2.set_ylabel('Ошибка, см', fontsize=12)
    ax2.set_title('5 якорей', fontsize=14)
    ax2.legend(fontsize=10)
    ax2.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('error_vs_variance.png', dpi=150)
    print("Сохранено в error_vs_variance.png")
    plt.show()


if __name__ == '__main__':
    plot_error_vs_variance()