import numpy as np
import matplotlib.pyplot as plt

# Ошибка позиционирования по выведенной 3D-модели (раздел «Точность определения координат»):
# четыре якоря в вершинах квадрата со стороной a в плоскости z=0, объект на высоте H над центром.
#   sigma_xyz = sigma_d * D * sqrt(2/a^2 + 1/(4 H^2)),   D = sqrt(a^2/2 + H^2)
# (та же формула, что в тексте; раньше график считался по другой, 2D, формуле — устранено).

sigma_d = 0.003   # м, СКО измерения дальности (см. бюджет случайной ошибки, sigma_d ≈ 3 мм)
H = 25            # м, высота объекта над платформой


def sigma_xyz(a, H, sigma_d):
    D = np.sqrt(a**2 / 2 + H**2)
    return sigma_d * D * np.sqrt(2 / a**2 + 1 / (4 * H**2))


a_vals = np.linspace(0.5, 5, 200)
errors = sigma_xyz(a_vals, H, sigma_d) * 100  # в сантиметрах

plt.figure(figsize=(10, 6))
plt.plot(a_vals, errors, 'b-', linewidth=2)
plt.xlabel('Расстояние между якорями $a$, м', fontsize=12)
plt.ylabel('СКО положения $\\sigma_{xyz}$, см', fontsize=12)
plt.title('Зависимость ошибки позиционирования от размера базы якорей\n'
          f'($\\sigma_d = {sigma_d*1000:.0f}$ мм, $H = {H}$ м)', fontsize=14)
plt.grid(True, alpha=0.3)
plt.xlim(0.5, 5)
plt.ylim(0, max(errors) * 1.1)

# Рабочая конфигурация
a0 = 2.0
e0 = sigma_xyz(a0, H, sigma_d) * 100
plt.axvline(x=a0, color='r', linestyle='--',
            label=f'Рабочая конфигурация: $a = {a0:.0f}$ м, $\\sigma_{{xyz}} \\approx {e0:.0f}$ см')
plt.plot([a0], [e0], 'ro')
plt.legend(fontsize=11)

plt.tight_layout()
plt.savefig('./images/gdop-vs-anchor-distance.png', dpi=150)
print(f"sigma_xyz(a=2 м, H=25 м, sigma_d=3 мм) = {e0:.2f} см")
print("График сохранён")
