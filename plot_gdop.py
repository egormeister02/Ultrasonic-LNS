import numpy as np
import matplotlib.pyplot as plt

# Симуляция: три якоря в форме равностороннего треугольника
# меняем размер треугольника - расстояние между якорями

sigma_d = 0.17  # м (ошибка измерения расстояния, соответствующая 1 мкс дискретизации)

def calculate_gdop(side_length, distance_to_anchors):
    """
    Расчёт GDOP для равностороннего треугольника якорей
    side_length - сторона треугольника
    distance_to_anchors - расстояние от объекта до якорей
    """
    # Площадь треугольника
    S = (side_length**2) * np.sqrt(3) / 4
    
    # Формула: sigma_xy = sigma_d * D / sqrt(2*S)
    sigma_xy = sigma_d * distance_to_anchors / np.sqrt(2 * S)
    
    return sigma_xy

# Диапазон расстояний между якорями (0.1 до 5 метров)
side_lengths = np.linspace(0.1, 5, 100)

# Фиксированное расстояние до якорей
D = 25  # метров (как в примере из документа)

errors = [calculate_gdop(s, D) for s in side_lengths]

plt.figure(figsize=(10, 6))
plt.plot(side_lengths, errors, 'b-', linewidth=2)
plt.xlabel('Расстояние между якорями, м', fontsize=12)
plt.ylabel('Ошибка координаты sigma_xy, м', fontsize=12)
plt.title(f'Зависимость ошибки позиционирования от взаимного расположения якорей (D = {D} м, sigma_d = {sigma_d} м)', fontsize=14)
plt.grid(True, alpha=0.3)
plt.xlim(0.1, 5)
plt.ylim(0, max(errors)*1.1)

# Вертикальная линия для текущего случая (диагональ 2м = сторона ~1.4м)
plt.axvline(x=1.4, color='r', linestyle='--', label='Текущая конфигурация (сторона ~1.4м)')
plt.legend()

plt.tight_layout()
plt.savefig('/home/clawd/Ultrasonic-LNS/images/gdop-vs-anchor-distance.png', dpi=150)
print("График сохранён")