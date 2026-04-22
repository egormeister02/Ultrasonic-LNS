"""
Speed of sound in humid air.

Implements the ideal-gas model from the thesis (see ultrasonic-navigation.tex,
eq. speed-of-sound-humid-air), based on Cramer 1993 without the virial
correction:

    c(T, p, h) = sqrt( gamma * R * T / M_mix )

with water vapour mole fraction x_v = h * p_sat(T) / p (Tetens).
"""
import math


R = 8.314462618        # J / (mol K)
M_D = 0.0289652        # kg / mol, dry air
M_V = 0.01801528       # kg / mol, water
CP_D = 1005.0          # J / (kg K), dry air at const. pressure
CP_V = 1850.0          # J / (kg K), water vapour at const. pressure


def p_sat(T):
    """Saturation water-vapour pressure (Tetens), T in kelvin, returns Pa."""
    return 611.21 * math.exp(17.502 * (T - 273.15) / (T - 32.19))


def speed_of_sound(T, p, h):
    """
    Speed of sound in humid air [m/s].

    Parameters
    ----------
    T : float   absolute temperature [K]
    p : float   total pressure [Pa]
    h : float   relative humidity, 0..1

    Returns
    -------
    c : float   speed of sound [m/s]
    """
    x_v = h * p_sat(T) / p
    M_mix = (1.0 - x_v) * M_D + x_v * M_V
    R_mix = R / M_mix
    cp_mix = (1.0 - x_v) * CP_D + x_v * CP_V
    gamma = cp_mix / (cp_mix - R_mix)
    return math.sqrt(gamma * R_mix * T)


def _selfcheck():
    cases = [
        # (label,            T [K],   p [Pa],  h)
        ("dry, 0 C, 1 atm",  273.15,  101325,  0.0),
        ("dry, 20 C, 1 atm", 293.15,  101325,  0.0),
        ("50% RH, 20 C",     293.15,  101325,  0.5),
        ("100% RH, 20 C",    293.15,  101325,  1.0),
        ("dry, 20 C, 86 kPa",293.15,   86500,  0.0),
        ("50% RH, 20 C, 86 kPa", 293.15, 86500, 0.5),
    ]
    print(f"{'case':28s}  {'c, m/s':>8s}  {'x_v':>8s}")
    for label, T, p, h in cases:
        c = speed_of_sound(T, p, h)
        x_v = h * p_sat(T) / p
        print(f"{label:28s}  {c:8.3f}  {x_v:8.5f}")


if __name__ == "__main__":
    _selfcheck()
