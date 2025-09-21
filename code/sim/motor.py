from dataclasses import dataclass
import numpy as np

@dataclass
class motor_pars:
    I: float # kg m^2 (moment of inertia)
    b: float # N * m / (rad/s) (damping coefficient)
    L: float # H (inductance)
    R: float # Ω (resistance)
    Kt: float # Nm/A (torque constant)
    Ke: float # V/(rad/s) (back EMF constant)

schiffer_motor = motor_pars(I=0.4, b=0.1, L=0.01, R=1, Kt=0.05, Ke=0.05)

def odefun_dc_motor(t, y, V, pars: motor_pars):
    """
    Simulates a DC motor with voltage input.
    """
    th, w, i = y
    T = pars.Kt * i

    dw_dt = (T - pars.b * w) / pars.I
    di_dt = (V - pars.R * i - pars.Ke * w) / pars.L
    dth_dt = w

    return [dth_dt, dw_dt, di_dt]