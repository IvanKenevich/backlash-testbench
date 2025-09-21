import numpy as np
import pandas as pd
from dataclasses import dataclass
from typing import Callable as function

from torque_splitter import T_dist
from motor import motor_pars, schiffer_motor

def wb_calc(thb, thd, wd, ks, cs, alpha):
    if thb <= -alpha:
        wb = max(0, wd + (ks/cs) * (thd - thb))
    elif np.abs(thb) < alpha:
        wb = wd + (ks/cs) * (thd - thb)
    elif thb >= alpha:
        wb = min(0, wd + (ks/cs) * (thd - thb))
    return wb

def backlash_calc(thb, wb, alpha):
    if (thb == -alpha and wb > 0) or (np.abs(thb) < alpha) or (thb == alpha and wb < 0):
        backlash = True
    else:
        backlash = False
    return backlash

def odefun_3mass_single_torque(t, y, Jm1, cm1, Jm2, cm2, Jl, cl, ks, cs, alpha1, alpha2, T_desired, Td_func):
    Tm1, Tm2 = T_dist(T_desired)
    return odefun_3mass_split_torque(t, y, Jm1, cm1, Jm2, cm2, Jl, cl, ks, cs, alpha1, alpha2, Tm1, Tm2, Td_func)

def odefun_3mass_split_torque(t, y, Jm1, cm1, Jm2, cm2, Jl, cl, ks, cs, alpha1, alpha2, Tm1, Tm2, Td_func):
    """
    Simulates 3-mass system with backlash with motor output torque as inputs.
    No electrical components simulated
    """
    wm1, wm2, wl, thm1, thm2, thl, thd1, thd2, thb1, thb2 = y
    Td = Td_func(t)

    wd1 = wm1 - wl
    wd2 = wm2 - wl

    wb1 = wb_calc(thb1, thd1, wd1, ks, cs, alpha1)
    wb2 = wb_calc(thb2, thd2, wd2, ks, cs, alpha2)

    backlash1 = backlash_calc(thb1, wb1, alpha1)
    backlash2 = backlash_calc(thb2, wb2, alpha2)

    Ts1 = 0 if backlash1 else ks * (thd1 - thb1) + cs * wd1
    Ts2 = 0 if backlash2 else ks * (thd2 - thb2) + cs * wd2

    dwm1_dt = (-cm1 * wm1 - Ts1 + Tm1) / Jm1
    dwm2_dt = (-cm2 * wm2 - Ts2 + Tm2) / Jm2
    dwl_dt = (-cl * wl + Ts1 + Ts2 - Td) / Jl
    thm1_dt = wm1
    thm2_dt = wm2
    thl_dt = wl
    thd1_dt = wd1
    thd2_dt = wd2
    thb1_dt = wb1
    thb2_dt = wb2

    return [dwm1_dt, dwm2_dt, dwl_dt, thm1_dt, thm2_dt, thl_dt, thd1_dt, thd2_dt, thb1_dt, thb2_dt]

@dataclass
class odefun_3mass_full_pars:
    m1: motor_pars
    m2: motor_pars
    Jl: float
    cl: float
    ks: float
    cs: float
    alpha1: float
    alpha2: float

@dataclass
class odefun_3mass_full_inputs:
    V1_func: function
    V2_func: function
    Td_func: function

class dummy:
    def __init__(self):
        self.calculations = pd.DataFrame(columns=["time", "Ts1", "Ts2", "Tm1", "Tm2"])

    def odefun_3mass_full(self, t, y, pars: odefun_3mass_full_pars, inputs: odefun_3mass_full_inputs):
        """
        Simulates 3-mass system with backlash with motor voltages as inputs.
        Includes electrical model of the system
        """
        wm1, wm2, wl, thm1, thm2, thl, thd1, thd2, thb1, thb2, i1, i2 = y
        Td = inputs.Td_func(t)

        Tm1 = pars.m1.Kt * i1
        Tm2 = pars.m2.Kt * i2

        wd1 = wm1 - wl
        wd2 = wm2 - wl

        wb1 = wb_calc(thb1, thd1, wd1, pars.ks, pars.cs, pars.alpha1)
        wb2 = wb_calc(thb2, thd2, wd2, pars.ks, pars.cs, pars.alpha2)

        backlash1 = backlash_calc(thb1, wb1, pars.alpha1)
        backlash2 = backlash_calc(thb2, wb2, pars.alpha2)

        Ts1 = 0 if backlash1 else pars.ks * (thd1 - thb1) + pars.cs * wd1
        Ts2 = 0 if backlash2 else pars.ks * (thd2 - thb2) + pars.cs * wd2

        dwm1_dt = (-pars.m1.b * wm1 - Ts1 + Tm1) / pars.m1.I
        dwm2_dt = (-pars.m2.b * wm2 - Ts2 + Tm2) / pars.m2.I
        dwl_dt = (-pars.cl * wl + Ts1 + Ts2 - Td) / pars.Jl
        thm1_dt = wm1
        thm2_dt = wm2
        thl_dt = wl
        thd1_dt = wd1
        thd2_dt = wd2
        thb1_dt = wb1
        thb2_dt = wb2
        di1_dt = (inputs.V1_func(t) - pars.m1.R * i1 - pars.m1.Ke * wm1) / pars.m1.L
        di2_dt = (inputs.V2_func(t) - pars.m2.R * i2 - pars.m2.Ke * wm2) / pars.m2.L

        self.calculations = pd.concat([self.calculations, pd.DataFrame(
            {'time': [t], 'Ts1': [Ts1], 'Ts2': [Ts2], 'Tm1': [Tm1], 'Tm2': [Tm2]})], ignore_index=True)
        return [dwm1_dt, dwm2_dt, dwl_dt, thm1_dt, thm2_dt, thl_dt, thd1_dt, thd2_dt, thb1_dt, thb2_dt, di1_dt, di2_dt]