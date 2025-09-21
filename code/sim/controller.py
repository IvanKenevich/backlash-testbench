import numpy as np

def controller(theta_act, t):
    theta_ref = np.sin(t * 2 * np.pi) if (t < 10) or (t > 20) else 0 
    err = theta_ref - theta_act
    Kp = 10
    return err * Kp # Torque request