def T_dist(T_req, holding_torque=5.):
    # set (Tm1, Tm2) such that total torque delivered to the output shaft is equal to T_req
    # with any holding torque compensated
    # when T_req > 0, Tm1 is driving, Tm2 is resisting and vice versa
    if T_req >= 0:
        Tm1 = T_req + holding_torque
        Tm2 = -holding_torque
    else:
        Tm1 = holding_torque
        Tm2 = T_req - holding_torque
    return Tm1, Tm2

def V_dist(V_req, holding_voltage=5.):
    # set (V1, V2) such that total voltage delivered to the output shaft is equal to V_req
    # with any holding voltage compensated
    # when V_req > 0, V1 is driving, V2 is resisting and vice versa
    if V_req >= 0:
        V1 = V_req + holding_voltage
        V2 = -holding_voltage
    else:
        V1 = holding_voltage
        V2 = V_req - holding_voltage
    return V1, V2