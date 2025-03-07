def ziegler_nichols_tuning(Ku, Pu, controller_type='pid'):
    """
    Ziegler-Nichols classical tuning rules.
    :param Ku: Ultimate gain
    :param Pu: Ultimate period
    :param controller_type: 'p', 'pi', or 'pid'
    :return: (Kp, Ki, Kd)
    """
    if controller_type.lower() == 'p':
        Kp = 0.5 * Ku
        Ki = 0.0
        Kd = 0.0
    elif controller_type.lower() == 'pi':
        Kp = 0.45 * Ku
        Ki = (1.2 * Kp) / Pu
        Kd = 0.0
    else:
        # 'pid'
        Kp = 0.6 * Ku
        Ki = (2.0 * Kp) / Pu
        Kd = (Kp * Pu) / 8.0
    
    return (Kp, Ki, Kd)
