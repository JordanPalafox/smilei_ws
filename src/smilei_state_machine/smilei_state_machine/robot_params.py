#!/usr/bin/env python3

"""
Parámetros específicos del robot SMILEi.
Incluye constantes para límites de articulaciones y parámetros físicos.
"""

class Joint_Limits:
    q_l1=[-90,90]  # Límites en grados
    q_l2=[-90,45]
    q_l3=[-90,135]
    q_l4=[-90,90]
    q_r1=[-90,90]
    q_r2=[-45,90]
    q_r3=[-135,90]
    q_r4=[-90,90]

class Position_Control_Mode_Table_Gains:
    """
    Declaraciones de ganancias para el modo de control de posición
    """
    p_gain_position = 5.0
    d_gain_position = 0.2
    i_gain_position = 0.0
    iq_max = 1.5
    p_gain_iq = 0.02
    i_gain_iq = 0.02
    d_gain_iq = 0
    p_gain_id = 0.02
    i_gain_id = 0.02
    d_gain_id = 0
    kt = 0.35

class Current_Control_Mode_Table_Gains:
    """
    Declaraciones de ganancias para el modo de control de corriente
    """
    p_gain_position = 0.0
    d_gain_position = 0.0
    i_gain_position = 0.0
    p_gain_force = 0.0
    d_gain_force = 0.0
    i_gain_force = 0.0
    iq_max = 3.0
    p_gain_iq = 0.277
    i_gain_iq = 0.061
    d_gain_iq = 0.0
    p_gain_id = 0.277
    i_gain_id = 0.061
    d_gain_id = 0
    kt = 0.35

class RightArmParams:
    """
    Parámetros físicos del brazo derecho
    """
    # Masas
    m1=0.361
    m2=0.400
    m3=0.452
    m4=0.181
    # Longitudes de eslabones
    d2=0.113
    a3=0.123
    # Parámetros de inercia
    rcx1= -0.0371
    rcx2= -0.0154
    rcy2= 0.0812
    rcz2= -0.2084
    rcy3= 0.1585
    rcz3= 0.1687
    rcx4= -0.1282
    rcy4= -0.0039
    rcz4= -0.4192
    # Gravedad
    g=9.81
    
class LeftArmParams:
    """
    Parámetros físicos del brazo izquierdo
    """
    # Masas
    m1=0.361
    m2=0.400
    m3=0.452
    m4=0.181
    # Longitudes de eslabones
    d2=0.113
    a3=0.123
    # Parámetros de inercia
    rcx1= -0.0264
    rcx2= -0.0235
    rcy2= -0.0609
    rcz2= -0.0386
    rcy3= 0.0150
    rcz3= 0.0022
    rcx4= -0.0228
    rcy4= 0.0568
    rcz4= -0.0279
    # Gravedad
    g=9.81 