#!usr/bin/env python
__author__ = "Luis Tabarez"
__email__ = "luistabarez96@gmail.com"
__copyright__ = "Copyright 2024 Denver_UdG"
__date__ = "Dec 07, 2024"
__version__ = "0.1.0"
__status__ = "Production"

"""
Welcome!!! SMILEi Robot is HERE!!

States:
    1. Activate Robot
    2. Go to Home Position
    3. Go to Zero Position
    5. Deactivate Robot
    6. Out
"""

from pybear import Manager 
from robot_params import LeftArmParams, RightArmParams
import numpy as np
import time
import sys
import select
import math 
import socket
import struct
import threading

host_remote_server='192.168.0.100' #Remote Server IP 
port_remote_server=5005            #Remote Server PORT 

host_remote_client='192.168.0.100' #Remote Client IP
port_remote_client= 4000           #Remote Client PORT   
    
#remote_client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#remote_client_socket.bind((host_remote_client,port_remote_client))
#remote_client_socket.settimeout(0.001) #Evita bloque indefinido
    
#remote_server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#remote_server_socket.bind((host_remote_server,port_remote_server))
#remote_server_socket.settimeout(0.001) #Evita bloque indefinido

local_addr=('192.168.0.2',5005)  #Client IP and PORT

received_data=[]
data_lock=threading.Lock()

flag = False
bear_r = Manager.BEAR(port="/dev/ttyUSB0", baudrate=8000000)  # need to identify the port name on your PC
bear_l = Manager.BEAR(port="/dev/ttyUSB1", baudrate=8000000)  # need to identify the port name on your PC

m_id_1 = 1  # BEAR ID (default is 1) RA
m_id_2 = 2
m_id_3 = 3
m_id_4 = 4
m_id_5 = 1  # BEAR ID (default is 1) LA
m_id_6 = 2
m_id_7 = 3
m_id_8 = 4

p_gain = 5.0  # Set P gain as spring stiffness
d_gain = 0.2  # Set D gain as damper strength
i_gain = 0.0  # I gain is usually not needed
iq_max = 3.0  # Max iq
Kt=0.35   # N.m/A

def show_menu(options):
    print('Select one option:')
    for key in sorted(options):
        print(f' {key}) {options[key][0]}')

def read_option(options):
    while (a := input('Option: ')) not in options:
        print('Incorrect option, try again!')
    return a

def ejecute_option(option,options):
    options[option][1]()

def gen_state_machine(options,out):
    option=None
    while option!=out:
        show_menu(options)
        option=read_option(options)
        ejecute_option(option,options)
        print()

def state_machine():
    states={
        '1':(' Enable Robot',enable_robot),
        '2':(' Go to Home Position',home_position),
        '3':(' Go to Zero Position',zero_position),
        '4':(' Say Hello',say_hello),
        '5':(' Local Teleoperation',local_teleoperation),
        '6':(' Remote Teleoperation',remote_teleop),
        '7':(' Disable Robot',disable_robot),
        '8':(' Out', out)
    }
    gen_state_machine(states,'8')

def enable_robot():
    error = False
    global flag
    BEAR_connected_r = bear_r.ping(m_id_1,m_id_2,m_id_3,m_id_4)[0]
    BEAR_connected_l = bear_l.ping(m_id_5,m_id_6,m_id_7,m_id_8)[0]

    if not BEAR_connected_r or not BEAR_connected_l:
        # BEAR is offline
        print("BEAR is offline. Check power and communication.")
        error = True
        flag = False
        exit()
    if not error:
        # BEARs are online
        # Set PID, mode, and limit
        flag = True
        print("Welcome aboard, Captain!")
        # PID id/iq
        bear_r.set_p_gain_iq((m_id_1, 0.02),(m_id_2,0.02),(m_id_3,0.02),(m_id_4,0.02))
        bear_r.set_i_gain_iq((m_id_1, 0.02),(m_id_2,0.02),(m_id_3,0.02),(m_id_4,0.02))
        bear_r.set_d_gain_iq((m_id_1, 0),(m_id_2,0),(m_id_3,0),(m_id_4,0))
        bear_r.set_p_gain_id((m_id_1, 0.02),(m_id_2,0.02),(m_id_3,0.02),(m_id_4,0.02))
        bear_r.set_i_gain_id((m_id_1, 0.02),(m_id_2,0.02),(m_id_3,0.02),(m_id_4,0.02))
        bear_r.set_d_gain_id((m_id_1, 0),(m_id_2,0),(m_id_3,0),(m_id_4,0))
        
        # PID position mode
        bear_r.set_p_gain_position((m_id_1, p_gain),(m_id_2, p_gain),(m_id_3, p_gain),(m_id_4, p_gain))
        bear_r.set_i_gain_position((m_id_1, i_gain),(m_id_2, i_gain),(m_id_3, i_gain),(m_id_4, i_gain))
        bear_r.set_d_gain_position((m_id_1, d_gain),(m_id_2, d_gain),(m_id_3, d_gain),(m_id_4, d_gain))

        bear_l.set_p_gain_position((m_id_5, p_gain),(m_id_6, p_gain),(m_id_7, p_gain),(m_id_8, p_gain))
        bear_l.set_i_gain_position((m_id_5, i_gain),(m_id_6, i_gain),(m_id_7, i_gain),(m_id_8, i_gain))
        bear_l.set_d_gain_position((m_id_5, d_gain),(m_id_6, d_gain),(m_id_7, d_gain),(m_id_8, d_gain))

        # Put into position mode
        bear_r.set_mode((m_id_1, 2),(m_id_2,2),(m_id_3,2),(m_id_4,2))
        bear_l.set_mode((m_id_5, 2),(m_id_6,2),(m_id_7,2),(m_id_8,2))

        # Set iq limit
        bear_r.set_limit_iq_max((m_id_1, iq_max),(m_id_2, iq_max),(m_id_3, iq_max),(m_id_4, iq_max))
        bear_l.set_limit_iq_max((m_id_5, iq_max),(m_id_6, iq_max),(m_id_7, iq_max),(m_id_8, iq_max))

        # Enable BEAR
        bear_r.set_torque_enable((m_id_1, 1),(m_id_2, 1),(m_id_3, 1),(m_id_4, 1))
        bear_l.set_torque_enable((m_id_5, 1),(m_id_6, 1),(m_id_7, 1),(m_id_8, 1))

        print("Robot is Enable!!")
        time.sleep(1)

def home_position():
    if flag == False:
        print("The robot is disabled, please enable it")
        time.sleep(1)
    if flag == True:
        home_pos_r=np.array([0,1.5707,-1.5707,-0.785])
        home_pos_l=np.array([0,-1.5707,1.5707,-0.785])
        init_r=np.array([bear_r.get_present_position(m_id_1)[0][0][0],bear_r.get_present_position(m_id_2)[0][0][0],bear_r.get_present_position(m_id_3)[0][0][0],bear_r.get_present_position(m_id_4)[0][0][0]])
        init_l=np.array([bear_l.get_present_position(m_id_5)[0][0][0],bear_l.get_present_position(m_id_6)[0][0][0],bear_l.get_present_position(m_id_7)[0][0][0],bear_l.get_present_position(m_id_8)[0][0][0]])
        num=100
        delta_angle_r=(home_pos_r-init_r)/num
        delta_angle_l=(home_pos_l-init_l)/num

        for i in range(num):
            goal_pos_r=init_r + delta_angle_r * (i + 1)
            goal_pos_l=init_l + delta_angle_l * (i + 1)
            bear_r.set_goal_position((m_id_1, goal_pos_r[0]),(m_id_2, goal_pos_r[1]),(m_id_3, goal_pos_r[2]),(m_id_4, goal_pos_r[3]))
            bear_l.set_goal_position((m_id_5, goal_pos_l[0]),(m_id_6, goal_pos_l[1]),(m_id_7, goal_pos_l[2]),(m_id_8, goal_pos_l[3]))
            time.sleep(0.01)
        print("Home position")
        time.sleep(1)

def zero_position():
    if flag == False:
        print("The robot is disabled, please enable it")
        time.sleep(1)
    if flag == True:
        zero_pos=np.zeros(4)
        num=100
        init_r=np.array([bear_r.get_present_position(m_id_1)[0][0][0],bear_r.get_present_position(m_id_2)[0][0][0],bear_r.get_present_position(m_id_3)[0][0][0],bear_r.get_present_position(m_id_4)[0][0][0]])
        init_l=np.array([bear_l.get_present_position(m_id_5)[0][0][0],bear_l.get_present_position(m_id_6)[0][0][0],bear_l.get_present_position(m_id_7)[0][0][0],bear_l.get_present_position(m_id_8)[0][0][0]])
        delta_angle_r=(zero_pos-init_r)/num
        delta_angle_l=(zero_pos-init_l)/num
        for i in range(num):
            goal_pos_r=init_r + delta_angle_r * (i + 1)
            goal_pos_l=init_l + delta_angle_l * (i + 1)
            bear_r.set_goal_position((m_id_1, goal_pos_r[0]),(m_id_2, goal_pos_r[1]),(m_id_3, goal_pos_r[2]),(m_id_4, goal_pos_r[3]))
            bear_l.set_goal_position((m_id_5, goal_pos_l[0]),(m_id_6, goal_pos_l[1]),(m_id_7, goal_pos_l[2]),(m_id_8, goal_pos_l[3]))
            time.sleep(0.01)
        print("Zero position!!")
        time.sleep(1)

def say_hello():
    zero_position()
    while True:
        secons = time.time()
        qd=0.22*math.sin(12.0*secons)-1
        bear_r.set_goal_position((m_id_4, qd))
        bear_l.set_goal_position((m_id_8, qd))
        time.sleep(0.01)

        if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
            line = input()
            run = False
            print("Terminated by user.")
            # Disable BEAR
            break
    
def right_gravity_vector(q):
    q[3]=-1.0*q[3]
    g=RightArmParams.g
    m1=RightArmParams.m1
    m2=RightArmParams.m2
    m3=RightArmParams.m3
    m4=RightArmParams.m4
    a3=RightArmParams.a3
    d2=RightArmParams.d2
    rcx1=RightArmParams.rcx1
    rcx2=RightArmParams.rcx2
    rcy2=RightArmParams.rcy2
    rcz2=RightArmParams.rcz2
    rcy3=RightArmParams.rcy3
    rcz3=RightArmParams.rcz3
    rcx4=RightArmParams.rcx4
    rcy4=RightArmParams.rcy4
    rcz4=RightArmParams.rcz4
    G=np.zeros(4)
    sinq=np.zeros(4)
    cosq=np.zeros(4)

    for i in range(4):
        sinq[i]=math.sin(q[i])
        cosq[i]=math.cos(q[i])
        
    g1=g*m4*(rcz4*(cosq[0]*cosq[2] - cosq[1]*sinq[0]*sinq[2]) + rcx4*(sinq[3]*(cosq[0]*sinq[2] + cosq[1]*cosq[2]*sinq[0]) + cosq[3]*sinq[0]*sinq[1]) + rcy4*(cosq[3]*(cosq[0]*sinq[2] + cosq[1]*cosq[2]*sinq[0]) - sinq[0]*sinq[1]*sinq[3]) + a3*sinq[3]*(cosq[0]*sinq[2] + cosq[1]*cosq[2]*sinq[0]) + d2*sinq[0]*sinq[1] + a3*cosq[3]*sinq[0]*sinq[1]) - g*m2*(rcy2*cosq[0] + rcx2*cosq[1]*sinq[0] - rcz2*sinq[0]*sinq[1]) + g*m3*(rcz3*(cosq[0]*cosq[2] - cosq[1]*sinq[0]*sinq[2]) + d2*sinq[0]*sinq[1] + rcy3*sinq[0]*sinq[1]) - g*m1*rcx1*sinq[0]
    g2=-g*cosq[0]*(d2*m3*cosq[1] + d2*m4*cosq[1] + m3*rcy3*cosq[1] + m2*rcz2*cosq[1] + m2*rcx2*sinq[1] + m3*rcz3*sinq[1]*sinq[2] + m4*rcz4*sinq[1]*sinq[2] + a3*m4*cosq[1]*cosq[3] + m4*rcx4*cosq[1]*cosq[3] - m4*rcy4*cosq[1]*sinq[3] - a3*m4*cosq[2]*sinq[1]*sinq[3] - m4*rcy4*cosq[2]*cosq[3]*sinq[1] - m4*rcx4*cosq[2]*sinq[1]*sinq[3])
    g3=g*m4*(a3*sinq[3]*(cosq[2]*sinq[0] + cosq[0]*cosq[1]*sinq[2]) - rcz4*(sinq[0]*sinq[2] - cosq[0]*cosq[1]*cosq[2]) + rcy4*cosq[3]*(cosq[2]*sinq[0] + cosq[0]*cosq[1]*sinq[2]) + rcx4*sinq[3]*(cosq[2]*sinq[0] + cosq[0]*cosq[1]*sinq[2])) - g*m3*rcz3*(sinq[0]*sinq[2] - cosq[0]*cosq[1]*cosq[2])    
    g4=g*m4*(rcx4*(cosq[3]*(sinq[0]*sinq[2] - cosq[0]*cosq[1]*cosq[2]) + cosq[0]*sinq[1]*sinq[3]) - rcy4*(sinq[3]*(sinq[0]*sinq[2] - cosq[0]*cosq[1]*cosq[2]) - cosq[0]*cosq[3]*sinq[1]) + a3*cosq[3]*(sinq[0]*sinq[2] - cosq[0]*cosq[1]*cosq[2]) + a3*cosq[0]*sinq[1]*sinq[3])

    G[0]=g1
    G[1]=g2
    G[2]=g3*0
    G[3]=g4*0
    return G

def rightArmFK(q):
        q[3]=-1.0*q[3]
        a3=RightArmParams.a3
        d2=RightArmParams.d2
        K=np.zeros(3)
        sinq=np.zeros(4)
        cosq=np.zeros(4)

        for i in range(4):
            sinq[i]=math.sin(q[i])
            cosq[i]=math.cos(q[i])

        Px=sinq[3]*cosq[2]*sinq[0]*cosq[1]*a3+sinq[3]*sinq[2]*cosq[0]*a3+sinq[0]*sinq[1]*a3*cosq[3]+sinq[0]*sinq[1]*d2
        Py=sinq[1]*cosq[2]*a3*sinq[3]-cosq[1]*a3*cosq[3]-cosq[1]*d2
        Pz=-sinq[3]*cosq[2]*cosq[1]*cosq[0]*a3+sinq[3]*sinq[2]*sinq[0]*a3-cosq[0]*sinq[1]*a3*cosq[3]-cosq[0]*sinq[1]*d2
        K=([Px,Py,Pz])
        return K

def leftArmFK(q):
    q[3]=-1.0*q[3]
    a3=LeftArmParams.a3
    d2=LeftArmParams.d2
    K=np.zeros(3)
    sinq=np.zeros(4)
    cosq=np.zeros(4)

    for i in range(4):
        sinq[i]=math.sin(q[i])
        cosq[i]=math.cos(q[i])

    Px=sinq[3]*cosq[2]*sinq[0]*cosq[1]*a3 + sinq[3]*sinq[2]*cosq[0]*a3 + sinq[0]*sinq[1]*a3*cosq[3]+sinq[0]*sinq[1]*d2
    Py=-sinq[1]*cosq[2]*a3*sinq[3] + cosq[1]*a3*cosq[3] + cosq[1]*d2
    Pz=sinq[3]*cosq[2]*cosq[1]*cosq[0]*a3 - sinq[3]*sinq[2]*sinq[0]*a3 + cosq[0]*sinq[1]*a3*cosq[3]+cosq[0]*sinq[1]*d2
    K=([Px,Py,Pz])
    return K

def left_gravity_vector(q):
    q[3]=-1.0*q[3]
    g=LeftArmParams.g
    m1=LeftArmParams.m1
    m2=LeftArmParams.m2
    m3=LeftArmParams.m3
    m4=LeftArmParams.m4
    a3=LeftArmParams.a3
    d2=LeftArmParams.d2
    rcx1=LeftArmParams.rcx1
    rcx2=LeftArmParams.rcx2
    rcy2=LeftArmParams.rcy2
    rcz2=LeftArmParams.rcz2
    rcy3=LeftArmParams.rcy3
    rcz3=LeftArmParams.rcz3
    rcx4=LeftArmParams.rcx4
    rcy4=LeftArmParams.rcy4
    rcz4=LeftArmParams.rcz4

    G=np.zeros(4)
    sinq=np.zeros(4)
    cosq=np.zeros(4)

    for i in range(4):
        sinq[i]=math.sin(q[i])
        cosq[i]=math.cos(q[i])
        
    g1= -g*m2*(rcy2*cosq[0] + rcx2*cosq[1]*sinq[0] + rcz2*sinq[0]*sinq[1]) - g*m4*(rcz4*(cosq[0]*cosq[2] - cosq[1]*sinq[0]*sinq[2]) + rcx4*(sinq[3]*(cosq[0]*sinq[2] + cosq[1]*cosq[2]*sinq[0]) + cosq[3]*sinq[0]*sinq[1]) + rcy4*(cosq[3]*(cosq[0]*sinq[2] + cosq[1]*cosq[2]*sinq[0]) - sinq[0]*sinq[1]*sinq[3]) + a3*sinq[3]*(cosq[0]*sinq[2] + cosq[1]*cosq[2]*sinq[0]) + d2*sinq[0]*sinq[1] + a3*cosq[3]*sinq[0]*sinq[1]) - g*m3*(rcz3*(cosq[0]*cosq[2] - cosq[1]*sinq[0]*sinq[2]) + d2*sinq[0]*sinq[1] - rcy3*sinq[0]*sinq[1]) - g*m1*rcx1*sinq[0]
    g2=g*cosq[0]*(d2*m3*cosq[1] + d2*m4*cosq[1] - m3*rcy3*cosq[1] + m2*rcz2*cosq[1] - m2*rcx2*sinq[1] + m3*rcz3*sinq[1]*sinq[2] + m4*rcz4*sinq[1]*sinq[2] + a3*m4*cosq[1]*cosq[3] + m4*rcx4*cosq[1]*cosq[3] - m4*rcy4*cosq[1]*sinq[3] - a3*m4*cosq[2]*sinq[1]*sinq[3] - m4*rcy4*cosq[2]*cosq[3]*sinq[1] - m4*rcx4*cosq[2]*sinq[1]*sinq[3])
    g3=g*m3*rcz3*(sinq[0]*sinq[2] - cosq[0]*cosq[1]*cosq[2]) - g*m4*(a3*sinq[3]*(cosq[2]*sinq[0] + cosq[0]*cosq[1]*sinq[2]) - rcz4*(sinq[0]*sinq[2] - cosq[0]*cosq[1]*cosq[2]) + rcy4*cosq[3]*(cosq[2]*sinq[0] + cosq[0]*cosq[1]*sinq[2]) + rcx4*sinq[3]*(cosq[2]*sinq[0] + cosq[0]*cosq[1]*sinq[2]))
    g4= -g*m4*(rcx4*(cosq[3]*(sinq[0]*sinq[2] - cosq[0]*cosq[1]*cosq[2]) + cosq[0]*sinq[1]*sinq[3]) - rcy4*(sinq[3]*(sinq[0]*sinq[2] - cosq[0]*cosq[1]*cosq[2]) - cosq[0]*cosq[3]*sinq[1]) + a3*cosq[3]*(sinq[0]*sinq[2] - cosq[0]*cosq[1]*cosq[2]) + a3*cosq[0]*sinq[1]*sinq[3])
    G[0]=g1
    G[1]=g2
    G[2]=g3*0
    G[3]=g4*0
    return G

def send_msg(local_server_socket,remote_addr,ql):
    # Send
    try:
        struct_ql = struct.pack('8f',*ql)
        local_server_socket.sendto(struct_ql,remote_addr)
    except Exception as e:
        print("[SEND ERROR]",e)
        #break
        
def receive_msg(local_client_socket):
    #Receive
    home=np.array([0,1.5707,-1.5707,0.785,0,-1.5707,1.5707,-0.785])
    try:
        data, addr = local_client_socket.recvfrom(1024)
        struct_qr = struct.unpack('<8f',data)
        with data_lock:
            received_data.append(struct_qr)
    except socket.timeout:
        print("Receive a timeout...")
    except Exception as e:
        print("[SEND ERROR]", e)
        #break 
    #time.sleep(0.001)   
        
def local_teleoperation():
    zero_position()
    time.sleep(2)
    bear_r.set_p_gain_iq((m_id_1, 0.277),(m_id_2, 0.277),(m_id_3, 0.277),(m_id_4, 0.277))
    bear_r.set_p_gain_id((m_id_1, 0.277),(m_id_2, 0.277),(m_id_3, 0.277),(m_id_4, 0.277))
    bear_l.set_p_gain_iq((m_id_5,0.277),(m_id_6,0.277),(m_id_7, 0.277),(m_id_8, 0.277))
    bear_l.set_p_gain_id((m_id_5,0.277),(m_id_6,0.277),(m_id_7, 0.277),(m_id_8, 0.277))
    
    bear_r.set_i_gain_iq((m_id_1, 0.061),(m_id_2, 0.061),(m_id_3, 0.061),(m_id_4, 0.061))
    bear_r.set_i_gain_id((m_id_1, 0.061),(m_id_2, 0.061),(m_id_3, 0.061),(m_id_4, 0.061))
    bear_l.set_i_gain_iq((m_id_5, 0.061),(m_id_6, 0.061),(m_id_7, 0.061),(m_id_8, 0.061))
    bear_l.set_i_gain_id((m_id_5, 0.061),(m_id_6, 0.061),(m_id_7, 0.061),(m_id_8, 0.061))

    bear_r.set_d_gain_iq((m_id_1, 0),(m_id_2, 0),(m_id_3, 0),(m_id_4, 0))
    bear_r.set_d_gain_id((m_id_1, 0),(m_id_2, 0),(m_id_3, 0),(m_id_4, 0))
    bear_l.set_d_gain_iq((m_id_5, 0),(m_id_6, 0),(m_id_7, 0),(m_id_8, 0))
    bear_l.set_d_gain_id((m_id_5, 0),(m_id_6, 0),(m_id_7, 0),(m_id_8, 0))

    # # PID position mode
    bear_r.set_p_gain_position((m_id_1, 0),(m_id_2, 0),(m_id_3, 0),(m_id_4, 0))
    bear_r.set_i_gain_position((m_id_1, 0),(m_id_2, 0),(m_id_3, 0),(m_id_4, 0))
    bear_r.set_d_gain_position((m_id_1, 0),(m_id_2, 0),(m_id_3, 0),(m_id_4, 0))

    bear_l.set_p_gain_position((m_id_5, 0),(m_id_6, 0),(m_id_7, 0),(m_id_8, 0))
    bear_l.set_i_gain_position((m_id_5, 0),(m_id_6, 0),(m_id_7, 0),(m_id_8, 0))
    bear_l.set_d_gain_position((m_id_5, 0),(m_id_6, 0),(m_id_7, 0),(m_id_8, 0))

    # # Clear PID direct force
    bear_r.set_p_gain_force((m_id_1, 0),(m_id_2, 0),(m_id_3, 0),(m_id_4, 0))
    bear_r.set_i_gain_force((m_id_1, 0),(m_id_2, 0),(m_id_3, 0),(m_id_4, 0))
    bear_r.set_d_gain_force((m_id_1, 0),(m_id_2, 0),(m_id_3, 0),(m_id_4, 0))

    bear_l.set_p_gain_force((m_id_5, 0),(m_id_6, 0),(m_id_7, 0),(m_id_8, 0))
    bear_l.set_i_gain_force((m_id_5, 0),(m_id_6, 0),(m_id_7, 0),(m_id_8, 0))
    bear_l.set_d_gain_force((m_id_5, 0),(m_id_6, 0),(m_id_7, 0),(m_id_8, 0))

    # # Put into current mode
    bear_r.set_mode((m_id_1, 0),(m_id_2, 0),(m_id_3, 0),(m_id_4, 0))
    bear_r.set_torque_enable((m_id_1, 1),(m_id_2, 1),(m_id_3, 1),(m_id_4, 1))

    bear_l.set_mode((m_id_5, 0),(m_id_6, 0),(m_id_7, 0),(m_id_8, 0))
    bear_l.set_torque_enable((m_id_5, 1),(m_id_6, 1),(m_id_7, 1),(m_id_8, 1))

    kp=[2,2,1,1]
    kd=[0.01,0.01,0.01,0.01]
    
    while True:
        q_l=np.array([bear_l.get_present_position(m_id_5)[0][0][0],bear_l.get_present_position(m_id_6)[0][0][0],bear_l.get_present_position(m_id_7)[0][0][0],bear_l.get_present_position(m_id_8)[0][0][0]])
        dq_l=np.array([bear_l.get_present_velocity(m_id_5)[0][0][0],bear_l.get_present_velocity(m_id_6)[0][0][0],bear_l.get_present_velocity(m_id_7)[0][0][0],bear_l.get_present_velocity(m_id_8)[0][0][0]])
        
        q_r=np.array([bear_r.get_present_position(m_id_1)[0][0][0],bear_r.get_present_position(m_id_2)[0][0][0],bear_r.get_present_position(m_id_3)[0][0][0],bear_r.get_present_position(m_id_4)[0][0][0]])
        dq_r=np.array([bear_r.get_present_velocity(m_id_1)[0][0][0],bear_r.get_present_velocity(m_id_2)[0][0][0],bear_r.get_present_velocity(m_id_3)[0][0][0],bear_r.get_present_velocity(m_id_4)[0][0][0]])
        
        G_L=left_gravity_vector(q_l)
        G_R=right_gravity_vector(q_r)

        iq_1=0*(-kp[0]*(q_r[0]-(-1.0*q_l[0]))-kd*dq_r[0]+G_R[0])/Kt
        iq_2=0*(-kp[1]*(q_r[1]-(-1.0*q_l[1]))-kd*dq_r[1]+G_R[1])/Kt
        iq_3=0*(-kp[2]*(q_r[2]-(-1.0*q_l[2]))-kd*dq_r[2])/Kt
        iq_4=0*(-kp[3]*(bear_r.get_present_position(m_id_4)[0][0][0]-(-1.0*bear_l.get_present_position(m_id_8)[0][0][0])))/Kt

        iq_5=0*(-kp[0]*(q_l[0]-(-1.0*q_r[0]))-kd*dq_l[0]+G_L[0])/Kt
        iq_6=0*(-kp[1]*(q_l[1]-(-1.0*q_r[1]))-kd*dq_l[1]+G_L[1])/Kt
        iq_7=0*(-kp[2]*(q_l[2]-(-1.0*q_r[2]))-kd*dq_l[2])/Kt
        iq_8=0*(-kp[3]*(bear_l.get_present_position(m_id_8)[0][0][0]-(-1.0*bear_l.get_present_position(m_id_4)[0][0][0])))/Kt

        bear_r.set_goal_iq((m_id_1,iq_1),(m_id_2,iq_2),(m_id_3,iq_3),(m_id_4,iq_4))
        bear_l.set_goal_iq((m_id_5,iq_5),(m_id_6,iq_6),(m_id_7,iq_7),(m_id_8,iq_8))

        time.sleep(0.001)

        if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
            line = input()
            run = False
            print("Terminated by user.")
            # Disable BEAR
            # PID id/iq
            time.sleep(1)
            bear_r.set_p_gain_iq((m_id_1,0.02),(m_id_2,0.02),(m_id_3,0.02),(m_id_4,0.02))
            bear_r.set_p_gain_id((m_id_1,0.02),(m_id_2,0.02),(m_id_3,0.02),(m_id_4,0.02))
            bear_l.set_p_gain_iq((m_id_5,0.02),(m_id_6,0.02),(m_id_7,0.02),(m_id_8,0.02))
            bear_l.set_p_gain_id((m_id_5,0.02),(m_id_6,0.02),(m_id_7,0.02),(m_id_8,0.02))

            bear_r.set_i_gain_iq((m_id_1,0.02),(m_id_2,0.02),(m_id_3,0.02),(m_id_4,0.02))
            bear_r.set_i_gain_id((m_id_1,0.02),(m_id_2,0.02),(m_id_3,0.02),(m_id_4,0.02))
            bear_l.set_i_gain_iq((m_id_5,0.02),(m_id_6,0.02),(m_id_7,0.02),(m_id_8,0.02))
            bear_l.set_i_gain_id((m_id_5,0.02),(m_id_6,0.02),(m_id_7,0.02),(m_id_8,0.02))

            bear_r.set_d_gain_iq((m_id_1,0),(m_id_2,0),(m_id_3,0),(m_id_4,0))
            bear_r.set_d_gain_id((m_id_1,0),(m_id_2,0),(m_id_3,0),(m_id_4,0))
            bear_l.set_d_gain_iq((m_id_5,0),(m_id_6,0),(m_id_7,0),(m_id_8,0))
            bear_l.set_d_gain_id((m_id_5,0),(m_id_6,0),(m_id_7,0),(m_id_8,0))

            # PID position mode
            bear_r.set_p_gain_position((m_id_1, p_gain),(m_id_2, p_gain),(m_id_3, p_gain),(m_id_4, p_gain))
            bear_r.set_i_gain_position((m_id_1, i_gain),(m_id_2, i_gain),(m_id_3, i_gain),(m_id_4, i_gain))
            bear_r.set_d_gain_position((m_id_1, d_gain),(m_id_2, d_gain),(m_id_3, d_gain),(m_id_4, d_gain))
            bear_l.set_p_gain_position((m_id_5, p_gain),(m_id_6, p_gain),(m_id_7, p_gain),(m_id_8, p_gain))
            bear_l.set_i_gain_position((m_id_5, i_gain),(m_id_6, i_gain),(m_id_7, i_gain),(m_id_8, i_gain))
            bear_l.set_d_gain_position((m_id_5, d_gain),(m_id_6, d_gain),(m_id_7, d_gain),(m_id_8, d_gain))

            # Put into position mode
            bear_r.set_mode((m_id_1,2),(m_id_2,2),(m_id_3,2),(m_id_4,2))
            bear_l.set_mode((m_id_5,2),(m_id_6,2),(m_id_7,2),(m_id_8,2))

            time.sleep(2)
            home_position()
            break

def remote_teleop():
    zero_position()
    time.sleep(2)
     # PID id/iq
    bear_r.set_p_gain_iq((m_id_1, 0.277),(m_id_2, 0.277),(m_id_3, 0.277),(m_id_4, 0.277))
    bear_r.set_p_gain_id((m_id_1, 0.277),(m_id_2, 0.277),(m_id_3, 0.277),(m_id_4, 0.277))
    bear_l.set_p_gain_iq((m_id_5, 0.277),(m_id_6, 0.277),(m_id_7, 0.277),(m_id_8, 0.277))
    bear_l.set_p_gain_id((m_id_5, 0.277),(m_id_6, 0.277),(m_id_7, 0.277),(m_id_8, 0.277))

    bear_r.set_i_gain_iq((m_id_1, 0.061),(m_id_2, 0.061),(m_id_3, 0.061),(m_id_4, 0.061))
    bear_r.set_i_gain_id((m_id_1, 0.061),(m_id_2, 0.061),(m_id_3, 0.061),(m_id_4, 0.061))
    bear_l.set_i_gain_iq((m_id_5, 0.061),(m_id_6, 0.061),(m_id_7, 0.061),(m_id_8, 0.061))
    bear_l.set_i_gain_id((m_id_5, 0.061),(m_id_6, 0.061),(m_id_7, 0.061),(m_id_8, 0.061))

    bear_r.set_d_gain_iq((m_id_1, 0),(m_id_2, 0),(m_id_3, 0),(m_id_4, 0))
    bear_r.set_d_gain_id((m_id_1, 0),(m_id_2, 0),(m_id_3, 0),(m_id_4, 0))
    bear_l.set_d_gain_iq((m_id_5, 0),(m_id_6, 0),(m_id_7, 0),(m_id_8, 0))
    bear_l.set_d_gain_id((m_id_5, 0),(m_id_6, 0),(m_id_7, 0),(m_id_8, 0))

    # PID position mode
    bear_r.set_p_gain_position((m_id_1, 0),(m_id_2, 0),(m_id_3, 0),(m_id_4, 0))
    bear_r.set_i_gain_position((m_id_1, 0),(m_id_2, 0),(m_id_3, 0),(m_id_4, 0))
    bear_r.set_d_gain_position((m_id_1, 0),(m_id_2, 0),(m_id_3, 0),(m_id_4, 0))

    bear_l.set_p_gain_position((m_id_5, 0),(m_id_6, 0),(m_id_7, 0),(m_id_8, 0))
    bear_l.set_i_gain_position((m_id_5, 0),(m_id_6, 0),(m_id_7, 0),(m_id_8, 0))
    bear_l.set_d_gain_position((m_id_5, 0),(m_id_6, 0),(m_id_7, 0),(m_id_8, 0))

    # Clear PID direct force
    bear_r.set_p_gain_force((m_id_1, 0),(m_id_2, 0),(m_id_3, 0),(m_id_4, 0))
    bear_r.set_i_gain_force((m_id_1, 0),(m_id_2, 0),(m_id_3, 0),(m_id_4, 0))
    bear_r.set_d_gain_force((m_id_1, 0),(m_id_2, 0),(m_id_3, 0),(m_id_4, 0))

    bear_l.set_p_gain_force((m_id_5, 0),(m_id_6, 0),(m_id_7, 0),(m_id_8, 0))
    bear_l.set_i_gain_force((m_id_5, 0),(m_id_6, 0),(m_id_7, 0),(m_id_8, 0))
    bear_l.set_d_gain_force((m_id_5, 0),(m_id_6, 0),(m_id_7, 0),(m_id_8, 0))

    # Put into current mode
    bear_r.set_mode((m_id_1, 0),(m_id_2, 0),(m_id_3, 0),(m_id_4, 0))
    bear_r.set_torque_enable((m_id_1, 1),(m_id_2, 1),(m_id_3, 1),(m_id_4, 1))

    bear_l.set_mode((m_id_5, 0),(m_id_6, 0),(m_id_7, 0),(m_id_8, 0))
    bear_l.set_torque_enable((m_id_5, 1),(m_id_6, 1),(m_id_7, 1),(m_id_8, 1))

    kp=[1.75,1.75,1.75,1.75]
    kd=[0.1,0.1,0.1,0.1]
    
    qr1=0.0
    qr2=0.0
    qr3=0.0
    qr4=0.0

    ql1=0.0
    ql2=0.0
    ql3=0.0
    ql4=0.0

    q_r1=0.0
    q_r2=0.0
    q_r3=0.0
    q_r4=0.0

    q_l1=0.0
    q_l2=0.0
    q_l3=0.0
    q_l4=0.0

    qr1_a=0.0
    qr2_a=0.0
    qr3_a=0.0
    qr4_a=0.0

    ql1_a=0.0
    ql2_a=0.0
    ql3_a=0.0
    ql4_a=0.0

    while True:

        qr1=bear_r.get_present_position(m_id_1)[0][0][0]
        dqr1=bear_r.get_present_velocity(m_id_1)[0][0][0]

        qr2=bear_r.get_present_position(m_id_2)[0][0][0]
        dqr2=bear_r.get_present_velocity(m_id_2)[0][0][0]

        qr3=bear_r.get_present_position(m_id_3)[0][0][0]
        dqr3=bear_r.get_present_velocity(m_id_3)[0][0][0]

        qr4=bear_r.get_present_position(m_id_4)[0][0][0]
        dqr4=bear_r.get_present_velocity(m_id_4)[0][0][0]

        ql1=bear_l.get_present_position(m_id_5)[0][0][0]
        dql1=bear_l.get_present_velocity(m_id_5)[0][0][0]

        ql2=bear_l.get_present_position(m_id_6)[0][0][0]
        dql2=bear_l.get_present_velocity(m_id_6)[0][0][0]

        ql3=bear_l.get_present_position(m_id_7)[0][0][0]
        dql3=bear_l.get_present_velocity(m_id_7)[0][0][0]

        ql4=bear_l.get_present_position(m_id_8)[0][0][0]
        dql4=bear_l.get_present_velocity(m_id_8)[0][0][0]

        q_l=[qr1,qr2,qr3,qr4,ql1,ql2,ql3,ql4]
        

        send_thread=threading.Thread(target=send_msg,args=(remote_server_socket,local_addr,q_l,),daemon=True)
        receive_thread=threading.Thread(target=receive_msg,args=(remote_client_socket,),daemon=True)

        q_l[3]=-1.0*q_l[3]
        receive_thread.start()
        send_thread.start()
     
        try:
            #threading.Event().wait()
            with data_lock:
                for entry in received_data:
                    qr1_a=entry[0]
                    qr2_a=entry[1]
                    qr3_a=entry[2]
                    qr4_a=entry[3]
                    ql1_a=entry[4]
                    ql2_a=entry[5]
                    ql3_a=entry[6]
                    ql4_a=entry[7]
                if qr1_a>-1.58 and qr1_a<1.58:
                    q_r1=qr1_a
                if qr2_a>-0.79 and qr2_a<1.58:
                    q_r2=qr2_a
                if qr3_a>-3.1416 and qr3_a<1.58:
                    q_r3=qr3_a
                if qr4_a>-0.18 and qr4_a<1.16:
                    q_r4=qr4_a
                if ql1_a>-1.58 and ql1_a<1.58:
                    q_l1=ql1_a
                if ql2_a>-1.58 and ql2_a<0.79:
                    q_l2=ql2_a
                if ql3_a>-1.58 and ql3_a<3.1416:
                    q_l3=ql3_a
                if ql4_a>-1.16 and ql4_a<0.18:
                    q_l4=ql4_a
        except KeyboardInterrupt:
            print("Terminando comunicacion...")
            remote_server_socket.close()
            remote_client_socket.close()
            print("Terminated by user.")
            # Disable BEAR
            # PID id/iq
            time.sleep(1)
            bear_r.set_p_gain_iq((m_id_1, 0.02),(m_id_2,0.02),(m_id_3,0.02),(m_id_4,0.02))
            bear_r.set_p_gain_id((m_id_1, 0.02),(m_id_2,0.02),(m_id_3,0.02),(m_id_4,0.02))
            bear_l.set_p_gain_iq((m_id_5, 0.02),(m_id_6,0.02),(m_id_7,0.02),(m_id_8,0.02))
            bear_l.set_p_gain_id((m_id_5, 0.02),(m_id_6,0.02),(m_id_7,0.02),(m_id_8,0.02))

            bear_r.set_i_gain_iq((m_id_1, 0.02),(m_id_2,0.02),(m_id_3,0.02),(m_id_4,0.02))
            bear_r.set_i_gain_id((m_id_1, 0.02),(m_id_2,0.02),(m_id_3,0.02),(m_id_4,0.02))
            bear_l.set_i_gain_iq((m_id_5, 0.02),(m_id_6,0.02),(m_id_7,0.02),(m_id_8,0.02))
            bear_l.set_i_gain_id((m_id_5, 0.02),(m_id_6,0.02),(m_id_7,0.02),(m_id_8,0.02))

            bear_r.set_d_gain_iq((m_id_1, 0),(m_id_2,0),(m_id_3,0),(m_id_4,0))
            bear_r.set_d_gain_id((m_id_1, 0),(m_id_2,0),(m_id_3,0),(m_id_4,0))
            bear_l.set_d_gain_iq((m_id_5, 0),(m_id_6,0),(m_id_7,0),(m_id_8,0))
            bear_l.set_d_gain_id((m_id_5, 0),(m_id_6,0),(m_id_7,0),(m_id_8,0))
            
            # PID position mode
            bear_r.set_p_gain_position((m_id_1, p_gain),(m_id_2, p_gain),(m_id_3, p_gain),(m_id_4, p_gain))
            bear_r.set_i_gain_position((m_id_1, i_gain),(m_id_2, i_gain),(m_id_3, i_gain),(m_id_4, i_gain))
            bear_r.set_d_gain_position((m_id_1, d_gain),(m_id_2, d_gain),(m_id_3, d_gain),(m_id_4, d_gain))

            bear_l.set_p_gain_position((m_id_5, p_gain),(m_id_6, p_gain),(m_id_7, p_gain),(m_id_8, p_gain))
            bear_l.set_i_gain_position((m_id_5, i_gain),(m_id_6, i_gain),(m_id_7, i_gain),(m_id_8, i_gain))
            bear_l.set_d_gain_position((m_id_5, d_gain),(m_id_6, d_gain),(m_id_7, d_gain),(m_id_8, d_gain))

            # Put into position mode
            bear_r.set_mode((m_id_1, 2),(m_id_2, 2),(m_id_3, 2),(m_id_4, 2))
            bear_l.set_mode((m_id_5, 2),(m_id_6, 2),(m_id_7, 2),(m_id_8, 2))

            time.sleep(2)
            home_position()
            break
        
        send_thread.join()
        receive_thread.join()

        #print([q_r1,q_r2,q_r3,q_r4,q_l1,q_l2,q_l3,q_l4])
        #print([qr1-q_r1,qr2-q_r2,qr3-q_r3,qr4-q_r4])
        g_qr=q_l[:4]
        g_ql=q_l[4:]
        g_ql[3]=-1.0*g_ql[3]
        GR=right_gravity_vector(g_qr)
        GL=left_gravity_vector(g_ql)

        i_g_1=((-kp[0]*(qr1-q_r1)-kd[0]*dqr1+GR[0])/Kt)
        i_g_2=((-kp[1]*(qr2-q_r2)-kd[1]*dqr2+GR[1])/Kt)
        i_g_3=((-kp[2]*(qr3-q_r3)-kd[2]*dqr3)/Kt)
        i_g_4=((-kp[3]*(qr4-q_r4)-kd[3]*dqr4)/Kt)
        
        

        i_g_5=((-kp[0]*(ql1-q_l1)-kd[0]*dql1+GL[0])/Kt)
        i_g_6=((-kp[1]*(ql2-q_l2)-kd[1]*dql2+GL[1])/Kt)
        i_g_7=((-kp[2]*(ql3-q_l3)-kd[2]*dql3)/Kt)
        i_g_8=((-kp[3]*(ql4-q_l4)-kd[3]*dql4)/Kt)

        bear_r.set_goal_iq((m_id_1,i_g_1),(m_id_2,i_g_2),(m_id_3,i_g_3),(m_id_4,i_g_4))
        bear_l.set_goal_iq((m_id_5,i_g_5),(m_id_6,i_g_6),(m_id_7,i_g_7),(m_id_8,i_g_8))
        #time.sleep(0.001)   

def disable_robot():
    global flag
    if flag==True:
        home_position()
    bear_r.set_torque_enable((m_id_1, 0),(m_id_2, 0),(m_id_3, 0),(m_id_4, 0))
    bear_l.set_torque_enable((m_id_5, 0),(m_id_6, 0),(m_id_7, 0),(m_id_8, 0))
    flag=False
    print("Robot is Disabled!!")
    time.sleep(1)

def out():
    global flag
    if flag==True:
        home_position()
    bear_r.set_torque_enable((m_id_1, 0),(m_id_2, 0),(m_id_3, 0),(m_id_4, 0))
    bear_l.set_torque_enable((m_id_5, 0),(m_id_6, 0),(m_id_7, 0),(m_id_8, 0))
    flag=False
    print("Thanks for using SMILEi Robot!!")

if __name__ == '__main__':
    state_machine()


