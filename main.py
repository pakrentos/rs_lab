from can import CAN_Bus
from can.motor import CAN_Motor
import sys
import os
from time import time, sleep
import os
import numpy as np
from graphics import plot_data

bus = CAN_Bus(interface ='can0', bitrate=1000000)
motor = CAN_Motor(0x141, bus)
# motor.send(b'\x19' + 7*b'\x00')
start = time()
pos = []
vel = []
time_stamps = []
desired_pos = 180
desired_vel = 0
k_p = 0
k_d = 0
k_p = 10
k_d = 1
bound = 2000
filter_flag = False
filter_flag = True

qs = [180]
qvs = [0]
Kps = [10, 1]
Kds = [10, 1]
Kis = [10, 1, 0.1]
bounds = [200, 50]
filters = [1, 20]
reducers = [1, 5]

# print(tasks)
motor.send(b'\x9B' + 7*b'\x00')
print(motor.recive())
motor.send(b'\x88' + 7*b'\x00')
print(motor.recive())
params = {
    'P': 180,
    'V': 0,
    'Kp': 0.002,
    'Kd': 0.0002,
    'Ki': 0,
    'B': 160,
    'VF': 1,
    'R': 1,
    'grav_comp': True,
    'desired': True
}
try:
    # motor.PID_control(**params)
    motor.trap_func(20, 50, 180)
except KeyboardInterrupt:
    motor.reset_log()
    motor._send_n_rcv_torque(0)
    # plot_data(plot='tor', **params)
    # plot_data(plot='vel', **params)
    plot_data(plot='pos', **params)


# except KeyboardInterrupt:
#     print('Exiting...')
#     motor.smooth_off()
#     print('Motor off...')
#     sleep(1)
#     bus.can_down()
