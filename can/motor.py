import numpy as np 
from can import CAN_Bus
from time import time
from typing import NoReturn, Optional, Union, List, Tuple
from time import sleep
# from scipy import integrate
import multiprocessing as mp
from datetime import datetime as dt
import sys
import csv
import os
from toolz import curry

class FeedbackLinearization:

    def __init__(self, l, m, J, b, g, f):
        self.params = l, m, J, b, g
        self.control_func = f

    def control_law(self, q, qd):
        Qd = self.Q_d(q, qd)
        D = (1/self.D(q))*self.control_func(q, qd)
        h = self.h(q, qd)
        res = Qd + h + D
        return res

    def D(self, q):
        l, m, J, b, g = self.params

        d11 = m * l ** 2 + J
        # d12 = m[1] * l[0] * l[1] * np.cos(alpha_1 - alpha_2)
        # d21 = d12
        # d22 = m[1] * l[1] ** 2 + J[1]
        return d11 #np.array([[d11, d12], [d21, d22]])

    def c_term(self, q, dq):
        # alpha_1 = q
        # alpha_2 = 0
        # dalpha_1 = dq
        # dalpha_2 = 0
        #
        # l, m, J, b, g = self.params
        #
        # c1 = m[1] * l[0] * l[1] * np.sin(alpha_1 - alpha_2) * dalpha_2 ** 2
        # c2 = -m[1] * l[0] * l[1] * np.sin(alpha_1 - alpha_2) * dalpha_1 ** 2
        return 0#np.array([c1, c2])

    def g_term(self, q):
        alpha_1 = q

        l, m, J, b, g = self.params

        g1 = m * g * l * np.cos(alpha_1)
        return g1

    def Q_d(self, q, dq):
        dalpha_1 = dq
        l, m, J, b, g = self.params
        Q_d_1 = b * dalpha_1
        # Q_d_2 = b[1] * dalpha_2
        return Q_d_1

    def h(self, q, dq):
        return self.c_term(q, dq) + self.g_term(q)

class CAN_Device:
    dev_id: int
    bus: CAN_Bus 

    def __init__(self, dev_id: int, bus: CAN_Bus):
        self.dev_id = dev_id
        self.bus = bus
    
    def send(self, data: bytes) -> None:
        if not isinstance(data, bytes):
            raise TypeError('Data type must be bytes')
        self.bus.send_bytes(self.dev_id, data)
    
    def recive(self) -> bytes:
        can_id, _, data = self.bus.recive_frame()
        if can_id == self.dev_id:
            return data

class CAN_MotorStatus:
    encoder_bytes: bytes
    speed_bytes: bytes
    torque_bytes: bytes
    tmp_bytes: bytes

    encoder_data: int = None
    speed_data: int = None
    torque_data: int = None
    tmp_data: int = None
    time_data: float = None
    time_start: float
    frmt_str_len = 0

    log: List[Tuple[int, int, int, int, int]] = []

    turns: int = 0
    FULL_TURN: int = 2**14

    def __init__(self, log: bool = False, thresholds=(2**12, (2**12)*3), verbose=False):
        self.LOG_SET = log
        self.thrshld = thresholds
        self.time_start = time()
        self.verbose = verbose
    
    def save_state(self):
        self.log.append((self.encoder_data, self.speed_data, self.torque_data, self.tmp_data, self.turns, self.time_data))

    def process_data(self, data: bytes) -> None:
        self.encoder_bytes = data[6:]
        self.speed_bytes = data[4:6]
        self.torque_bytes = data[2:4]
        encoder_temp = int.from_bytes(self.encoder_bytes, byteorder='little')
        speed_temp = int.from_bytes(self.speed_bytes, byteorder='little', signed=True)
        torque_temp = int.from_bytes(self.torque_bytes, byteorder='little', signed=True)
        time_temp = time() - self.time_start
        if self.encoder_data is not None:
            self.process_encoder(encoder_temp, speed_temp, time_temp)
        self.speed_data = speed_temp
        self.encoder_data = encoder_temp
        self.time_data = time_temp
        self.torque_data = torque_temp*0.82
        if self.verbose:
            self.print_state()
        self.save_state()

    def print_state(self):
        pos = np.around(self.encoder_data/self.FULL_TURN*360 + self.turns*360, 2)
        frmt_str = f'POS: {pos:<10} VEL: {self.speed_data:<10} TOR: {self.torque_data}'
        self.frmt_str_len = len(frmt_str)
        sys.stdout.flush()
        sys.stdout.write("\b" * (self.frmt_str_len + 1))
        sys.stdout.write(frmt_str)
    
    def process_encoder(self, next_p, next_s, next_t) -> None:
        threshold_hi = self.thrshld[1]
        threshold_lo = self.thrshld[0]
        prev_p = self.encoder_data
        if prev_p > threshold_hi and next_p <= threshold_lo:
            self.turns += 1
        elif prev_p <= threshold_lo and next_p > threshold_hi:
            self.turns -= 1

    def get_data(self, v_filter: int = 1):
        vel = self.speed_data
        return self.encoder_data + self.turns*self.FULL_TURN, vel, self.encoder_data


    def reset_log(self):
        self.log = []
    
    def write_log(self):
        name = f'MOTOR_LOG.csv'
        try:
            os.remove(name)
        except Exception:
            pass
        with open(name, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile, delimiter=',',
                            quotechar='|', quoting=csv.QUOTE_MINIMAL)
            writer.writerow(['Encoder', 'Speed', 'Torque', 'Temperature', 'Turns', 'Time'])
            writer.writerows(self.log)



class CAN_Motor(CAN_Device):
    _rad_mul = np.pi*2/2**14
    _deg_mul = 360/2**14
    MOTOR_STATUS_BASE: bytes = b'\x9C' + 7*b'\x00'
    TORQUE_CONTROL_BASE: bytes = b'\xA1'
    MOTOR_OFF_BASE: bytes = b'\x80' + 7*b'\x00'
    K_P: float = 0
    K_D: float = 0

    def __init__(self, dev_id: int, bus: CAN_Bus, log: bool = False, verbose=False):
        super().__init__(dev_id, bus)
        self.status = CAN_MotorStatus(log=log, verbose=verbose)

    def _generate_torque_frame(self, val: float, bound = 1.64):
        if val >= 1.64:
            raise ValueError('Torque must be in range [-2000; 2000]')
        val = bound if val > bound else val
        val = -bound if val < -bound else val
        val = int(val/0.00082)
        # print(val)

        val_b = val.to_bytes(2, 'little', signed=True)
        temp = self.TORQUE_CONTROL_BASE + 3*b'\x00' + val_b + 2*b'\x00'
        return temp
    
    def _send_n_rcv_torque(self, val: int, bound=200):
        frame = self._generate_torque_frame(val, bound=bound)
        self.send(frame)
        self.recv_status()

    def gravity_compensation(self, pos, mass=0.062):
        g = 9.82
        return mass*g*np.cos(pos/180*np.pi -np.pi/2)*0.094

    def PD_control(self, q, qdot, Kp=10, Kd=1, Ki=0, bound = 50, v_filter=1):
        self.send(self.MOTOR_STATUS_BASE)
        self.recv_status()
        qr, qdotr = self.status.get_data(v_filter=v_filter)
        qr = self._calc_degrees(qr)
        e = np.abs(qr - q)
        while qdotr > 0 or e > 5:
            qr, qdotr = self.status.get_data(v_filter=v_filter)
            qr = self._calc_degrees(qr)
            t = Kp*(q - qr) + Kd*(qdot - qdotr)
            e = np.abs(qr - q)
            self._send_n_rcv_torque(t, bound=bound)
        self._send_n_rcv_torque(0, bound=bound)
    
    def PID_control(self, P, V, Kp=10, Kd=1, Ki=0, B = 2000, VF=1, R=1, grav_comp=True, desired=False):
        self.send(self.MOTOR_STATUS_BASE)
        self.recv_status()
        i = 0
        errors = []
        timeline = []
        start = time()
        while True:
            qr, qdotr, real_q = self.status.get_data(v_filter=VF)
            qr = self._calc_degrees(qr)
            # if qdotr
            timeline.append(time() - start)
            errors.append(P - qr)
                # print(np.cos(qr/180*np.pi))
            t = Kp*(P - qr) + Kd*(V - qdotr) + Ki*np.trapz(errors, x=timeline)
            if grav_comp:
                g_comp = self.gravity_compensation(qr) if not desired else self.gravity_compensation(P)
                t += g_comp
                # print(t)
            self._send_n_rcv_torque(t, bound = B)

    def FL(self):
        self.send(self.MOTOR_STATUS_BASE)
        self.recv_status()
        control_f = lambda q, dq: 0.2 * (np.sin(time()) - q) + 0.02 * (np.cos(time()) - dq)
        # fl_obj = FeedbackLinearization(0.1*2, 0.062, 0.062*((0.1*2)**2), 0.01, 9.82, control_f)

        while True:
            qr, dq, _ = self.status.get_data(v_filter=1)
            qr = self._calc_radians(qr) - np.pi/2
            dq = dq/180*np.pi
            res_torque = control_f(qr, dq)#fl_obj.control_law(qr, dq)
            self._send_n_rcv_torque(res_torque)

    
    def smooth_off(self, Kd=1):
        self.status.write_log()
        self.send(self.MOTOR_STATUS_BASE)
        self.recv_status()
        con = 2**14
        while self.status.get_data()[1] != 0 and np.abs((self.status.get_data()[0]/con*360)%360) > 3:
            q, qdotr = self.status.get_data()
            q = (q/con*360)%360
            t = 1*(0 - qdotr) + 10*(0 - q)
            self._send_n_rcv_torque(int(t))
        sleep(1)
        self.motor_off()
        sleep(1)

    def reset_log(self):
        self.status.write_log()
        self.status.reset_log()

    def new_trap(self, am, vm, f, x):
        T = 1 / f
        s = np.ceil(x * f / vm)
        n = np.ceil(x * f * f / (am * s))

        while n > s:
            s += 1
            n = np.ceil(x * f * f / (am * s))

        n = np.ceil(x * f * f / (s * am))
        k = s - n
        a = x * f * f / (s * n)
        v = a * n / f
        tc = n * T
        tf = (2 * n + k) * T

        return tc, tf, a, v, k, n

    @curry
    def trap_v_func(self, tc, tf, acc, t):
        if isinstance(t, np.ndarray):
            mask_tc = t < tc
            mask_tc_tftc = (t >= tc) * (t < (tf - tc))
            mask_tftc_tf = t >= (tf - tc)
            res = np.ones(t.shape[0])
            res[mask_tc] *= acc * t[mask_tc]
            res[mask_tc_tftc] *= acc * tc
            res[mask_tftc_tf] *= acc * (tc - (t[mask_tftc_tf] - (tf - tc)))
            return res

        if t < tc:
            return acc * t
        elif t >= tc and t < (tf - tc):
            return acc * tc
        elif t >= (tf - tc):
            return acc * (tc - (t - (tf - tc)))
        else:
            raise ValueError('Out of bounds')

    @curry
    def trap_x_func(self, tc, tf, acc, t):
        tc_path = acc * tc * tc / 2
        tc_tf_path = acc * tc * (tf - 1.5 * tc)
        if t < tc:
            return acc * t * t / 2
        elif t >= tc and t < (tf - tc):
            return acc * tc * (t - tc) + tc_path
        elif t >= (tf - tc) and t <= tf:
            temp = tf - t
            return tc_path + tc_tf_path - acc * temp * temp / 2
        else:
            raise ValueError('Out of bounds')

    def trap_func(self, a, v, x):
        self.send(self.MOTOR_STATUS_BASE)
        self.recv_status()
        tc, tf, a, v, _, _ = self.new_trap(a/180*np.pi, v/180*np.pi, 100, x/180*np.pi)
        vf = self.trap_v_func(tc, tf, a)
        xf = self.trap_x_func(tc, tf, a)
        print(tc, tf)
        # torque = 0.062*((0.094*2)**2)*a
        # torque = 30*0.00082
        # tc = 0.5
        # tf = tc*2
        delta = 0
        start = time()

        while delta <= tf:
            qr, dq, _ = self.status.get_data(v_filter=1)
            qr = self._calc_degrees(qr)
            eq = xf(delta) - qr/180*np.pi
            edq = vf(delta) - dq/180*np.pi
            pd_term = eq*0.3 + edq*0.03
            # if pd_term >= 0.021:
            #     pd_term = 0.021
            # if pd_term <= -0.021:
            #     pd_term = -0.021
            res_torque = pd_term + self.gravity_compensation(qr)
            self._send_n_rcv_torque(res_torque)
            delta = time() - start

        while True:
            qr, _, _ = self.status.get_data(v_filter=1)
            qr = self._calc_degrees(qr)
            res_torque = 0 + self.gravity_compensation(qr)
            self._send_n_rcv_torque(res_torque)

    
    def motor_off(self):
        self.send(self.MOTOR_OFF_BASE)
        
    def _calc_radians(self, val) -> float:
        return val*self._rad_mul
    
    def _calc_degrees(self, val) -> float:
        return val*self._deg_mul

    def _calc_abs_pos(self) -> int:
        return self.hi*self.current_turns + self.current_position
    
    def recv_status(self) -> None:
        self.status.process_data(self.recive())

    def get_position(self, units: Optional[str] = None) -> Union[float, int]:
        pos = self.status.get_data()[0]
        if units is None:
            return pos
        elif units is 'rad':
            return self._calc_radians(pos)
        elif units is 'deg':
            return self._calc_degrees(pos)
        else: 
            raise ValueError(f'No such unit {units}')

    def get_inst_vel(self) -> Optional[float]:
        return self.status.get_data()[1]
