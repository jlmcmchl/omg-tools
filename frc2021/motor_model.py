import numpy as np


class MotorModel(object):
    def __init__(self, params):
        # RPM
        self.free_speed = params["free speed"]
        # A
        self.free_current = params["free current"]
        # W
        self.max_power = params["maximum power"]
        # N*m
        self.stall_torque = params["stall torque"]
        # A
        self.stall_current = params["stall current"]
        # V
        self.test_voltage = params["test_voltage"]


class DTModel(object):
    def __init__(self, param):
        # $C$3
        self.mass = param["mass"]
        # $C$4
        self.in_od = param["in OD"]
        # $C$5
        self.ratio = param["ratio"]
        # $C$6
        self.v_ocv = param["V_ocv"]
        # $C$7
        self.r_bat = param["R_bat"]
        # $C$8
        self.n_motors = param["# of motors"]
        # $C$9
        self.i_lim = param["I Limit"]
        # $C$10
        self.mu = param["mu"]
        # other
        self.dt = param["dt"]
        self.track_width = param['track width']
        self.motor_model = MotorModel(param["motor model"])

    def init(self):
        # C13 + dt
        t = 0
        # C14+J14*$B$15
        v = 0
        # C14/($C$4*PI())*12*60*$C$5
        motor_speed = v / (np.pi * self.in_od) * 12 * 60 * self.ratio
        # $C$6*(257-(257-1.5)*(D14/6380))/(12+$C$7*$C$8)*$C$8
        i_req = (
            self.v_ocv
            * (
                self.motor_model.stall_current
                - (self.motor_model.stall_current - self.motor_model.free_current)
                * (motor_speed / self.motor_model.free_speed)
            )
            / (12 + self.r_bat * self.n_motors)
            * self.n_motors
        )
        # MIN($C$9*$C$8,E14)
        i = min(i_req, self.i_lim * self.n_motors)
        # $C$6-F14*$C$7
        vbat = self.v_ocv - i * self.r_bat
        # 4.69*(1-D14/6380)*G14/12*F14/E14
        torque = (
            self.motor_model.stall_torque
            * (1 - motor_speed / self.motor_model.free_speed)
            * vbat
            / self.motor_model.test_voltage
            * i
            / i_req
        )
        # H14*$C$5*0.9*2/($C$4*25.4/1000)*$C$8
        force_to_carpet = (
            torque
            * self.ratio
            * self.mu
            * 2
            / (self.in_od * 25.4 / 1000.0)
            * self.n_motors
        )
        # =I14/$C$3*1000/25.4/12
        accel = force_to_carpet / self.mass * 1000 / 25.4 / 12
        # I14/$C$10*0.224809
        down_force_req = force_to_carpet / self.mu * 0.224809
        # K14-$C$3*2.2
        vacuum_req = down_force_req - self.mass * 2.2

        return {
            "t": t,
            "v": v,
            "motor_speed": motor_speed,
            "i_req": i_req,
            "i": i,
            "vbat": vbat,
            "torque": torque,
            "force_to_carpet": force_to_carpet,
            "accel": accel,
            "down_force_req": down_force_req,
            "vacuum_req": vacuum_req,
        }

    def iter(self, state, dt):
        if state is None:
            return self.init()

        t = state["t"] + dt
        # C13 + dt
        t = 0
        # C14+J14*$B$15
        v = state["v"] + state["accel"] * dt
        # C14/($C$4*PI())*12*60*$C$5
        motor_speed = v / (np.pi * self.in_od) * 12 * 60 * self.ratio
        # $C$6*(257-(257-1.5)*(D14/6380))/(12+$C$7*$C$8)*$C$8
        i_req = (
            self.v_ocv
            * (
                self.motor_model.stall_current
                - (self.motor_model.stall_current - self.motor_model.free_current)
                * (motor_speed / self.motor_model.free_speed)
            )
            / (12 + self.r_bat * self.n_motors)
            * self.n_motors
        )
        # MIN($C$9*$C$8,E14)
        i = min(i_req, self.i_lim * self.n_motors)
        # $C$6-F14*$C$7
        vbat = self.v_ocv - i * self.r_bat
        # 4.69*(1-D14/6380)*G14/12*F14/E14
        torque = (
            self.motor_model.stall_torque
            * (1 - motor_speed / self.motor_model.free_speed)
            * vbat
            / self.motor_model.test_voltage
            * i
            / i_req
        )
        # H14*$C$5*0.9*2/($C$4*25.4/1000)*$C$8
        force_to_carpet = (
            torque
            * self.ratio
            * self.mu
            * 2
            / (self.in_od * 25.4 / 1000.0)
            * self.n_motors
        )
        # =I14/$C$3*1000/25.4/12
        accel = force_to_carpet / self.mass * 1000 / 25.4 / 12
        # I14/$C$10*0.224809
        down_force_req = force_to_carpet / self.mu * 0.224809
        # K14-$C$3*2.2
        vacuum_req = down_force_req - self.mass * 2.2

        return {
            "t": t,
            "v": v,
            "motor_speed": motor_speed,
            "i_req": i_req,
            "i": i,
            "vbat": vbat,
            "torque": torque,
            "force_to_carpet": force_to_carpet,
            "accel": accel,
            "down_force_req": down_force_req,
            "vacuum_req": vacuum_req,
        }

    def time_to_current_limit(self, dt):
        t = 0
        state = self.init()
        while state["i_req"] > state["i"]:
            t += dt
            state = self.iter(state, dt)
        return t, state

    def max_accel(self, ratio=None, in_od=None):
        ratio = ratio if ratio else self.ratio
        in_od = in_od if in_od else self.in_od
        i_req = (
            self.v_ocv
            * self.motor_model.stall_current
            / (12 + self.r_bat * self.n_motors)
            * self.n_motors
        )
        i = min(i_req, self.i_lim * self.n_motors)
        vbat = self.v_ocv - i * self.r_bat
        torque = (
            self.motor_model.stall_torque
            * vbat
            / self.motor_model.test_voltage
            * i
            / i_req
        )
        force_to_carpet = (
            torque
            * ratio
            * self.mu
            * 2
            / (in_od * 25.4 / 1000.0)
            * self.n_motors
        )
        return force_to_carpet / self.mass * 1000 / 25.4 / 12

    def v_at_current_limit(self):
        return (self.motor_model.stall_current - self.i_lim * (12 + self.r_bat * self.n_motors) / self.v_ocv) * self.motor_model.free_speed / (self.motor_model.stall_current - self.motor_model.free_current)

    def imperial_v_at_current_limit(self, ratio=None, in_od=None):
        ratio = ratio if ratio else self.ratio
        in_od = in_od if in_od else self.in_od
        return self.v_at_current_limit() * in_od * np.pi / (12 * 60 * ratio)

    def turn_rad_per_sec_at_current_limit(self, ratio=None, in_od=None):
        v = self.imperial_v_at_current_limit(ratio, in_od)
        return 2 * v / self.track_width



PROPOSED_DT_MODEL = DTModel(
    {
        "mass": 58.0 / 2.2,
        "in OD": 4.0,
        "ratio": 6.0,
        "V_ocv": 13.2,
        "R_bat": 0.013,
        "# of motors": 6.0,
        "I Limit": 55.0,
        "mu": 0.9,
        "dt": 0.01,
        'track width': 2,
        "motor model": {
            "free speed": 6380.0,
            "free current": 1.5,
            "maximum power": 783.0,
            "stall torque": 4.69,
            "stall current": 257.0,
            "test_voltage": 12.0,
        },
    }
)
