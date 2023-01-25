import math

class Pid:
    def __init__(self, kp, ki, kd) -> None:
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.reset()

    def __call__(self, *args, **kwargs):
        return self.calc(*args, **kwargs)

    def calc(self, err):
        self.out_p = err*self.kp
        self.out_i += err*self.ki
        # call calc() with fixed interval
        self.out_d = (err-self.err_last)*self.kd
        self.err_last = err
        return self.out_p + self.out_i + self.out_d

    def reset(self):
        self.out_p = 0
        self.out_i = 0
        self.out_d = 0
        self.err_last = 0
