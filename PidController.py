class PidController:

    def __init__(self, ts, kp=0.1, kd=0, ki=0):
        self.__kp = kp
        self.__kd = kd / ts
        self.__ki = ki * ts
        self.__ts = ts
        self.__error_previous = None
        self.__sum_errors = 0

    def control(self, y, set_point=0):
        error = set_point - y
        u = self.__kp * error

        if self.__error_previous is not None:
            u += self.__kd * (error - self.__error_previous)

        u += self.__ki * self.__sum_errors
        self.__error_previous = error
        self.__sum_errors += error

        return u
