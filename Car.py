from numpy import cos, sin, tan, linspace
from scipy.integrate import solve_ivp


class Car:

    def __init__(self, length=2.3, velocity=5, x_position=0, y_position=0, theta=0):
        self.__length = length
        self.__velocity = velocity
        self.__x_position = x_position
        self.__y_position = y_position
        self.__theta = theta

    def move(self, steering_angle, dt):
        z_initial = [self.__x_position,
                     self.__y_position,
                     self.__theta]

        number_of_points = 100
        solution = solve_ivp(self.__system_dynamics, [0, dt], z_initial, args=[steering_angle],
                             t_eval=linspace(0, dt, number_of_points))

        self.__x_position = solution.y[0][-1]
        self.__y_position = solution.y[1][-1]
        self.__theta = solution.y[2][-1]

    def __system_dynamics(self, t, z, u):
        theta = z[2]
        return [self.__velocity * cos(theta),
                self.__velocity * sin(theta),
                self.__velocity * tan(u) / self.__length]

    def get_x_position(self):
        return self.__x_position

    def get_y_position(self):
        return self.__y_position

    def get_pose(self):
        return self.__theta
