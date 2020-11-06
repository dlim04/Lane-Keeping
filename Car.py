from numpy import cos, sin, tan, linspace
from scipy.integrate import solve_ivp


class Car:
    """
    Class to define the Car object which will be the target of the model.
    """

    def __init__(self, length=2.3, velocity=5, x_position=0, y_position=0, steering_disturbance=0, theta=0):
        """
        Constructor for class Car.
        :param length: The length of the car as a float in meters
        :param velocity: The modulus of the velocity as as float in m s ^-1
        :param x_position: The x position of the car in relation to the origin as a float in meters
        :param y_position: The y position of the car in relation to the origin as a float in meters
        :param steering_disturbance: The additive disturbance in the car to be added to steering angle
        :param theta: The argument of the velocity as a float in radians
        """
        self.__length = length
        self.__velocity = velocity
        self.__x_position = x_position
        self.__y_position = y_position
        self.__steering_disturbance = steering_disturbance
        self.__theta = theta

    def move(self, steering_angle, dt):
        """
        Public method to move the car in the x, y plane with respect to time.
        :param steering_angle: The angle the car is currently steering at as a float in radians
        :param dt: The time the move takes place over as a float in seconds
        """
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
        """
        Private helper method to be used by move to describe the attributes of the car at a point in time.
        :param t: An array where the first element is the start time and the second element is the end time as floats
        :param z: An array containing the state of the car where the first element is the x position, the second is the
        y position and the 3rd is the orientation of the car where the normal is the orientation of the car in relation
        to the positive x axis where positive is in the anti-clockwise direction
        :param u: The steering angle as a float in radians from the same normal sa the orientation
        :return: A new z based on the current steering angle of the car
        """
        theta = z[2]
        return [self.__velocity * cos(theta),
                self.__velocity * sin(theta),
                self.__velocity * tan(u + self.__steering_disturbance) / self.__length]

    def get_x_position(self):
        """
        Getter for the x position of the car.
        :return: The x position in meters as a float
        """
        return self.__x_position

    def get_y_position(self):
        """
        Getter for the y position of the car.
        :return: The y position in meters as a float
        """
        return self.__y_position

    def get_theta(self):
        """
        Getter for the current angle of the car in relation to the positive x axis.
        :return: The angle of the car as a float in radians
        """
        return self.__theta
