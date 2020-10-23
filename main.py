from matplotlib import pyplot
from numpy import vstack, arange, array

from Car import Car
from PidController import PidController


def main():
    car = Car(y_position=0.5)
    sampling_interval = 0.01
    pid = PidController(sampling_interval, kd=0.2)
    
    y_cache = array([car.get_y_position()])
    samples = 2000
    for t in range(samples):
        steering_angle = pid.control(car.get_y_position())
        car.move(steering_angle, sampling_interval)
        y_cache = vstack((y_cache, [car.get_y_position()]))
    
    time_span = sampling_interval * arange(samples + 1)
    pyplot.plot(time_span, y_cache)
    pyplot.grid()
    pyplot.xlabel('Time (s)')
    pyplot.ylabel('y (m)')
    pyplot.show()


if __name__ == '__main__':
    main()
