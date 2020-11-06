from math import radians

from matplotlib import pyplot
from numpy import vstack, arange, array

from Car import Car
from PidController import PidController


def question_1():
    car = Car(y_position=0.3, theta=radians(5))
    steering_angle = radians(-2)
    time_span = 2

    x_cache = array([car.get_x_position()])
    y_cache = array([car.get_y_position()])
    theta_cache = array([car.get_theta()])
    samples = 1000
    sampling_interval = time_span / samples
    for t in range(samples):
        car.move(steering_angle, sampling_interval)
        x_cache = vstack((x_cache, [car.get_x_position()]))
        y_cache = vstack((y_cache, [car.get_y_position()]))
        theta_cache = vstack((theta_cache, [car.get_theta()]))

    time_span = sampling_interval * arange(samples + 1)
    pyplot.plot(time_span, x_cache)
    pyplot.xlabel('Time (s)')
    pyplot.ylabel('x Position (m)')
    pyplot.grid()
    pyplot.savefig('figures\\question_1_1_a.eps', format='eps')
    pyplot.show()

    pyplot.plot(time_span, y_cache)
    pyplot.xlabel('Time (s)')
    pyplot.ylabel('y Position (m)')
    pyplot.grid()
    pyplot.savefig('figures\\question_1_1_b.eps', format='eps')
    pyplot.show()

    pyplot.plot(time_span, theta_cache)
    pyplot.xlabel('Time (s)')
    pyplot.ylabel('Theta (rad)')
    pyplot.grid()
    pyplot.savefig('figures\\question_1_1_c.eps', format='eps')
    pyplot.show()

    pyplot.plot(x_cache, y_cache)
    pyplot.xlabel('x Position (m)')
    pyplot.ylabel('y Position (m)')
    pyplot.grid()
    pyplot.savefig('figures\\question_1_2.eps', format='eps')
    pyplot.show()


def question_2():
    sampling_interval = 0.025
    time_span = 50
    samples = int(time_span / sampling_interval)

    kps = [0.1, 0.2, 0.3, 0.4, 0.5]

    results = []

    for kp in kps:
        car = Car(steering_disturbance=radians(1))
        pid = PidController(sampling_interval, kp=kp)
        x_cache = array([car.get_x_position()])
        y_cache = array([car.get_y_position()])

        for t in range(samples):
            steering_angle = pid.control(car.get_y_position())
            car.move(steering_angle, sampling_interval)
            x_cache = vstack((x_cache, [car.get_x_position()]))
            y_cache = vstack((y_cache, [car.get_y_position()]))

        results.append([x_cache, y_cache])

    pyplot.plot(results[0][0], results[0][1], label="K$_p$ = 0.1")
    pyplot.plot(results[1][0], results[1][1], label="K$_p$ = 0.2")
    pyplot.plot(results[2][0], results[2][1], label="K$_p$ = 0.3")
    pyplot.plot(results[3][0], results[3][1], label="K$_p$ = 0.4")
    pyplot.plot(results[4][0], results[4][1], label="K$_p$ = 0.5")
    pyplot.xlabel('x Position (m)')
    pyplot.ylabel('y Position (m)')
    pyplot.legend()
    pyplot.grid()
    pyplot.savefig('figures\\question_2_1.eps', format='eps')
    pyplot.show()

    kds = [0.1, 0.2, 0.3, 0.4, 0.5]

    results = []

    for kd in kds:
        car = Car(steering_disturbance=radians(1))
        pid = PidController(sampling_interval, kd=kd)
        x_cache = array([car.get_x_position()])
        y_cache = array([car.get_y_position()])

        for t in range(samples):
            steering_angle = pid.control(car.get_y_position())
            car.move(steering_angle, sampling_interval)
            x_cache = vstack((x_cache, [car.get_x_position()]))
            y_cache = vstack((y_cache, [car.get_y_position()]))

        results.append([x_cache, y_cache])

    pyplot.plot(results[0][0], results[0][1], label="K$_d$ = 0.1")
    pyplot.plot(results[1][0], results[1][1], label="K$_d$ = 0.2")
    pyplot.plot(results[2][0], results[2][1], label="K$_d$ = 0.3")
    pyplot.plot(results[3][0], results[3][1], label="K$_d$ = 0.4")
    pyplot.plot(results[4][0], results[4][1], label="K$_d$ = 0.5")
    pyplot.xlabel('x Position (m)')
    pyplot.ylabel('y Position (m)')
    pyplot.legend()
    pyplot.grid()
    pyplot.savefig('figures\\question_2_2.eps', format='eps')
    pyplot.show()

    car = Car(steering_disturbance=radians(1))
    pid = PidController(sampling_interval, kp=0.4, kd=0.5, ki=0.1)

    u_cache = array([0])
    x_cache = array([car.get_x_position()])
    y_cache = array([car.get_y_position()])
    for t in range(samples):
        steering_angle = pid.control(car.get_y_position())
        car.move(steering_angle, sampling_interval)
        u_cache = vstack((u_cache, steering_angle))
        x_cache = vstack((x_cache, [car.get_x_position()]))
        y_cache = vstack((y_cache, [car.get_y_position()]))

    time_span = sampling_interval * arange(samples + 1)
    pyplot.plot(time_span, u_cache)
    pyplot.grid()
    pyplot.xlabel('Time (s)')
    pyplot.ylabel('u (rad)')
    pyplot.savefig('figures\\question_2_3_a.eps', format='eps')
    pyplot.show()

    pyplot.plot(x_cache, y_cache)
    pyplot.xlabel('x Position (m)')
    pyplot.ylabel('y Position (m)')
    pyplot.grid()
    pyplot.savefig('figures\\question_2_3_b.eps', format='eps')
    pyplot.show()


def main():
    question_1()
    question_2()


if __name__ == '__main__':
    main()
