import numpy as np
import matplotlib.pyplot as plt
from filterpy.kalman import KalmanFilter

def initMotors(robot, timestep):
    # rotation motors
    motor_FR = robot.getDevice('front_right_wheel')
    motor_FL = robot.getDevice('front_left_wheel')
    motor_RR = robot.getDevice('rear_right_wheel')
    motor_RL = robot.getDevice('rear_left_wheel')
    motor_FR.setPosition(float('inf'))
    motor_FL.setPosition(float('inf'))
    motor_RR.setPosition(float('inf'))
    motor_RL.setPosition(float('inf'))
    motor_FR.enableTorqueFeedback(timestep)
    motor_FL.enableTorqueFeedback(timestep)
    motor_RR.enableTorqueFeedback(timestep)
    motor_RL.enableTorqueFeedback(timestep)

    # steering motors
    steering_FR = robot.getDevice('right_steer')
    steering_FL = robot.getDevice('left_steer')
    
    return motor_FR, motor_FL, motor_RR, motor_RL, steering_FR, steering_FL


def initSensors(robot, timestep):
    # accelerometer
    accelerometer = robot.getDevice('accelerometer')
    accelerometer.enable(timestep)

    # gyroscope
    gyro = robot.getDevice('gyro')
    gyro.enable(timestep)

    # Wheel position
    position_FR = robot.getDevice('position_FR')
    position_FL = robot.getDevice('position_FL')
    position_RR = robot.getDevice('position_RR')
    position_RL = robot.getDevice('position_RL')
    position_FR.enable(timestep)
    position_FL.enable(timestep)
    position_RR.enable(timestep)
    position_RL.enable(timestep)

    return accelerometer, gyro, position_FR, position_FL, position_RR, position_RL


def initKalmanFilter():
    kf = KalmanFilter(dim_x = 3, dim_z = 3)
    kf.x = np.array([[0.],
                    [0.],
                    [0.]])
    kf.F = np.eye(3)
    kf.H = np.eye(3)
    kf.P *= 10.
    kf.R = np.eye(3) * 20
    kf.Q = np.eye(3) * 1

    return kf


def plotCarDyanmics(time, noisy, a_filtered, a_ideal, speed, direction):
    plt.figure(direction + ' Dynamics')
    plt.subplot(1,2,1)
    plt.plot(time, noisy, label= 'Noisy')
    plt.plot(time, a_filtered, label= 'Filtered')
    plt.plot(time, a_ideal, label = 'ideal')
    plt.legend()
    plt.xlabel('Time (s)')
    plt.ylabel('A' + direction + ' (m/s^2)')
    plt.grid()
    plt.title('A' + direction + ' vs. Time')
    plt.subplot(1,2,2)
    plt.plot(time, speed)
    plt.xlabel('Time (s)')
    plt.ylabel('V' + direction + ' (m/s)')
    plt.grid()
    plt.title('V' + direction + ' vs. Time')


def plotYawRate(time, actual_yaw, filtered_yaw, ref_yaw, yaw_ideal):
    plt.figure("Yaw Rate")
    plt.plot(time, actual_yaw, label= 'Noisy')
    plt.plot(time, filtered_yaw, label= 'Filtered')
    plt.plot(time, ref_yaw, label= 'Reference')
    plt.plot(time, yaw_ideal, label=' Ideal')
    plt.legend()
    plt.xlabel('Time (s)')
    plt.ylabel('Yaw Rate (rad/s)')
    plt.grid()
    plt.title('Yaw Rate vs. Time')


def plotWheelDynamics(time, rpm, slip_ratio, wheel):
    plt.figure(wheel + ' Wheel Rotation')
    plt.subplot(1,2,1)
    plt.plot(time, rpm)
    #plt.ylim([2500,3500])
    plt.xlabel('Time (s)')
    plt.ylabel(wheel + ' Wheel Speed (rpm)')
    plt.grid()
    plt.title('RPM '+ wheel + ' vs. Time')
    plt.subplot(1,2,2)
    plt.plot(time, slip_ratio)
    plt.xlabel('Time (s)')
    plt.ylabel(wheel + ' Wheel Slip Ratio')
    plt.grid()
    plt.title('Slip Ration ' + wheel + ' vs. Time')


def plotTorques(time, T_FR, T_FL, T_RR, T_RL, name):
    plt.figure(name)
    plt.plot(time, T_FR, label= 'FR')
    plt.plot(time, T_FL, label= 'FL')
    plt.plot(time, T_RR, label= 'RR')
    plt.plot(time, T_RL, label= 'RL')
    plt.legend()
    plt.grid()
    plt.xlabel('Time (s)')
    plt.ylabel('Wheel Torques (N.m)')


def plotOutput(time, output, name):
    plt.figure(name)
    plt.plot(time, output)
    plt.grid()
    plt.xlabel('Time (s)')
    plt.ylabel(name)