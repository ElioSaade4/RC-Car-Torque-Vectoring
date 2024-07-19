"""my_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
import csv
import math
import random
import numpy as np
from controller import Robot
import matplotlib.pyplot as plt
from slip_fuzzy import initSlipFuzzy, slipCorrection
from yaw_fuzzy import initYawFuzzy, yawCorrection
from utils import initMotors, initSensors, initKalmanFilter, plotCarDyanmics, plotYawRate, plotWheelDynamics, plotTorques, plotOutput


if __name__ == '__main__':
    # create the Robot instance.
    robot = Robot()

    # get the time step of the current world.
    timestep = int(robot.getBasicTimeStep())

    # set simulation duration
    sim_duration = 11.5

    # set Motor voltage
    voltage = 7.4

    ## get all devices from the robot
    
    motor_FR, motor_FL, motor_RR, motor_RL, steering_FR, steering_FL = initMotors(robot, timestep)
    
    accelerometer, gyro, position_FR, position_FL, position_RR, position_RL = initSensors(robot, timestep)
    

    # set constant speed of motors in RAD/S!!
    motor_FR.setVelocity(75.)
    motor_FL.setVelocity(75.)
    motor_RR.setVelocity(75.)
    motor_RL.setVelocity(75.)
    


    ## Arrays to store plotting variables
    time_values = []
    ax_noisy_values = []
    ax_filtered_values = []
    ay_noisy_values = []
    ay_filtered_values = []
    vx_values = []
    vy_values = []
    v_values = []
    yaw_noisy_values = []
    yaw_filtered_values = []
    yaw_ref_values = []
    rpm_FR_values = []
    rpm_FL_values = []
    rpm_RR_values = []
    rpm_RL_values = []
    slip_FR_values = []
    slip_FL_values = []
    slip_RR_values = []
    slip_RL_values = []
    T_FR_values = []
    T_FL_values = []
    T_RR_values = []
    T_RL_values = []

    V_FR_values = []

    ax_ideal = []
    ay_ideal = []
    yaw_ideal = []

    vel_FR_values = []
    vel_FL_values = []
    vel_RR_values = []
    vel_RL_values = []

    sideslip_values = []
    yaw_err_values = []
    yaw_err_dt_values = []

    yaw_corr_values = []

    steering = []


    max_w = 150
    limit_yaw = 1
    limit_yaw_dt = 20
    limit_sideslip = 0.5

    ## Create Kalman Filter to reduce MPU6050 noise
    kf = initKalmanFilter()


    ## Initialize Torque Vectoring Controllers
    YS_controller = initYawFuzzy()
    SR_controller = initSlipFuzzy()


    ## Main loop
    # Initialize main loop values
    Vx_0 = 0
    Vy_0 = 0
    angle_FR_0 = position_FR.getValue()
    angle_FL_0 = position_FL.getValue()
    angle_RR_0 = position_RR.getValue()
    angle_RL_0 = position_RL.getValue()
    w_FR_0 = 0
    w_FL_0 = 0
    w_RR_0 = 0
    w_RL_0 = 0
    yaw_err_0 = 0
    slip_err_FR_0 = 0
    slip_err_FL_0 = 0
    slip_err_RR_0 = 0
    slip_err_RL_0 = 0
    vx_i = 0
    vy_i = 0

    ## time for the sensors to calibrate
    while robot.getTime() < 0.1:
        robot.step(timestep)

    # perform simulation steps until the simulation duration elapses 
    # or until Webots stops the controller
    while robot.step(timestep) != -1:
        time_i = robot.getTime()

        # User Inputs
        if time_i > 5 and time_i < 5.25:
            steering_angle = 0.4 * time_i - 2
        elif time_i >= 5.25 and time_i < 5.75:
            steering_angle = 0.1
        elif time_i >= 5.75 and time_i < 6.25:
            steering_angle = -0.4 * time_i + 2.4
        elif time_i >= 6.25 and time_i < 6.75:
            steering_angle = -0.1
        elif time_i >= 6.75 and time_i < 7.25:
            steering_angle = 0.4 * time_i - 2.8
        elif time_i >= 7.25 and time_i < 7.75:
            steering_angle = 0.1  
        elif time_i >= 7.75 and time_i < 8.25:
            steering_angle = -0.4 * time_i + 3.2 
        elif time_i >= 8.25 and time_i < 8.75:
            steering_angle = -0.1
        elif time_i >= 8.75 and time_i < 9.25:
            steering_angle = 0.4 * time_i -3.6
        elif time_i >= 9.25 and time_i < 9.75:
            steering_angle = 0.1
        elif time_i >= 9.75 and time_i < 10:
            steering_angle = -0.4 * time_i + 4
        else: 
            steering_angle = 0.

        # steering_angle = 0.

        # if time_i > 5 and time_i < 6:
        #     motor_FR.setVelocity(110.)
        #     motor_FL.setVelocity(90.)
        #     motor_RR.setVelocity(110.)
        #     motor_RL.setVelocity(90.)
        # elif time_i >= 6 and time_i < 7:
        #     motor_FR.setVelocity(90.)
        #     motor_FL.setVelocity(110.)
        #     motor_RR.setVelocity(90.)
        #     motor_RL.setVelocity(110.)
        # elif time_i >= 7 and time_i < 8:
        #     motor_FR.setVelocity(110.)
        #     motor_FL.setVelocity(90.)
        #     motor_RR.setVelocity(110.)
        #     motor_RL.setVelocity(90.)
        # else:
        #     motor_FR.setVelocity(100.)
        #     motor_FL.setVelocity(100.)
        #     motor_RR.setVelocity(100.)
        #     motor_RL.setVelocity(100.)

        pwm_FR = 0.5
        pwm_FL = 0.5
        pwm_RR = 0.5
        pwm_RL = 0.5
        
        # Read sensors
        ax, ay, az = accelerometer.getValues()

        ay = max(min(ay, 5), -5)
        # ay *= -1
        pitch, roll, yaw_rate = gyro.getValues()
        #yaw_rate *= (180. / math.pi)
        angle_FR_i = position_FR.getValue()
        angle_FL_i = position_FL.getValue()
        angle_RR_i = position_RR.getValue()
        angle_RL_i = position_RL.getValue()

        Torque_FR = motor_FR.getTorqueFeedback()
        Torque_FL = motor_FL.getTorqueFeedback()
        Torque_RR = motor_RR.getTorqueFeedback()
        Torque_RL = motor_RL.getTorqueFeedback()

        # Add noise to sensors
        noise_i = np.array([[random.uniform(-1.5,1.5)], [random.uniform(-1.5,1.5)], [random.uniform(-0.2, 0.2)]])
        noisy_data_i = np.array([[ax], [ay], [yaw_rate]]) + noise_i

        #Reduce noise using Kalman Filter
        kf.predict()
        kf.update(noisy_data_i)
        filtered_data_i = kf.x

        ## Process sensor data
        # Car Velocities
        # vx_i = Vx_0 + (filtered_data_i[0] * timestep / 1000) 
        # vy_i = Vy_0 + (filtered_data_i[1] * timestep / 1000)
        vx_i = Vx_0 + ((ax + yaw_rate * Vy_0) * timestep / 1000) 
        vy_i = Vy_0 + ((ay - yaw_rate * Vx_0) * timestep / 1000)
        v_i = math.sqrt(vx_i**2 + vy_i**2)

        # Wheels Angular Velocity
        w_FR_i = (angle_FR_i - angle_FR_0) / (timestep / 1000)
        w_FL_i = (angle_FL_i - angle_FL_0) / (timestep / 1000)
        w_RR_i = (angle_RR_i - angle_RR_0) / (timestep / 1000)
        w_RL_i = (angle_RL_i - angle_RL_0) / (timestep / 1000)

        # Wheels Speed (RPM)
        rpm_FR_i = w_FR_i * 9.5492968
        rpm_FL_i = w_FL_i * 9.5492968
        rpm_RR_i = w_RR_i * 9.5492968
        rpm_RL_i = w_RL_i * 9.5492968

        # Wheels Slip Ratio
        slip_FR_i = ((w_FR_i * 0.048) - (vx_i * math.cos(steering_angle))) / (vx_i * math.cos(steering_angle))
        slip_FL_i = ((w_FL_i * 0.048) - (vx_i * math.cos(steering_angle))) / (vx_i * math.cos(steering_angle))
        slip_RR_i = ((w_RR_i * 0.048) - vx_i) / vx_i 
        slip_RL_i = ((w_RL_i * 0.048) - vx_i) / vx_i

        # Reference Yaw Rate
        yaw_ref_i = ((vx_i * math.tan(steering_angle)) / 0.23)
        

        # Motor Calculations
        ##IDEA: Use the set velocity function, but at the same time, limit the torque
        T_FR_i = ((pwm_FR * voltage * 0.00402273) - (w_FR_0 * 0.00006835)) * 3.2
        T_FL_i = ((pwm_FL * voltage * 0.00402273) - (w_FL_0 * 0.00006835)) * 3.2
        T_RR_i = ((pwm_RR * voltage * 0.00402273) - (w_RR_0 * 0.00006835)) * 3.2
        T_RL_i = ((pwm_RL * voltage * 0.00402273) - (w_RL_0 * 0.00006835)) * 3.2

        if time_i >= 4.9:
            motor_FR.setAvailableTorque(0.35)
            motor_FL.setAvailableTorque(0.35)
            motor_RR.setAvailableTorque(0.35)
            motor_RL.setAvailableTorque(0.35)
        else:
            motor_FR.setAvailableTorque(0.1)
            motor_FL.setAvailableTorque(0.1)
            motor_RR.setAvailableTorque(0.1)
            motor_RL.setAvailableTorque(0.1)


        ## Control 
        # Yaw rate Error
        yaw_err = float(yaw_ref_i - yaw_rate)#filtered_data_i[2])

        #Yaw Rate Error Derivative
        yaw_err_dt = float((yaw_err - yaw_err_0) / (timestep / 1000))

        # Sideslip Angle
        sideslip = math.atan(vy_i / vx_i)

        

        yaw_corr = yawCorrection(YS_controller, max(min(yaw_err, limit_yaw), -limit_yaw),\
                                 max(min(yaw_err_dt, limit_yaw_dt), -limit_yaw_dt),\
                                 max(min(-sideslip, limit_sideslip), -limit_sideslip))

        # # Slip Ratio Errors
        # slip_err_FR = slip_FR_i - 0.2
        # slip_err_FL = slip_FL_i - 0.2
        # slip_err_RR = slip_RR_i - 0.2
        # slip_err_RL = slip_RL_i - 0.2

        # # Slip Ratio Error Derivatives
        # slip_err_dt_FR = (slip_err_FR - slip_err_FR_0) / (timestep / 1000)
        # slip_err_dt_FL = (slip_err_FL - slip_err_FL_0) / (timestep / 1000)
        # slip_err_dt_RR = (slip_err_RR - slip_err_RR_0) / (timestep / 1000)
        # slip_err_dt_RL = (slip_err_RL - slip_err_RL_0) / (timestep / 1000)

        #print("Slip Error: ", slip_err_FR)
        #print("Error Derivative: ", slip_err_dt_FR)

        # # Slip Ratio Speed Corrections
        # if slip_FR_i > 0.2:
        #     slip_corr_FR = slipCorrection(SR_controller, slip_err_FR, slip_err_dt_FR)
        # else: 
        #     slip_corr_FR = 0

        # if slip_FL_i > 0.2:
        #     slip_corr_FL = slipCorrection(SR_controller, slip_err_FL, slip_err_dt_FL)
        # else:
        #     slip_corr_FL = 0

        # if slip_RR_i > 0.2:
        #     slip_corr_RR = slipCorrection(SR_controller, slip_err_RR, slip_err_dt_RR)
        # else: 
        #     slip_corr_RR = 0

        # if slip_RL_i > 0.2:
        #     slip_corr_RL = slipCorrection(SR_controller, slip_err_RL, slip_err_dt_RL)
        # else: 
        #     slip_corr_RL = 0

        
        # yaw_corr = 0

        ## Actuator commands
        steering_FR.setPosition(steering_angle)
        steering_FL.setPosition(steering_angle)  

        vel_FR = float(max(min(pwm_FR + yaw_corr, 1), -1) * max_w)
        vel_FL = float(max(min(pwm_FL - yaw_corr, 1), -1) * max_w)
        vel_RR = float(max(min(pwm_RR + yaw_corr, 1), -1) * max_w)
        vel_RL = float(max(min(pwm_RL - yaw_corr, 1), -1) * max_w)

        # print(vel_FR)

        motor_FR.setVelocity(vel_FR)
        motor_FL.setVelocity(vel_FL)
        motor_RR.setVelocity(vel_RR)
        motor_RL.setVelocity(vel_RL)
        
        

        # store values for plotting
        time_values.append(time_i)
        ax_noisy_values.append(noisy_data_i[0][0])
        ax_filtered_values.append(filtered_data_i[0][0])
        ay_noisy_values.append(noisy_data_i[1][0])
        ay_filtered_values.append(filtered_data_i[1][0])
        vx_values.append(vx_i)
        vy_values.append(vy_i)
        v_values.append(v_i)
        yaw_noisy_values.append(noisy_data_i[2][0])
        yaw_filtered_values.append(filtered_data_i[2][0])
        yaw_ref_values.append(yaw_ref_i)
        rpm_FR_values.append(rpm_FR_i)
        rpm_FL_values.append(rpm_RL_i)
        rpm_RR_values.append(rpm_RR_i)
        rpm_RL_values.append(rpm_RL_i)
        slip_FR_values.append(slip_FR_i)
        slip_FL_values.append(slip_FL_i)
        slip_RR_values.append(slip_RR_i)
        slip_RL_values.append(slip_RL_i)
        T_FR_values.append(Torque_FR)
        T_FL_values.append(Torque_FL)
        T_RR_values.append(T_RR_i)
        T_RL_values.append(T_RL_i)

        ax_ideal.append(ax)
        ay_ideal.append(ay)
        yaw_ideal.append(yaw_rate)

        vel_FR_values.append(vel_FR)
        vel_FL_values.append(vel_FL)
        vel_RR_values.append(vel_RR)
        vel_RL_values.append(vel_RL)

        sideslip_values.append(sideslip)
        yaw_err_values.append(yaw_err)
        yaw_err_dt_values.append(yaw_err_dt)

        yaw_corr_values.append(yaw_corr)

        steering.append(steering_angle)

        
        # check simulation duration
        if time_i > sim_duration:
            break


        # values for next iteration
        Vx_0 = vx_i
        Vy_0 = vy_i
        angle_FR_0 = angle_FR_i
        angle_FL_0 = angle_FL_i
        angle_RR_0 = angle_RR_i
        angle_RL_0 = angle_RL_i
        w_FR_0 = w_FR_i
        w_FL_0 = w_FL_i
        w_RR_0 = w_RR_i
        w_RL_0 = w_RL_i
        yaw_err_0 = yaw_err
        # slip_err_FR_0 = slip_err_FR
        # slip_err_FL_0 = slip_err_FL
        # slip_err_RR_0 = slip_err_RR
        # slip_err_RL_0 = slip_err_RL


    ## Plot stored variables
    # Longitudinal Car Dynamics
    plotCarDyanmics(time_values, ax_noisy_values, ax_filtered_values, ax_ideal, vx_values, 'x')

    # Lateral Car Dynamics
    plotCarDyanmics(time_values, ay_noisy_values, ay_filtered_values, ay_ideal, vy_values, 'y')

    # Yaw Rate
    plotYawRate(time_values, yaw_noisy_values, yaw_filtered_values, yaw_ref_values, yaw_ideal)

    # Front Right Wheel Rotation
    # plotWheelDynamics(time_values, rpm_FR_values, slip_FR_values, 'FR')
    
    # Front Left Wheel Rotation
    # plotWheelDynamics(time_values, rpm_FL_values, slip_FL_values, 'FL')

    # Rear Right Wheel Rotation
    # plotWheelDynamics(time_values, rpm_RR_values, slip_RR_values, 'RR')

    # Rear Left Wheel Rotation
    # plotWheelDynamics(time_values, rpm_RL_values, slip_RL_values, 'RL')

    # Wheel Torques & Speed Commands
    # plotTorques(time_values, T_FR_values, T_FL_values, T_RR_values, T_RL_values, 'torques')
    # plotTorques(time_values, vel_FR_values, vel_FL_values, vel_RR_values, vel_RL_values, 'Wheel Velocity')

    # Velocity
    # plotOutput(time_values, v_values, 'Car Velocity')
    
    # Sideslip Angle
    plotOutput(time_values, sideslip_values, 'Sideslip')

    # Yaw Rate Error
    plotOutput(time_values, yaw_err_values, 'Yaw Rate Error')

    # Yaw Rate Error Derivative
    plotOutput(time_values, yaw_err_dt_values, 'Yaw Rate Error Derivative')

    # Yaw Correction
    plotOutput(time_values, yaw_corr_values, 'Yaw Rate Correction')

    # Steering Angle
    plotOutput(time_values, steering, 'Steerign Angle')

    rows = zip(time_values, yaw_noisy_values, yaw_filtered_values, yaw_ref_values, ax_noisy_values, ax_filtered_values, vx_values, yaw_err_values, yaw_err_dt_values, yaw_corr_values, steering, sideslip_values, ay_noisy_values, ay_filtered_values, vy_values)
    with open("Trial.csv", "w") as f:
        writer = csv.writer(f)
        writer.writerow(["Time Values", "Noisy Yaw", "Filtered Yaw", "Reference Yaw", "Noisy Ax", "Filtered Ax", "Vx Values", "Yaw Error", "Yaw Error Derivatives", "Yaw Correction", "Steering Values", "Sideslip", "Noisy Ay", "Filtered Ay", "Vy"])
        for row in rows:
            writer.writerow(row)
    print("Points saved")

    plt.show()