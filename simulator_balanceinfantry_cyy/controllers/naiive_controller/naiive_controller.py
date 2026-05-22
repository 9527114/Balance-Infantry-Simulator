"""my_controller controller."""

from controller import Robot, Motor, Keyboard
import numpy as np
import scipy.io as sio
import math

PI = 3.1415926535

# 加载控制参数
const_set = sio.loadmat("../../LQR_control/const_set(1206).mat")
const_set = const_set['const_set']

def CalculateTheta(theta1, theta4, pitch):
    xd = 0.15 * np.cos(theta4) + 0.108 / 2
    yd = 0.15 * np.sin(theta4)
    xb = 0.15 * np.cos(theta1) - 0.108 / 2
    yb = 0.15 * np.sin(theta1)
    A0 = 2 * 0.25 * (xd - xb)
    B0 = 2 * 0.25 * (yd - yb)
    lbd = np.sqrt((xb - xd) * (xb - xd) + (yb - yd) * (yb - yd))
    C0 = lbd * lbd
    theta2 = 2 * np.arctan2(B0 + np.sqrt(A0 * A0 + B0 * B0 - C0 * C0), A0 + C0)
    xc = 0.15 * np.cos(theta1) + 0.25 * np.cos(theta2) - 0.108 / 2
    yc = 0.15 * np.sin(theta1) + 0.25 * np.sin(theta2)
    length = np.sqrt(xc * xc + yc * yc)
    theta = np.pi / 2 - (np.arctan2(yc, xc) - pitch)

    phi0 = np.arctan2(yc, xc) 
    phi4 = theta4

    theta3 = np.arctan2((length*np.sin(phi0) - 0.15*np.sin(phi4)),(length * np.cos(phi0) - 0.15*np.cos(phi4) - 0.054))    
    return length, theta, theta2, theta3, phi0

# 初始化机器人
robot = Robot()
timestep = int(32)

# 初始化电机和传感器
motor1 = robot.getDevice('motor_1')
motor2 = robot.getDevice('motor_2')
motor3 = robot.getDevice('motor_3')
motor4 = robot.getDevice('motor_4')

position_sensor1 = motor1.getPositionSensor()
position_sensor2 = motor2.getPositionSensor()
position_sensor3 = motor3.getPositionSensor()
position_sensor4 = motor4.getPositionSensor()

position_sensor1.enable(timestep)
position_sensor2.enable(timestep)
position_sensor3.enable(timestep)
position_sensor4.enable(timestep)

imu = robot.getDevice("imu")
imu.enable(1)
gyro = robot.getDevice("gyro")
wheel_gyro = robot.getDevice("wheel_gyro")
wheel_gyro1 = robot.getDevice("wheel_gyro1")
gyro.enable(1)
wheel_gyro.enable(1)
wheel_gyro1.enable(1)

wheel_motor = robot.getDevice('wheel_motor')
wheel_motor1 = robot.getDevice('wheel_motor1')

# 初始化键盘控制
keyboard = Keyboard()
keyboard.enable(timestep)

# 控制参数
max_torque = 40
angle = 90 - 25.58
cmd_velocity = 0.0
cmd_steering = 0.0
max_cmd_velocity = 2 # 减小初始指令速度
max_cmd_steering = 1  # 减小初始转向指令
target_v = 0.0
target_s = 0.0
s_left = 0
s_right = 0
last_theta_ll = 0
last_theta_lr = 0

# 初始化电机位置
motor1.setAvailableTorque(max_torque)
motor1.setPosition(float(-angle / 180 * PI))
motor2.setAvailableTorque(max_torque)
motor2.setPosition(float(angle / 180 * PI))
motor3.setAvailableTorque(max_torque)
motor3.setPosition(float(-angle / 180 * PI))
motor4.setAvailableTorque(max_torque)
motor4.setPosition(float(angle / 180 * PI))

wheel_motor.setPosition(float('inf'))
wheel_motor1.setPosition(float('inf'))

# 主循环
while robot.step(timestep) != -1:
    # 处理键盘输入（瞬时响应）
    key = keyboard.getKey()
    
    # 重置指令（松开按键时归零）
    if key == -1:  # 没有按键按下
        cmd_velocity *= 0.8  # 缓慢归零
        cmd_steering *= 0.8
    else:
        if key == ord('W') or key == Keyboard.UP:
            cmd_velocity = max_cmd_velocity
        elif key == ord('S') or key == Keyboard.DOWN:
            cmd_velocity = -max_cmd_velocity
        if key == ord('A') or key == Keyboard.LEFT:
            cmd_steering = max_cmd_steering
        elif key == ord('D') or key == Keyboard.RIGHT:
            cmd_steering = -max_cmd_steering
    
    # 原有控制算法
    l_ll, theta_ll, phi_2l, phi_3l, phi_0l = CalculateTheta(-position_sensor1.getValue() + PI/2, -position_sensor2.getValue() + PI/2, imu.getRollPitchYaw()[1])
    delta_theta_ll = (theta_ll - last_theta_ll) / 32 * 1000
    last_theta_ll = theta_ll

    l_lr, theta_lr, phi_2r, phi_3r, phi_0r = CalculateTheta(-position_sensor3.getValue() + PI/2, -position_sensor4.getValue() + PI/2, imu.getRollPitchYaw()[1])
    delta_theta_lr = (theta_lr - last_theta_lr) / 32 * 1000
    last_theta_lr = theta_lr
    
    Leg_set = np.array([1, l_ll, l_ll*l_ll, l_lr, l_lr*l_lr, l_ll*l_lr])
    K_matrix = np.zeros((4,10))
    for i in range(6):
        K_matrix += const_set[:,:,i] * Leg_set[i]
    
    # 计算差速转向（作为额外扭矩添加）
    left_torque_adjust = cmd_velocity + cmd_steering
    right_torque_adjust = cmd_velocity - cmd_steering
    
    # 更新轮子扭矩（将键盘控制作为额外扭矩添加）
    base_left_torque = (K_matrix[0,0] * (s_left) + 
                       K_matrix[0,1] * (wheel_gyro.getValues()[2]*0.1) - 
                       K_matrix[0,4] * (theta_ll) - 
                       K_matrix[0,5] * delta_theta_ll - 
                       K_matrix[0,2] * imu.getRollPitchYaw()[2] - 
                       K_matrix[0,3] * gyro.getValues()[2] - 
                       K_matrix[0][6]*theta_lr - 
                       K_matrix[0][7]*delta_theta_lr - 
                       K_matrix[0][8]*imu.getRollPitchYaw()[1] - 
                       K_matrix[0][9]*gyro.getValues()[1])
    
    base_right_torque = (K_matrix[0,0] * (s_right) + 
                        K_matrix[0,1] * (wheel_gyro1.getValues()[2]*0.1) - 
                        K_matrix[0,4]* (theta_lr) - 
                        K_matrix[0,5] * delta_theta_lr + 
                        K_matrix[0,2] * imu.getRollPitchYaw()[2] + 
                        K_matrix[0,3] * gyro.getValues()[2] - 
                        K_matrix[0][6]*theta_ll - 
                        K_matrix[0][7]*delta_theta_ll - 
                        K_matrix[0][8]*imu.getRollPitchYaw()[1] - 
                        K_matrix[0][9]*gyro.getValues()[1])
    
    # 应用扭矩（基础扭矩+调整扭矩）
    wheel_motor.setTorque(base_left_torque + left_torque_adjust)
    wheel_motor1.setTorque(base_right_torque + right_torque_adjust)
    
    # 更新腿的位置
    s_left += (wheel_gyro.getValues()[2] * 0.1) * 0.032 
    s_right += (wheel_gyro1.getValues()[2] * 0.1) * 0.032