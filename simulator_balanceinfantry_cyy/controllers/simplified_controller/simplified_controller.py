"""my_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from controller import Motor

import numpy as np
import scipy.io as sio


PI = 3.1415926535

const_set = sio.loadmat("../../LQR_control/const_set(1206).mat")
const_set = const_set['const_set']




def CalculateTheta(theta1, theta4, pitch):
    xd = 0.15* np.cos(theta4) + 0.108 / 2
    yd = 0.15* np.sin(theta4)
    xb = 0.15* np.cos(theta1) - 0.108 / 2
    yb = 0.15* np.sin(theta1)
    A0 = 2 * 0.25 * (xd - xb)
    B0 = 2 * 0.25 * (yd - yb)
    lbd = np.sqrt((xb - xd) * (xb - xd) + (yb - yd) * (yb - yd))
    C0 = lbd * lbd
    theta2 = 2 * np.arctan2(B0 + np.sqrt(A0 * A0 + B0 * B0 - C0 * C0), A0 + C0)
    xc = 0.15 * np.cos(theta1) + 0.25 * np.cos(theta2) - 0.108 / 2
    yc = 0.15 * np.sin(theta1) + 0.25 * np.sin(theta2)
    length = np.sqrt(xc * xc + yc * yc)
    theta = np.pi / 2 - (np.arctan2(yc, xc) -  pitch)

    phi0 = np.arctan2(yc, xc) 
    phi4 = theta4

    theta3 = np.arctan2((length*np.sin(phi0) - 0.15*np.sin(phi4)),(length * np.cos(phi0) - 0.15*np.cos(phi4) - 0.054))    

    return length, theta, theta2, theta3, phi0



max_torque = 200


# data = np.array([
#     [-0.0157182502634262, -2.73190485580608, 0.410813750325312, 0.359295289968512, -15.6749006706595, -1.65439726626218, 2.23537205485116, 0.0285385083371951, -1.04391148574123, -0.112024483619615],
#     [-0.0157182502634270, -2.73190485580626, -0.410813750323929, -0.359295289968155, 2.23537205484926, 0.0285385083370085, -15.6749006706578, -1.65439726626204, -1.04391148574124, -0.112024483619618],
#     [-0.00121173608026206, -0.210548153270880, -1.55422521896685, -0.576367610223468, 5.51695775384208, 0.440381235414383, -4.80426769038583, -0.529603029480969, -4.89540636023820, -0.360187650069227],
#     [-0.00121173608026119, -0.210548153270688, 1.55422521896702, 0.576367610223495, -4.80426769038585, -0.529603029480960, 5.51695775384238, 0.440381235414421, -4.89540636023816, -0.360187650069223]
# ])
# K_matrix = np.array(data)
K_matrix = np.zeros((4,10))

# LQR_K1 = data[0][0]


# LQR_K2 = data[0][1]

# LQR_K3 = data[0][4]
# LQR_K4 = data[0][5]
# LQR_K5 = data[0][2]
# LQR_K6 = data[0][3]





t = 0

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(10)
motor1 = robot.getDevice('motor_1')
motor2 = robot.getDevice('motor_2')


position_sensor1 = motor1.getPositionSensor()
position_sensor2 = motor2.getPositionSensor()


position_sensor1.enable(timestep)
position_sensor2.enable(timestep)







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



max_speed = 4*PI
# angle = 25.58

# motor1.setPosition(0.0)
# motor2.setPosition(0.0)
# motor3.setPosition(0.0)
# motor4.setPosition(0.0)
motor1.setAvailableTorque(max_torque)
motor1.setPosition(float(0 / 180*3.1415926535))
# motor1.setVelocity(0.0)
motor2.setAvailableTorque(max_torque)
motor2.setPosition(float(0 / 180*3.1415926535))

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)
# wheel_motor.setAvailableTorque(3)
# wheel_motor1.setAvailableTorque(3)

wheel_motor.setPosition(float('inf'))
wheel_motor1.setPosition(float('inf'))
# Main loop:
# - perform simulation steps until Webots is stopping the controller
last_theta_ll = 0
last_theta_lr = 0

s_left = 0
s_right = 0
target_s = 0
while robot.step(timestep) != -1:
    # motor1.setPosition(float('-2.5'))
    # wheel_motor.setVelocity(5.0)
    # wheel_motor1.setVelocity(5.0)
    # print("Pitch:"+str(imu.getRollPitchYaw()[1]))
    # print(gyro.getValues()[1])
    # pass
    # print(wheel_gyro.getValues()[2])
    # wheel_motor.setTorque(-(LQR_K2 * (wheel_motor.getVelocity()*0.1-0.0)  - LQR_K3 * imu.getRollPitchYaw()[1] - LQR_K4 * gyro.getValues()[1] + LQR_K5 * imu.getRollPitchYaw()[2] + LQR_K6 * gyro.getValues()[2]))
    # wheel_motor1.setTorque(-(LQR_K2 * (wheel_motor.getVelocity()*0.1 -0.0) - LQR_K3 * imu.getRollPitchYaw()[1] - LQR_K4 * gyro.getValues()[1] + LQR_K5 * imu.getRollPitchYaw()[2] + LQR_K6 * gyro.getValues()[2]))
    


    # wheel_motor.setTorque(LQR_K2 * (wheel_gyro.getValues()[2]*0.1) -  LQR_K3 * theta_ll - LQR_K4 * delta_theta_ll + LQR_K5 * imu.getRollPitchYaw()[2] + LQR_K6 * gyro.getValues()[2] + K_matrix[0][6]*theta_lr + K_matrix[0][7]*delta_theta_lr)
    # wheel_motor1.setTorque(LQR_K2 * (wheel_gyro1.getValues()[2]*0.1) -  LQR_K3 * theta_lr - LQR_K4 * delta_theta_lr - LQR_K5 * imu.getRollPitchYaw()[2] - LQR_K6 * gyro.getValues()[2] + K_matrix[0][6]*theta_ll + K_matrix[0][7]*delta_theta_ll)
    # print((LQR_K2 * (wheel_motor.getVelocity()*0.1-0.0) + LQR_K3 * imu.getRollPitchYaw()[1] ))
    
    # l_ll,theta_ll,phi_2l,phi_3l,phi_0l = CalculateTheta(-position_sensor1.getValue() + PI/2, -position_sensor2.getValue()+ PI/2,imu.getRollPitchYaw()[1])
    # l_ll,theta_ll,phi_2l,phi_3l = CalculateTheta(position_sensor2.getValue() + PI/2, position_sensor1.getValue()+ PI/2,imu.getRollPitchYaw()[1])
    l_ll = 0.24
    theta_ll = position_sensor1.getValue() + imu.getRollPitchYaw()[1] 
    # print(theta_ll)
    delta_theta_ll = (theta_ll-last_theta_ll)/32*1000   ### Time step is 32ms
    last_theta_ll = theta_ll


    l_lr = 0.24
    theta_lr = position_sensor2.getValue() + imu.getRollPitchYaw()[1] 
    # print(theta_lr)
    delta_theta_lr = (theta_lr-last_theta_lr)/32*1000   ### Time step is 32ms
    last_theta_lr = theta_lr
#     l_lr,theta_lr,phi_2r,phi_3r,phi_0r = CalculateTheta(-position_sensor3.getValue() + PI/2, -position_sensor4.getValue()+ PI/2,imu.getRollPitchYaw()[1])
#     # l_lr,theta_lr,phi_2r,phi_3r = CalculateTheta(position_sensor4.getValue() + PI/2, position_sensor3.getValue()+ PI/2,imu.getRollPitchYaw()[1])

#     delta_theta_lr = (theta_lr-last_theta_lr)/32*1000   ### Time step is 32ms
#     last_theta_lr = theta_lr
    
    Leg_set = np.array([1,l_ll,l_ll*l_ll,l_lr,l_lr*l_lr,l_ll*l_lr])
    K_matrix = np.zeros((4,10))
    for i in range(6):
        K_matrix += const_set[:,:,i]*Leg_set[i]


#     ###########   Below is the Jacobian for VMC From SJTU  ##################

   
  

#     ###   Jacobian for VMC  ##################
   


# ##################   Below is the Jacobian for VMC From Master Xi  ##################


#     # Jacob_left = np.array([[(l_u*np.sin(phi_0l - phi_3l)*np.sin(-position_sensor1.getValue()+ PI/2-phi_2l)/np.sin(phi_3l-phi_2l)), (l_u*np.cos(phi_0l-phi_3l)*np.sin(-position_sensor1.getValue()+ PI/2-phi_2l)/l_ll/np.sin(phi_3l-phi_2l))],
#     #                        [l_u*np.sin(phi_0l- phi_2l)*np.sin(phi_3l-(PI/2-position_sensor2.getValue()))/np.sin(phi_3l-phi_2l), l_u*np.cos(phi_0l-phi_2l)*np.sin(phi_3l-(PI/2-position_sensor2.getValue()))/l_ll/np.sin(phi_3l-phi_2l)]])

#     # #jacob right
#     # Jacob_right = np.array([[(l_u*np.sin(phi_0r - phi_3r)*np.sin(-position_sensor3.getValue()+ PI/2-phi_2r)/np.sin(phi_3r-phi_2r)), (l_u*np.cos(phi_0r-phi_3r)*np.sin(-position_sensor3.getValue()+ PI/2-phi_2r)/l_lr/np.sin(phi_3r-phi_2r))],
#     #                         [l_u*np.sin(phi_0r- phi_2r)*np.sin(phi_3r-(PI/2-position_sensor4.getValue()))/np.sin(phi_3r-phi_2r), l_u*np.cos(phi_0r-phi_2r)*np.sin(phi_3r-(PI/2-position_sensor4.getValue()))/l_lr/np.sin(phi_3r-phi_2r)]])
   
   
#     F_left = 3.25*9.8 - ( l_ll - 0.22)*900

#     F_right = 3.25*9.8 - ( l_lr - 0.22)*900

    target_v = 1.0

    T_left = (K_matrix[2,0] * (s_left + target_s) + K_matrix[2][1] * (wheel_gyro.getValues()[2]*0.1 + target_v ) -  K_matrix[2][4] * (theta_ll ) - K_matrix[2][5] * delta_theta_ll + K_matrix[2][2] * imu.getRollPitchYaw()[2] + K_matrix[2][3] * gyro.getValues()[2] - K_matrix[2][6]*theta_lr - K_matrix[2][7]*delta_theta_lr - K_matrix[2][8]*imu.getRollPitchYaw()[1] - K_matrix[2][9]*gyro.getValues()[1] )
    T_right = (K_matrix[3,0] * (s_left + target_s) + K_matrix[3][1] * (wheel_gyro1.getValues()[2]*0.1 + target_v ) -  K_matrix[3][4] * (theta_ll ) - K_matrix[3][5] * delta_theta_ll + K_matrix[3][2] * imu.getRollPitchYaw()[2] + K_matrix[3][3] * gyro.getValues()[2] - K_matrix[3][6]*theta_lr - K_matrix[3][7]*delta_theta_lr - K_matrix[3][8]*imu.getRollPitchYaw()[1] - K_matrix[3][9]*gyro.getValues()[1] )

#     # T_left =  -(theta_ll - 0.0)*200
#     # T_right =  -(theta_lr - 0.0)*200


    motor1.setTorque(( T_left ))
    motor2.setTorque( T_right)

   
#     # motor1.setTorque(-( Jacob_left[0][0]*F_left ))
#     # motor2.setTorque(Jacob_left[1][0]*F_left)
 
#     # motor3.setTorque(-(Jacob_right[0][0]*F_right ))
#     # motor4.setTorque( Jacob_right[1][0]*F_right )
#     # # # motor1.setTorque(-5)
#     # motor2.setTorque(5)
    target_s += target_v * 0.032
    s_left  += (wheel_gyro.getValues()[2]*0.1 ) * 0.032 
    s_right += (wheel_gyro1.getValues()[2]*0.1 ) * 0.032
    # wheel_motor.setTorque(0)
    # wheel_motor1.setTorque(0)
    wheel_motor.setTorque( (K_matrix[0,0] * (s_left + target_s) + K_matrix[0,1] * (wheel_gyro.getValues()[2]*0.1 + target_v ) -  K_matrix[0,4] * (theta_ll ) - K_matrix[0,5] * delta_theta_ll + K_matrix[0,2] * imu.getRollPitchYaw()[2] + K_matrix[0,3] * gyro.getValues()[2] - K_matrix[0][6]*theta_lr - K_matrix[0][7]*delta_theta_lr - K_matrix[0][8]*imu.getRollPitchYaw()[1] - K_matrix[0][9]*gyro.getValues()[1] ))
    wheel_motor1.setTorque( (K_matrix[0,0] * (s_right + target_s) + K_matrix[0,1]  * (wheel_gyro1.getValues()[2]*0.1 + target_v ) -   K_matrix[0,4]* (theta_lr ) - K_matrix[0,5] * delta_theta_lr - K_matrix[0,2] * imu.getRollPitchYaw()[2] - K_matrix[0,3] * gyro.getValues()[2] - K_matrix[0][6]*theta_ll - K_matrix[0][7]*delta_theta_ll - K_matrix[0][8]*imu.getRollPitchYaw()[1] - K_matrix[0][9]*gyro.getValues()[1] ))
    
    print(wheel_gyro.getValues()[2]*0.1)
    # wheel_motor.setTorque(LQR_K2 * (wheel_gyro.getValues()[2]*0.1 )  - LQR_K3 * (theta_ll ) - LQR_K4 * delta_theta_ll - K_matrix[0][6]*theta_lr - K_matrix[0][7]*delta_theta_lr - K_matrix[0][8]*imu.getRollPitchYaw()[1] - K_matrix[0][9]*gyro.getValues()[1])
    # wheel_motor1.setTorque(LQR_K2 * (wheel_gyro.getValues()[2]*0.1 ) -  LQR_K3 * (theta_ll ) - LQR_K4 * delta_theta_ll - K_matrix[0][6]*theta_lr - K_matrix[0][7]*delta_theta_lr - K_matrix[0][8]*imu.getRollPitchYaw()[1] - K_matrix[0][9]*gyro.getValues()[1])
    
    # print("Jacob_left:  " + str(Jacob_left) + "  Jacob_right:  " + str(Jacob_right))

    # display.drawPixel(t, K_matrix[0][6]*theta_lr * 3.2 + 32)

    # print(( wheel_motor.getVelocity()))
# Enter here exit cleanup code.
