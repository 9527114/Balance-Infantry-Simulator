"""my_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from controller import Motor
from controller import Keyboard
from controller import Camera
import numpy as np

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
    theta = np.pi / 2 - (np.arctan2(yc, xc) - pitch)
    return length, theta




## parameter setting initialize
PI = 3.1415926535 
r = 0
t = 0
T = 0
LQR_K1 = -0.0224
LQR_K2 = -2.2572
LQR_K3 = -12.8597
LQR_K4 = -3.0695
LQR_K5 = 2.2361
LQR_K6 = 0.6389

# raw data
data = [
    [-0.0111734475018034, -1.12848937711373, 0.642911297879309, 0.201408686321038, -9.07828015021314, -0.997667443909873, -2.50243089788043, -0.303694712229440, 0.982817718403077, 0.0998536832918112],
    [-0.0111734475018022, -1.12848937711364, -0.642911297879817, -0.201408686321114, -2.50243089788054, -0.303694712229442, -9.07828015021289, -0.997667443909842, 0.982817718403046, 0.0998536832918095],
    [-0.000785037386657426, -0.0791799898099167, -1.82938794470585, -0.892222475717143, 12.9295769194268, 1.25431455126950, -12.0756309572312, -1.33374743902778, -7.91341714337549, -0.456790018608269],
    [-0.000785037386661259, -0.0791799898101114, 1.82938794470614, 0.892222475717239, -12.0756309572322, -1.33374743902789, 12.9295769194271, 1.25431455126951, -7.91341714337557, -0.456790018608277]
]

# input = np.array([0,
#                      wheel_gyro.getValues()[2]*0.1,
#                      imu.getRollPitchYaw()[2],
#                      gyro.getValues()[2],
#                      theta_ll,
#                      theta_ll-last_theta_ll,
#                      theta_lr,
#                      theta_lr-last_theta_lr,
#                      imu.getRollPitchYaw()[1],
#                      imu.getRollPitchYaw()[1]-last_pitch])


#   convert to np array
K_matrix = np.array(data)

lw = 0
rw = 0
lt = 0
rt = 0
key = 0
Hflag = 0           ## Ment for standing up, can use for relax mode
Jflag = 0           ## Trial flag of Jumping
max_speed = 6.28
max_torque = 40.0
max_wtorque = 200.5
angle = 25.58       ## 25.58/-0.45 for Test output data, -1.8 for Floor situation.
Langle = 0    
Rangle = 0
Fangle = 0
Bangle = 0
timestep = int(10)  ## get the time step of the current world.
timedelay = 0

## create the Robot instance.
robot = Robot()

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


display = robot.getDevice('display')
display.setOpacity(1)

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

# print(wheel_motor)
# print(wheel_motor1)

## get keyboard input - Rededge 1117
keyboard = Keyboard()
keyboard.enable(timestep)


## get camera vision - Rededge 1117
    ## camera parameters setted inside webot -Rededge 1117
camera = Camera("cam1")
camera = Camera.enable(camera,1)


##Leg joint motor set
motor1.setPosition(float(-Langle / 180*PI))
motor1.setAvailableTorque(float(max_torque))
#motor1.setVelocity(max_speed)
motor2.setPosition(float(Langle / 180*PI))
motor2.setAvailableTorque(float(max_torque))
#motor2.setVelocity(max_speed)
motor3.setPosition(float(-Rangle / 180*PI))
motor3.setAvailableTorque(float(max_torque))
#motor3.setVelocity(max_speed)
motor4.setPosition(float(Rangle / 180*PI))
motor4.setAvailableTorque(float(max_torque))
#motor4.setVelocity(max_speed)
# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)
# wheel_motor.setPosition(float('inf'))
# wheel_motor1.setPosition(float('inf'))
# Main loop:
# - perform simulation steps until Webots is stopping the controller
wheel_motor.setAvailableTorque(float(max_wtorque))
wheel_motor1.setAvailableTorque(float(max_wtorque))
wheel_motor.enableTorqueFeedback(timestep)
wheel_motor1.enableTorqueFeedback(timestep)
motor1.enableTorqueFeedback(timestep)
motor2.enableTorqueFeedback(timestep)
motor3.enableTorqueFeedback(timestep)
motor4.enableTorqueFeedback(timestep)



while robot.step(timestep) != -1:
    # motor1.setPosition(float('-2.5'))
    # wheel_motor.setVelocity(5.0)
    # wheel_motor1.setVelocity(5.0)
    # print("Pitch:"+str(imu.getRollPitchYaw()[1]))
    # print(gyro.getValues()[1])
    # pass
    ## parameters calculating area - Rededge 1117
    #     # Trial Turn offset
    # Langle = 5*lt
    # Rangle = 5*rt
        # Trial VMC ctrl
    Fangle = 15*lw
    Bangle = 15*rw
    
    lw = 0
    rw = 0
    lt = 0
    rt = 0
    T += 1
    if(timedelay>=1):
        timedelay -= 1
    #else:
        #timedelay = 0

        ## recursively get key input to have multiple input of key pressed - Rededge 1117
    key = Keyboard.getKey(3)
    while key != -1:

        if(key == Keyboard.CONTROL+ord('A')):
            lt = 1
            #print("A")
        elif(key == Keyboard.CONTROL+ord('D')):
            rt = 1
            #print("D")

        if(key == Keyboard.CONTROL+ord('W')):
            lw = 1 + lt
            rw = 1 + rt
            #print("W")
        elif(key == Keyboard.CONTROL+ord('S')):
            lw = -1 - lt
            rw = -1 - rt
            #print("S")
        
        if(timedelay == 0):
            if(key == Keyboard.CONTROL+ord('X')):
                if(Hflag == 0):
                    Hflag = 1
                    #print("L1")
                elif(Hflag == 1):
                    Hflag = 0
                    #print("L2")
                timedelay=20
        #if(Hflag == 1):
        #    if(key == Keyboard.CONTROL+ord('L')):
        #        if(Jflag == 0):
        #            Jflag = 1
        #            angle = 25.58
        #            #print("L1")
        #       elif(Jflag == 1):
        #            Jflag = 0
        #            angle = 25.58*4
        #            #print("L2")
        #        timedelay=20  
        key = Keyboard.getKey(1)


    ## Parameter setting area -Rededge 1117
    motor1.setPosition(float((-angle-Langle+Fangle) / 180*PI))
    # motor1.setVelocity(0.0)
    motor2.setPosition(float((angle+Langle+Bangle) / 180*PI))
    # motor2.setVelocity(0.0)
    motor3.setPosition(float((-angle-Rangle+Bangle) / 180*PI))
    # motor3.setVelocity(0.0)
    motor4.setPosition(float((angle+Rangle+Fangle) / 180*PI))
    # motor4.setVelocity(0.0)
    
    if(Hflag == 1):
        angle = 25.58*4
        #print("L1")
    elif(Hflag == 0):
        angle = 25.58
        #print("L2")

    # wheel_motor.setTorque(-(LQR_K2 * (wheel_motor.getVelocity()*0.1-0.0)  - LQR_K3 * imu.getRollPitchYaw()[1] - LQR_K4 * gyro.getValues()[1] + LQR_K5 * imu.getRollPitchYaw()[2] + LQR_K6 * gyro.getValues()[2]))
    # wheel_motor1.setTorque(-(LQR_K2 * (wheel_motor.getVelocity()*0.1 -0.0) - LQR_K3 * imu.getRollPitchYaw()[1] - LQR_K4 * gyro.getValues()[1] + LQR_K5 * imu.getRollPitchYaw()[2] + LQR_K6 * gyro.getValues()[2]))
    
    ## Former LQR formula - Rededge 1117
    #wheel_motor.setTorque(LQR_K2 * (wheel_gyro.getValues()[2]*0.1+rw) -  LQR_K3 * imu.getRollPitchYaw()[1] - LQR_K4 * gyro.getValues()[1] + LQR_K5 * imu.getRollPitchYaw()[2] + LQR_K6 * gyro.getValues()[2])
    #wheel_motor1.setTorque(LQR_K2 * (wheel_gyro1.getValues()[2]*0.1+lw) -  LQR_K3 * imu.getRollPitchYaw()[1] - LQR_K4 * gyro.getValues()[1] + LQR_K5 * imu.getRollPitchYaw()[2] - LQR_K6 * gyro.getValues()[2])
    
    ## Canceled LQR_K5 to enable keyboard controled turn - Rededge 1117
    l_l,theta_ll = CalculateTheta(position_sensor2.getValue() + PI/2, position_sensor1.getValue() + PI/2, imu.getRollPitchYaw()[1])
    # print(position_sensor2.getValue())
    # print(l_l,theta_ll)
    # theta_ll = -theta_ll


    l_r,theta_lr = CalculateTheta(position_sensor4.getValue() + PI/2 , position_sensor3.getValue() + PI/2, imu.getRollPitchYaw()[1])
    # theta_lr = -theta_lr

    last_theta_ll = theta_ll

    last_theta_lr = theta_lr

    # input = np.array([0,
    #                  wheel_gyro.getValues()[2]*0.1,
    #                  imu.getRollPitchYaw()[2],
    #                  gyro.getValues()[2],
    #                  theta_ll,
    #                  theta_ll-last_theta_ll,
    #                  theta_lr,
    #                  theta_lr-last_theta_lr,
    #                  imu.getRollPitchYaw()[1],
    #                  imu.getRollPitchYaw()[1]-last_pitch])

    input = np.array([0,
                     wheel_gyro.getValues()[2]*0.1,
                     0,
                     0,
                     0,
                     0,
                     0,
                     0,
                     0,
                     0])
                     
    
    
    output = np.dot(K_matrix,input)
    print(output)
    
    
    
    if(Hflag == 0):
        wheel_motor.setTorque(output[0])
        wheel_motor1.setTorque(output[1])
    elif(Hflag == 1):
        wheel_motor.setTorque(0)
        wheel_motor1.setTorque(0)
    # print((LQR_K2 * (wheel_motor.getVelocity()*0.1-0.0) + LQR_K3 * imu.getRollPitchYaw()[1] ))
    
    #t = t + 0.5
    
    ## Console Feedback zone - Rededge 1117
        ## delayed feedback
    if r>=5:
        # print('Steptime = ',T)
        # print('Pitch =',imu.getRollPitchYaw()[1])
        # print('DeltaYaw =',gyro.getValues()[2])
        # print('LFMotorAngle = ',motor1.getTargetPosition())
        # print('RFMotorAngle = ',motor4.getTargetPosition())
        # print('WheelMotorTor = ',wheel_motor.getTorqueFeedback())
        r = 0
    else:
        r += 1
    
    # if (abs(gyro.getValues()[2]) <= 2.1):                               ## display yaw speed alarm
    #     display.setColor(0xFFFFFF)
    # else:
    #     display.setColor(0xFF0000)
    #     print("\r\033[31mOver max rotate speed! At ",gyro.getValues()[2]," currently.\033[0m") 

        ## display refresh - Rededge 1117
    if (t <= display.getWidth()):
        display.setColor(0xFFFFFF)
        display.fillRectangle(t+1,display.getHeight()/4,1,display.getHeight()/2)
        display.setColor(0x000000)
        display.fillRectangle(t,0,1,display.getHeight())
        
        display.setColor(0xFFFFFF) ## White Top                                       ## Remember to initiate first at the front
        display.drawPixel(t,display.getHeight()/4 - wheel_motor.getTorqueFeedback() * 10)  ## Left wheel Torque
        display.setColor(0xFF6F30) ## Orange Top
        display.drawPixel(t,display.getHeight()/4 - wheel_motor1.getTorqueFeedback() * 10) ## Right wheel Torque

        display.setColor(0xFFFF00) ## Yellow Middle
        display.drawPixel(t,display.getHeight()/2 - imu.getRollPitchYaw()[1] * 30)    ## Robot Pitch Feedback

        display.setColor(0xFF00FF) ## Purple Bottom
        display.drawPixel(t,display.getHeight()*3/4 - motor1.getTorqueFeedback()*2)     ## Robot LeftFront Joint Motor Torque
        display.setColor(0xFF002F) ## Red Bottom
        display.drawPixel(t,display.getHeight()*3/4 - motor4.getTorqueFeedback()*2)     ## Robot RightFront Joint Motor Torque
        display.setColor(0xD45057) ##  Bottom
        display.drawPixel(t,display.getHeight()*3/4 - motor2.getTorqueFeedback()*2)     ## Robot LeftBack Joint Motor Torque
        display.setColor(0x25F057) ##  Bottom
        display.drawPixel(t,display.getHeight()*3/4 - motor3.getTorqueFeedback()*2)     ## Robot RightBack Joint Motor Torque

        t += 1
    else:
        t = 0

        

# Enter here exit cleanup code.
