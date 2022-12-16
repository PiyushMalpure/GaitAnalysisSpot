#!/usr/bin/env python3

import rospy

from sensor_msgs.msg import Joy,Imu
from RobotController import RobotController
from InverseKinematics import robot_IK
from std_msgs.msg import Float64

USE_IMU = True
RATE = 60

rospy.init_node("Robot_Controller")

# Robot geometry
body = [0.1908, 0.080]
legs = [0.0, 0.04, 0.100, 0.094333] 

spot_robot = RobotController.Robot(body, legs, USE_IMU)
inverseKinematics = robot_IK.InverseKinematics(body, legs)

command_topics = ["/spot_controller/FR1_joint/command",
                  "/spot_controller/FR2_joint/command",
                  "/spot_controller/FR3_joint/command",
                  "/spot_controller/FL1_joint/command",
                  "/spot_controller/FL2_joint/command",
                  "/spot_controller/FL3_joint/command",
                  "/spot_controller/RR1_joint/command",
                  "/spot_controller/RR2_joint/command",
                  "/spot_controller/RR3_joint/command",
                  "/spot_controller/RL1_joint/command",
                  "/spot_controller/RL2_joint/command",
                  "/spot_controller/RL3_joint/command"]

publishers = []
for i in range(len(command_topics)):
    publishers.append(rospy.Publisher(command_topics[i], Float64, queue_size = 0))

if USE_IMU:
    rospy.Subscriber("spot_imu/base_link_orientation",Imu,spot_robot.imu_orientation)
rospy.Subscriber("spot_joy/joy_ramped",Joy,spot_robot.joystick_command)

rate = rospy.Rate(RATE)

del body
del legs
del command_topics
del USE_IMU
del RATE

while not rospy.is_shutdown():
    leg_positions = spot_robot.run()
    spot_robot.change_controller()

    dx = spot_robot.state.body_local_position[0]
    dy = spot_robot.state.body_local_position[1]
    dz = spot_robot.state.body_local_position[2]
    
    roll = spot_robot.state.body_local_orientation[0]
    pitch = spot_robot.state.body_local_orientation[1]
    yaw = spot_robot.state.body_local_orientation[2]

    try:
        joint_angles = inverseKinematics.inverse_kinematics(leg_positions,
                               dx, dy, dz, roll, pitch, yaw)

        for i in range(len(joint_angles)):
            publishers[i].publish(joint_angles[i])
    except:
        pass

    rate.sleep()
