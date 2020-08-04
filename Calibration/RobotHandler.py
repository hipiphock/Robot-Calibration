# Deals with Robot initialization, movement,
# and reading/writing robot value
import numpy as np
import urx

# setting robot
def set_robot(cam_robot_addr, nocam_robot_addr):
	cam_robot = urx.Robot(cam_robot_addr)
	nocam_robot = urx.Robot(nocam_robot_addr)

	# cam_robot_home_joint_rad = np.deg2rad([67.83, -71.38, 130.59, -149.20, -90.08, 66.66])
	# nocam_robot_home_joint_rad = np.deg2rad([67.83, -71.38, 130.59, -149.20, -90.08, 66.66])
    cam_home_joint_rad = np.deg2rad([0.3209, -113.0970, -4.5383, -152.3580, 89.6613, 1.2152])  # : Cam Pose
    # nocam_home_joint_rad_b = np.deg2rad([40.9664, -74.1802, 117.9032, -112.9013, 247.8042, -224.6624 + 180])  # : Tray Pose - big
    nocam_home_joint_rad_s = [0.7150, -1.29469, 2.0578, -1.9705, 4.3250, -3.9211]  # : Tray Pose(rad) - small

	cam_robot.set_tcp([0, 0, 0.153, 0, 0, 0])
	nocam_robot.set_tcp([0, 0, 0.170, 0, 0, 0])

	cam_robot.movej(cam_robot_home_joint_rad, 0.5, 0.5)
    nocam_robot.movej(nocam_home_joint_rad_s, 0.5, 0.5)
    return cam_robot, nocam_robot