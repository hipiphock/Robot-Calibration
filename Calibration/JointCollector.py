"""
This module deals with joint collection needed for calibration.
"""
import os
import numpy as np
import cv2
import urx
import pyrealsense2 as rs

from urx.robotiq_two_finger_gripper import Robotiq_Two_Finger_Gripper
from Calibration.HSVfilter import read_hsv_filter, get_max_radius
import Calibration.CameraStream as CameraStream

# TODO: implement method
def save_joint_data():
	pass

def collect_joint(pipeline, cam_robot, hsv_filter_path, savepath):
	low_hsv, high_hsv = read_hsv_filter(hsv_filter_path)
	# robot
	# rob = urx.Robot("192.168.10.72")              # 오른쪽
	# rob = urx.Robot("192.168.10.77")              # 왼쪽
	rob = urx.Robot(cam_robot)  # 왼쪽
	rob.set_tcp([0, 0, 0.153, 0, 0, 0])
	home_joint_rad = np.deg2rad([55.62, -41.46, 75.42, -123.94, -89.92, 55.89])
	rob.movej(home_joint_rad, 0.5, 0.5)

	# main function part
	joint_file = savepath + "joint.txt"
	file = open(joint_file, "r")
	line_cnt = 0
	while True:
		is_free = False

		_, _, _, color_image = CameraStream.get_frames_and_images(pipeline)

		img = color_image

		# Normal masking algorithm
		lower_filter = low_hsv
		upper_filter = high_hsv

		hsv = np.copy(img)
		hsv = cv2.cvtColor(hsv, cv2.COLOR_RGB2HSV)
		mask = cv2.inRange(hsv, lower_filter, upper_filter)
		result = cv2.bitwise_and(img, img, mask=mask)
		# : ---- 위는 모두 마스킹

		ret, thresh = cv2.threshold(result, 16, 255, cv2.THRESH_BINARY)  # : 스레스홀드 127로 설정, 최대 255
		blurred = cv2.medianBlur(thresh, 5)  # : 메디안 필터를 이용한 블러
		blurred = cv2.cvtColor(blurred, cv2.COLOR_RGB2GRAY)  # : 그레이스케일로 변환
		# th3 = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2) # : 가우시안, 어뎁티브 스레스홀드
		_, th3 = cv2.threshold(blurred, 10, 255, cv2.THRESH_BINARY)

		_, contours, hierarchy = cv2.findContours(th3, 1, 2)  # :
		cv2.drawContours(result, contours, -1, (0, 255, 0), 1)

		max_radius, cx, cy = get_max_radius(contours)

		fontScale = 1
		color = (0, 0, 255)  # : BGR
		location = (0, 50)
		font = cv2.FONT_ITALIC
		try:
			cx, cy = int(cx), int(cy)
			if 0 < max_radius <= 3:
				print("-->>hsv : put it closer!")
				cv2.rectangle(result, (0, 0), 1280, 720, (0, 0, 255), 2)  # draw circle in red color
			elif max_radius is 0:
				text = "-->>hsv : NoBall yeah. put it ON!"
				cv2.putText(result, text, location, font, fontScale, color)
				cv2.rectangle(result, (0, 0), 1280, 720, (0, 0, 255), 2)  # draw circle in red color
			else:
				cv2.circle(result, (int(cx), int(cy)), int(max_radius), (0, 0, 255), 2)  # draw circle in red color
		except:
			print("-->>hsv : Can't Find the ball")

		cv2.moveWindow('Color filter', 2560 - int(1280 / 2) - 1, 320)
		cv2.imshow('Color filter', cv2.resize(result, (int(1280 / 2), int(720 / 2))))
		cv2.moveWindow('origin', 2560 - int(1280 / 2) - 1, 680)
		cv2.imshow('origin', cv2.resize(img, (int(1280 / 2), int(720 / 2))))

		key = cv2.waitKey(33)

		if key == ord('f'):  # : 프리드라이브
			rob.set_freedrive(not is_free, 3600)  # 3600 sec.
			is_free = not is_free
			if is_free:
				print(" Free Drive Mode : ON !!")
			else:
				print(" Free Drive Mode : OFF !!")

		if key == 115:  # : S   # : 저장
			if 6 < max_radius:
				curj = np.round(rob.getj(), 4)
				f.write("{} {} {} {} {} {}\n".format(*curj))
				print("Saved joint data . {}".format(line_cnt))
				line_cnt += 1
			else:
				print("Not Saved joint data in {} ---- ---- ".format(line_cnt))

		if key == 113:  # : Q   # : 종료
			print("Saved all.")
			cv2.destroyAllWindows()
			break

	file.close()
	print("End collect  Total : {}".format(line_cnt - 1))
	exit()

#####################

# : 2 : 각 로봇의 포즈 저장
# : 다음 3 : main 실행