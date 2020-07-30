import pyrealsense2 as rs

import cv2
import urx
import numpy as np
from Calibration.HSVfilter import read_hsv_filter
from Calibration.collecting_position import DataRecord

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

# Start streaming
pipe_profile = pipeline.start(config)

# : 3 : 포즈를 기반으로 bin파일 생성
# : 다음 4 : Kinect3DCalib.cpp 프로젝트 실행
def get_cam_img():
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()

    align = rs.align(rs.stream.color)
    frameset = align.process(frames)

    aligned_depth_frame = frameset.get_depth_frame()
    # depth_frame = frames.get_depth_frame()

    # Intrinsics & Extrinsics
    depth_intrin = aligned_depth_frame.profile.as_video_stream_profile().intrinsics
    # depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
    color_intrin = color_frame.profile.as_video_stream_profile().intrinsics
    # color_intrin = color_frame.profile.as_video_stream_profile().intrinsics
    depth_to_color_extrin = aligned_depth_frame.profile.get_extrinsics_to(color_frame.profile)
    # depth_to_color_extrin = depth_frame.profile.get_extrinsics_to(color_frame.profile)

    # Convert images to numpy arrays
    # depth_image = np.asanyarray(depth_frame.get_data())
    depth_image = np.asanyarray(aligned_depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())

    return aligned_depth_frame, color_frame, depth_intrin, color_intrin, depth_image, color_image
    # return depth_frame, color_frame, depth_intrin, color_intrin, depth_image, color_image

def init_cam(fp, low_hsv, high_hsv):
    print("-->>sys : initializing Realsense ......")
    for num in range(0, fp):
        _, _, _, _, _, color_image = get_cam_img()
        test_view = np.copy(color_image)

        hsv = cv2.cvtColor(test_view, cv2.COLOR_RGB2HSV)
        lower_blue = low_hsv
        upper_blue = high_hsv
        mask = cv2.inRange(hsv, lower_blue, upper_blue)


        result = cv2.bitwise_and(test_view, color_image, mask=mask)

        ret, thresh = cv2.threshold(result, 127, 255, cv2.THRESH_BINARY)
        blurred0 = cv2.medianBlur(thresh, 5)
        blurred = cv2.cvtColor(blurred0, cv2.COLOR_RGB2GRAY)
        # th3 = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)
        _, th3 = cv2.threshold(blurred, 10, 255, cv2.THRESH_BINARY)

        cv2.moveWindow('threshold', 2560 - int(1280 / 2) - 1, 0)
        cv2.imshow('threshold', cv2.resize(th3, (int(1280 / 2), int(720 / 2))))
        cv2.moveWindow('color_image', 2560 - int(1280 / 2) - 1, 390)
        cv2.imshow('color_image', cv2.resize(color_image, (int(1280 / 2), int(720 / 2))))
        cv2.waitKey(2)

    print("-->>sys : Realsense initializing completed.")


def create_binary(pipeline, hsv_filter_path, joint_path):
    rob = urx.Robot("192.168.10.55")
    home_joint_rad = np.deg2rad([55.62, -41.46, 75.42, -123.94, -89.92, 55.89])
    rob.movej(home_joint_rad, 0.5, 0.5)

    low_hsv, high_hsv = read_hsv_filter(hsv_filter_path)

    print("-->>sys : Starting Calibration datacollect ......")
    init_cam(120)

    dataRecord = DataRecord()
    print("-->>sys : move robot to HOME position ......")
    rob.movej(home_joint_rad, 2, 2)

    # open binary datafile
    binary_path = ""
    binary_file = open(binary_path, "r")
    init = 0
    for _ in range(init):
        binary_file.readline()

    for idx, line in enumerate(binary_file):
        try:
            k = line.split(' ')
            j_pt = [float(j) for j in k]
            rob.movej(j_pt, 0.8, 0.8)

            cv2.waitKey(100)

            endEffector_pos = [round(x, 4) for x in rob.getl()[0:3]]

            depth_frame, _, depth_intrin, _, depth_image, color_image = get_cam_img()
            g_view = np.copy(color_image)

            # Filter
            hsv = cv2.cvtColor(g_view, cv2.COLOR_RGB2HSV)
            lower_blue = low_hsv
            upper_blue = np.array([255, 255, 255])  # FIX
            mask = cv2.inRange(hsv, lower_blue, upper_blue)
            result = cv2.bitwise_and(g_view, color_image, mask=mask)

            ret, thresh = cv2.threshold(result, 16, 255, cv2.THRESH_BINARY)
            blurred0 = cv2.medianBlur(thresh, 5)
            blurred = cv2.cvtColor(blurred0, cv2.COLOR_RGB2GRAY)
            _, th3 = cv2.threshold(blurred, 10, 255, cv2.THRESH_BINARY)

            _, contours, hierarchy = cv2.findContours(th3, 1, 2)  # :
            cv2.drawContours(color_image, contours, -1, (0, 255, 0), 2)

            cv2.moveWindow('threshold', 2560 - int(1280 / 2) - 1, 0)
            cv2.imshow('threshold', cv2.resize(th3, (int(1280 / 2), int(720 / 2))))
            cv2.moveWindow('color_image', 2560 - int(1280 / 2) - 1, 390)
            cv2.imshow('color_image', cv2.resize(color_image, (int(1280 / 2), int(720 / 2))))
            cv2.waitKey(2)

            index = np.argwhere(np.squeeze(th3) == 255)
            h = index[:, 0]
            w = index[:, 1]
            ###########
            c_x, c_y = int(np.mean(w)), int(np.mean(h))

            depth0 = depth_frame.get_distance(c_x, c_y)
            depth0_ = depth_image[c_y, c_x]

            depth_point_0 = rs.rs2_deproject_pixel_to_point(depth_intrin, [c_x, c_y], depth0)

            camera_x, camera_y, camera_z = [], [], []
            for i in range(c_y - 3, c_y + 3 + 1):
                for j in range(c_x - 3, c_x + 3 + 1):
                    depth_1 = depth_frame.get_distance(j, i)
                    depth_point_1 = rs.rs2_deproject_pixel_to_point(depth_intrin, [j, i], depth_1)
                    camera_x.append(depth_point_1[0])
                    camera_y.append(depth_point_1[1])
                    camera_z.append(depth_point_1[2])

            camera_x_avg1 = sum(camera_x) / camera_x.__len__()
            camera_y_avg1 = sum(camera_y) / camera_y.__len__()
            camera_z_avg1 = sum(camera_z) / camera_z.__len__()

            sorted_y = camera_y.copy()
            sorted_y.sort()
            sorted_y_array = np.array(sorted_y)

            camera_x2, camera_y2, camera_z2 = [], [], []
            for k in range(c_y - 3, c_y + 3 + 1):
                for l in range(c_x - 3, c_x + 3 + 1):
                    depth_2 = depth_frame.get_distance(l, k)
                    if depth_2 == 0:
                        continue
                    depth_point_2 = rs.rs2_deproject_pixel_to_point(depth_intrin, [l, k], depth_2)
                    if ((camera_y_avg1 - abs(camera_y_avg1 / 10)) < depth_point_2[1] < (
                            camera_y_avg1 + abs(camera_y_avg1 / 10))) and \
                            ((camera_x_avg1 - abs(camera_x_avg1 / 10)) < depth_point_2[0] < (
                                    camera_x_avg1 + abs(camera_x_avg1 / 10))) and \
                            ((camera_z_avg1 - abs(camera_z_avg1 / 10)) < depth_point_2[2] < (
                                    camera_z_avg1 + abs(camera_z_avg1 / 10))):
                        camera_x2.append(depth_point_2[0])
                        camera_y2.append(depth_point_2[1])
                        camera_z2.append(depth_point_2[2])

            camera_x_avg2 = sum(camera_x2) / camera_x2.__len__()
            camera_y_avg2 = sum(camera_y2) / camera_y2.__len__()
            camera_z_avg2 = sum(camera_z2) / camera_z2.__len__()

            DataRecord.DataInsert([endEffector_pos[0], endEffector_pos[1], endEffector_pos[2]],
                                  [camera_x_avg2, camera_y_avg2, camera_z_avg2])  # -#-

            print("----e10")

            print("Current value: ", idx)

            if idx == 800:
                break
        except:
            print('frame passed {}'.format(idx))
            continue
    rob.movej(home_joint_rad, 2, 2)
    DataRecord.CloseData()

    # 정상적으로 저장 완료 된 상태.
    print('finish')
    rob.set_tcp([0, 0, 0, 0, 0, 0])
    rob.close()
