import time
import cv2
import numpy as np
from Calibration.Helper import nothing, create_directory

def get_max_radius(contours):
    max_radius = 0
    cx, cy = None, None
    for cnt in contours:
        if cv2.contourArea(cnt) < 60000:
            (cx, cy), radius = cv2.minEnclosingCircle(cnt)
            if radius > max_radius:
                max_radius = radius
    return max_radius, cx, cy

def save_image(image, dir_name):
    time_str = time.strftime('%Y%m%d-%H-%M-%S', time.localtime(time.time()))
    cv2.imwrite("./{}/img_{}.png".format(dir_name, time_str), image)
    print("-->>sys :  img_saved ")

# saves filter
def exit_program(savepath, low_h, low_s, low_v, high_h, high_s, high_v):
    print("-->>sys :  Exit program ")
    file_name = "hsv_{}.txt".format(savepath)
    file_path = os.path.join(savepath + "/", file_name)
    f = open(file_path, "a")
    f.write("{}".format(savepath, time.strftime('%Y%m%d-%H-%M-%S', time.localtime(time.time()))))
    f.write("\n-->>sys :  low__Result ~ H:{0:3d}, S:{1:3d}, V:{2:3d}".format(low_h, low_s, low_v))
    f.write("\n-->>sys :  high_Result ~ H:{0:3d}, S:{1:3d}, V:{2:3d}".format(high_h, high_s, high_v))
    f.close()
    cv2.destroyAllWindows()

# TODO: 죄다 고쳐야함
def read_hsv_filter(hsv_filter_path):
    f_hsv = open(hsv_filter_path, "r")
    line = []
    i = 0
    while True:
        text = f_hsv.readline()
        data_line = text.strip('\n')
        # data_line = text.splitlines()
        if text.__len__() == 0:
            break
        else:
            line.append(data_line)
            print("contents : " + data_line)
            i += 1

    l_h = int(line[-2][-17:-14])
    l_s = int(line[-2][-10:-7])
    l_v = int(line[-2][-3:])

    h_h = int(line[-1][-17:-14])
    h_s = int(line[-1][-10:-7])
    h_v = int(line[-1][-3:])
    low_hsv, high_hsv = [], []
    return low_hsv, high_hsv

# get image and convert color image into numpy array
def save_hsv_filter(pipeline, savepath):
    """
    main image handler method for calibration
    """
    cv2.namedWindow('result')
    cv2.namedWindow('th3')
    cv2.namedWindow('bar')
    cv2.resizeWindow('bar', 640, 320)

    # : opencv 함수 트랙바 생성 (트랙바의 이름, 띄울 창, 0~n, 변화시)
    cv2.createTrackbar('low_h', 'bar', 0, 180, nothing)
    cv2.createTrackbar('low_s', 'bar', 0, 255, nothing)
    cv2.createTrackbar('low_v', 'bar', 0, 255, nothing)

    cv2.createTrackbar('high_h', 'bar', 0, 180, nothing)
    cv2.createTrackbar('high_s', 'bar', 0, 255, nothing)
    cv2.createTrackbar('high_v', 'bar', 0, 255, nothing)
    cv2.setTrackbarPos('high_h', 'bar', 180)
    cv2.setTrackbarPos('high_s', 'bar', 255)
    cv2.setTrackbarPos('high_v', 'bar', 255)

    create_directory(savepath)

    put_it_on_flag = 0

    # : RL robot
    rob1 = urx.Robot("192.168.0.52")  # 오른쪽
    rob2 = urx.Robot("192.168.0.51")
    rob1_home_joint_rad = np.deg2rad([67.83, -71.38, 130.59, -149.20, -90.08, 66.66])
    rob2_home_joint_rad = np.deg2rad([67.83, -71.38, 130.59, -149.20, -90.08, 66.66])

    rob1.set_tcp([0, 0, 0.153, 0, 0, 0])
    rob2.set_tcp([0, 0, 0.170, 0, 0, 0])

    rob.movej(home_joint_rad, 0.5, 0.5)

    # rob.movel([-0.060355069913524816,
    #  -0.4409053222811345,
    #  -0.022449952179487417,
    #  -2.1925502393945053,
    #  -2.2480136500524344,
    #  0.001303274362250708], 0.5, 0.5)

    while True:  # : 프로그램이 돌아가는 영역 - 반복
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        # Intrinsics & Extrinsics
        depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
        color_intrin = color_frame.profile.as_video_stream_profile().intrinsics
        depth_to_color_extrin = depth_frame.profile.get_extrinsics_to(color_frame.profile)

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        img = color_image
        # img = cv2.resize(img, (int(1280 / 2), int(720 / 2)))  # : 이미지 변형
        hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)  # : 이미지를 RGB에서 HSV로 변환후 저장

        # get info from track bar and appy to result    # : 트랙바의 현재 상태를 받아 저장 (해당 트랙바 이름, 뜨운 창)
        low_h = cv2.getTrackbarPos('low_h', 'bar')
        low_s = cv2.getTrackbarPos('low_s', 'bar')
        low_v = cv2.getTrackbarPos('low_v', 'bar')

        high_h = cv2.getTrackbarPos('high_h', 'bar')
        high_s = cv2.getTrackbarPos('high_s', 'bar')
        high_v = cv2.getTrackbarPos('high_v', 'bar')

        # Normal masking algorithm
        lower_color = np.array([low_h, low_s, low_v])  # : 각  h,s,v를 저장하는 배열생성
        upper_color = np.array([high_h, high_s, high_v])  # : 각 최대 값

        # : 스레스홀드를 lower_color로 지정하여, 이하는 0값을 출력, 범위안의 것은 255를 출력하여 마스크를 생성
        mask = cv2.inRange(hsv, lower_color, upper_color)

        # : 마스크를 씌운 이미지와 마스크를 씌우지 않은 이미지에서 모두 0이 아닌경우에만 출력
        result = cv2.bitwise_and(img, img, mask=mask)

        ret, thresh = cv2.threshold(result, 16, 255, cv2.THRESH_BINARY)  # : 스레스홀드 127로 설정, 최대 255
        blurred = cv2.medianBlur(thresh, 5)  # : 메디안 필터를 이용한 블러
        blurred = cv2.cvtColor(blurred, cv2.COLOR_RGB2GRAY)  # : 그레이스케일로 변환
        # th3 = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11,
        #                             2)  # : 가우시안, 어뎁티브 스레스홀드
        _, th3 = cv2.threshold(blurred, 10, 255, cv2.THRESH_BINARY)

        # _, contours, hierarchy = cv2.findContours(th3.copy(), 1, 2)  # :
        # _, contours, hierarchy = cv2.findContours(th3, mode=cv2.RETR_EXTERNAL, method=cv2.CHAIN_APPROX_SIMPLE)  # : opencv 3
        contours, hierarchy = cv2.findContours(th3, mode=cv2.RETR_EXTERNAL,
                                               method=cv2.CHAIN_APPROX_SIMPLE)  # : opencv 4
        cv2.drawContours(result, contours, -1, (0, 255, 0), 2)

        # get maximum radius of enclosing circle
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
                put_it_on_flag = 0
        except:
            put_it_on_flag += 1
            if 1 <= put_it_on_flag:
                if put_it_on_flag == 1:
                    print("-->>hsv : Can't Find the ball")

            else:
                pass

            text = "-->>hsv : Can't Find the ball"
            thickness = 1
            color = (0, 0, 255)  # : BGR
            location = (0, 100)
            font = cv2.FONT_ITALIC
            cv2.rectangle(result, (0, 0), (1280, 720), (0, 0, 255), 2)  # draw circle in red color
            cv2.putText(result, text, location, font, thickness, color)

        cv2.moveWindow('bar', 2560 - int(1280 / 2) - 1, 0)
        cv2.moveWindow('result', 2560 - int(1280 / 2) - 1, 320)
        cv2.moveWindow('th3', 2560 - int(1280 / 2) - 1, 680)

        img_ = copy.deepcopy(img)
        th_list = np.argwhere(th3 == 255)
        img_[th_list[:, 0], th_list[:, 1], :] = 255
        result_ = cv2.resize(result, (int(1280 / 2), int(720 / 2)))
        # th3_ = cv2.resize(th3, (int(1280 / 2), int(720 / 2)))
        img_ = cv2.resize(img_, (int(1280 / 2), int(720 / 2)))

        cv2.imshow('result', result_)
        # cv2.imshow('th3', th3_)
        cv2.imshow('th3', img_)

        # _, contours, hierarchy = cv2.findContours(th3, mode=cv2.RETR_EXTERNAL, method=cv2.CHAIN_APPROX_SIMPLE)  # :
        # cv2.drawContours(th3, contours, -1, 127, 2)
        # cv2.imshow('th3', th3)

        # cv2.imshow('bar', _)
        k = cv2.waitKey(1)
        if k == ord('s'):
            print("-->>sys :  low__Result : H : {}, S : {}, V : {}".format(low_h, low_s, low_v))
            print("-->>sys :  high_Result : H : {}, S : {}, V : {}".format(high_h, high_s, high_v))
            save_image(img, savepath)
        if k == ord('h'):
            left_back = np.deg2rad([67.83, -71.38, 130.59, -149.20, -90.08, 66.66])
            rob.movej(left_back, 1, 1)
        if k == ord('['):
            left_back = np.deg2rad([55.62, -41.46, 75.42, -123.94, -89.92, 55.89])
            rob.movej(left_back, 1, 1)
        if k == ord(']'):
            right_back = np.deg2rad([21.23, -73.99, 136.01, -152.00, -90.00, 21.50])
            rob.movej(right_back, 1, 1)
        if k == ord('\''):
            right_front = np.deg2rad([125.01, -75.55, 138.53, -152.96, -90.00, 125.27])
            rob.movej(right_front, 1, 1)
        if k == ord(';'):
            left_front = np.deg2rad([105.98, -42.31, 77.11, -124.78, -89.93, 106.26])
            rob.movej(left_front, 1, 1)

        if k & 0xFF == 27:  # ESC
            exit_program(savepath)
            break

        #####################
        # img_temp01 = cv2.imread(
        #     'C:/Users/user/Desktop/AI_Project/ball_calib_python/20200508_LG_labeling_00/org/img_20200508-17-42-55.png')
        # cv2.imshow("test_img01", img_temp01)
        # cv2.waitKey(2)
        #
        # img_temp02 = cv2.imread(
        #     'C:/Users/user/Desktop/AI_Project/ball_calib_python/20200508_LG_labeling_00/blue_cup/img_20200508-17-44-33.png')
        # cv2.imshow("test_img02", img_temp02)
        # cv2.waitKey(2)
        #
        # img_temp03 = np.array(img_temp02) - np.array(img_temp01)
        # cv2.imshow("test_img03", img_temp03)
        # cv2.waitKey(2)