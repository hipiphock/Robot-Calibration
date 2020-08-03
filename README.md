# Calibration
카메라는 필연적으로 왜곡이 있을 수 밖에 없다. 이는 빛이 들어오는 거리와 방향에 의해서 일어나는 자연스러운 현상이다. 하지만, 로봇과 같이 정밀한 움직임을 요구하는 기기에게는 이러한 왜곡은 critical할 수 밖에 없다. 이러한 문제를 해결하기 위해서 calibration을 해준다.

# Prerequisite
기본적으로 다음 library들이 필요하다.
 * Pyrealsense2
 * numpy

# Design
이 project에서 Calibration의 과정은 네 가지로 나뉜다.
 * Generating HSV filter
 * Collecting Joint of Robot
 * Assembling filter and joint
 * Executing RANSAC Algorithm