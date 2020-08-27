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
다만, 기존에 Joint를 이미 구해놨다면, 특별히 joint를 구할 필요는 없다.

 ## Generating HSV filter
 이 과정에서는 camera로 detect하는 object의 filter를 구한다.
 Camera의 HSV 값을 조절하며 화면에서 object만을 detect 하는 값을 저장하면 된다.

 ## Collecting Joint of Robot
 이 과정에서는 robot의 joint를 구한다. 직접 robot을 움직여가며 joint를 구한다.

 ## Assembling filter and joint
 기존에 구한 filter 값과 joint 값들을 합쳐서 calibration을 하는데 필요한 binary file을 만든다.

 이 모든 과정이 끝난다면, 