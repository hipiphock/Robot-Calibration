import time
import copy
import numpy as np

from context import Calibration
import Calibration.CameraStream as CameraStream
import Calibration.HSVfilter as HSVfilter
import Calibration.JointCollector as JointCollector
import Calibration.Assembler as Assembler
import Calibration.RobotHandler as RobotHandler

if __name__ == "__main__":

    # basic configuration used for all routines
    pipeline = CameraStream.config_pipeline()

    cam_robot_addr =    "192.168.0.2"
    nocam_robot_addr =  "192.168.0.29" 
    cam_robot, nocam_robot = RobotHandler.set_robot(cam_robot_addr, nocam_robot_addr)

    # choose save path to save filter, joint, and other stuff
    # TODO: configure the fie name of hsv_filter, joint, and calculated matrix files
    save_dir = "./resource/example/"

    # save hsv filter
    HSVfilter.save_hsv_filter(pipeline, cam_robot, nocam_robot, save_dir)

    # TODO: implement additional method for fetching hsv filter path
    hsv_filter_path = "./resource/example/hsv_result.txt"

    # collect joint
    JointCollector.collect_joint(pipeline, cam_robot, nocam_robot, hsv_filter_path, save_dir)

    joint_path = "./resource/example/"

    # assemble collected joint and hsv filter
    Assembler.create_binary(pipeline, cam_robot, nocam_robot, hsv_filter_path, save_dir)

    # finally, activate Kinetic project in Visual Studio
    # TODO: embed or extend python into c++ project