import time
import copy

import numpy as np
import cv2
import pyrealsense2 as rs
import urx

from context import Calibration
import Calibration.CameraStream as CameraStream
import Calibration.HSVfilter as HSVfilter
import Calibration.JointCollector as JointCollector
import Calibration.Assembler as Assembler

if __name__ == "__main__":

    # basic configuration used for all routines
    pipeline = CameraStream.config_pipeline()

    # choose save path to save filter, joint, and other stuff
    savepath = "resource/somepath"

    # save hsv filter
    HSVfilter.save_hsv_filter(pipeline, savepath)

    # collect joint
    JointCollector.collect_joint(pipeline, savepath, savepath)

    # assemble collected joint and hsv filter
    Assembler.create_binary(pipeline, savepath, savepath)

    # finally, activate Kinetic project in Visual Studio
