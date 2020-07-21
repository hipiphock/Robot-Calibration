import time
import copy
import numpy as np

from context import Calibration
import Calibration.CameraStream as CameraStream
import Calibration.HSVfilter as HSVfilter
import Calibration.JointCollector as JointCollector
import Calibration.Assembler as Assembler

if __name__ == "__main__":

    # basic configuration used for all routines
    pipeline = CameraStream.config_pipeline()

    # choose save path to save filter, joint, and other stuff
    # TODO: configure the fie name of hsv_filter, joint, and calculated matrix files
    save_dir = "resource/somepath"

    # save hsv filter
    HSVfilter.save_hsv_filter(pipeline, save_dir)

    # TODO: implement additional method for fetching hsv filter path
    hsv_filter_path = ""

    # collect joint
    JointCollector.collect_joint(pipeline, hsv_filter_path, save_dir)

    # assemble collected joint and hsv filter
    Assembler.create_binary(pipeline, hsv_filter_path, save_dir)

    # finally, activate Kinetic project in Visual Studio
    # TODO: embed or extend python into c++ project