#!/usr/bin/env python3

import mvsdk
import yaml
import numpy as np

import rospy as ros
from rospkg import RosPack
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class Camera:
    def __init__(self, cam_info):
        self.camera_nh = -1
        self.capability = None
        self.frame_buffer = None
        self.config = None

        self.rospack = RosPack()
        self.pack_path = self.rospack.get_path("mv_cam")

        try:
            self.camera_nh = mvsdk.CameraInit(cam_info, -1, -1)
        except mvsdk.CameraException as e:
            ros.logfatal("CameraInit Failed({}): {}".format(e.error_code, e.message))
            exit()

        # get cam info
        self.capability = mvsdk.CameraGetCapability(self.camera_nh)

        # calculate and allocate the frame buffer size
        buffer_size = self.capability.sResolutionRange.iWidthMax * self.capability.sResolutionRange.iHeightMax * 3
        self.frame_buffer = mvsdk.CameraAlignMalloc(buffer_size, 16)

        self.load_config()

        # set trigger mode to soft trigger
        mvsdk.CameraSetTriggerMode(self.camera_nh, 1)

        # set to manual exposure
        mvsdk.CameraSetAeState(self.camera_nh, 0)
        mvsdk.CameraPlay(self.camera_nh)

        # fill the buffer in case time out
        mvsdk.CameraSetExposureTime(self.camera_nh, self.config["exposure_time"][0])
        mvsdk.CameraSoftTrigger(self.camera_nh)

    def __del__(self):
        mvsdk.CameraUnInit(self.camera_nh)
        mvsdk.CameraAlignFree(self.frame_buffer)

    def load_config(self):
        try:
            with open(self.pack_path + "/config.yaml") as cf:
                self.config = yaml.load(cf, Loader=yaml.FullLoader)
            for c in "dynamic_config", "output_format", "exposure_time", \
                     "contrast", "saturation", "rgb_gain", "sharpness":
                if c not in self.config:
                    raise Exception
            ros.loginfo("Loaded config from mv_cam/config.yaml")

        except:
            ros.logwarn("Load config faild, using default")
            self.config = {
                "dynamic_config": False,
                "output_format": "bgr8",
                "exposure_time": [18000, 1500],
                "contrast": 100,
                "saturation": 100,
                "rgb_gain": [90, 85, 120],
                "sharpness": 0
            }

        if self.config["output_format"] == "bgr10":
            mvsdk.CameraSetIspOutFormat(self.camera_nh, mvsdk.CAMERA_MEDIA_TYPE_BGR10)
        elif self.config["output_format"] == "bgr12":
            mvsdk.CameraSetIspOutFormat(self.camera_nh, mvsdk.CAMERA_MEDIA_TYPE_BGR12)
        else:
            mvsdk.CameraSetIspOutFormat(self.camera_nh, mvsdk.CAMERA_MEDIA_TYPE_BGR8)

        mvsdk.CameraSetContrast(self.camera_nh, self.config["contrast"])
        mvsdk.CameraSetSaturation(self.camera_nh, self.config["saturation"])
        mvsdk.CameraSetGain(self.camera_nh, *self.config["rgb_gain"])
        mvsdk.CameraSetSharpness(self.camera_nh, self.config["sharpness"])

    def grab(self, seq):
        mvsdk.CameraSetExposureTime(self.camera_nh, self.config["exposure_time"][seq%2])
        mvsdk.CameraSoftTrigger(self.camera_nh)

        try:
            raw_data, frame_head = mvsdk.CameraGetImageBuffer(self.camera_nh, 100)
            mvsdk.CameraImageProcess(self.camera_nh, raw_data, self.frame_buffer, frame_head)
            mvsdk.CameraReleaseImageBuffer(self.camera_nh, raw_data)

            frame_data = (mvsdk.c_ubyte * frame_head.uBytes).from_address(self.frame_buffer)
            frame = np.ctypeslib.as_array(frame_data)
            frame = frame.reshape((frame_head.iHeight, frame_head.iWidth, 3))

            return frame

        except mvsdk.CameraException as e:
            if e.error_code == mvsdk.CAMERA_STATUS_TIME_OUT:
                ros.logwarn("Grab frame time out")
            else:
                ros.logerr("CameraGetImageBuffer failed({}): {}".format(e.error_code, e.message))

            return None


def main():
    ros.init_node("cam_multi_exp")
    long_exp_pub = ros.Publisher("/cam/long_exp", Image, queue_size=10)
    short_exp_pub = ros.Publisher("/cam/short_exp", Image, queue_size=10)

    cam_list = mvsdk.CameraEnumerateDevice()
    cam_num = len(cam_list)
    if cam_num < 1:
        ros.logfatal("No camera was found!")
        exit()

    for cam, info in enumerate(cam_list):
        ros.loginfo("{}: {} {}".format(cam, info.GetFriendlyName(), info.GetPortType()))

    if cam_num == 1:
        selected_cam = 0
    else:
        ros.loginfo("Select a camera:")
        selected_cam = int(input())

    cam_info = cam_list[selected_cam]
    ros.loginfo("\n{}".format(cam_info))

    cam = Camera(cam_info)
    bridge = CvBridge()

    ros.sleep(2)

    seq = 0
    while not ros.is_shutdown():
        image = cam.grab(seq)

        if type(image) is np.ndarray:
            img_msg = bridge.cv2_to_imgmsg(image, cam.config["output_format"])
            img_msg.header.seq = seq
            img_msg.header.stamp = ros.Time.now()
        else:
            ros.logerr("Not image obtained")
            continue

        if seq % 2:
            long_exp_pub.publish(img_msg)
        else:
            short_exp_pub.publish(img_msg)

        if cam.config["dynamic_config"] and seq % 100 == 0:
            cam.load_config()

        seq += 1


if __name__ == "__main__":
    main()
