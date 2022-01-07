#!/usr/bin/env python3

import mvsdk
import numpy as np

import rospy as ros
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class Camera():
    def __init__(self, cam_info):
        self.camera_nh = -1
        self.capability = None
        self.frame_buffer = None

        self.exposure_time = (15 * 1000, 1.5*1000)
        # self.gamma = 1
        self.contrast = 100
        self.saturation = 100
        self.rgb_gain = (91,87,100)
        # self.sharpness = 0

        try:
            self.camera_nh = mvsdk.CameraInit(cam_info, -1, -1)
        except mvsdk.CameraException as e:
            ros.logfatal("CameraInit Failed({}): {}".format(e.error_code, e.message))
            exit()

        # get cam info
        self.capability = mvsdk.CameraGetCapability(self.camera_nh)


        # calculate and allocate the frame buffer size
        buffer_size = \
                self.capability.sResolutionRange.iWidthMax * self.capability.sResolutionRange.iHeightMax * 3
        self.frame_buffer = mvsdk.CameraAlignMalloc(buffer_size, 16)

        # config
        mvsdk.CameraSetIspOutFormat(self.camera_nh, mvsdk.CAMERA_MEDIA_TYPE_BGR8)
        # mvsdk.CameraSetGamma(self.camera_nh, self.gamma)
        mvsdk.CameraSetContrast(self.camera_nh, self.contrast)
        mvsdk.CameraSetSaturation(self.camera_nh, self.saturation)
        mvsdk.CameraSetGain(self.camera_nh, *self.rgb_gain)
        # mvsdk.CameraSetSharpness(self.camera_nh, self.sharpness)

        # set trigger mode to soft trigger
        mvsdk.CameraSetTriggerMode(self.camera_nh, 1)

        # set to manual exposure
        mvsdk.CameraSetAeState(self.camera_nh, 0)
        mvsdk.CameraPlay(self.camera_nh)

        # fill the buffer in case time out
        mvsdk.CameraSetExposureTime(self.camera_nh, self.exposure_time[0])
        mvsdk.CameraSoftTrigger(self.camera_nh)

    def __del__(self):
        mvsdk.CameraUnInit(self.camera_nh)
        mvsdk.CameraAlignFree(self.frame_buffer)

    def grab(self, exp_time):
        mvsdk.CameraSetExposureTime(self.camera_nh, exp_time)
        mvsdk.CameraSoftTrigger(self.camera_nh)

        try:
            raw_data, frame_head = mvsdk.CameraGetImageBuffer(self.camera_nh, 200)
            mvsdk.CameraImageProcess(self.camera_nh, raw_data, self.frame_buffer, frame_head)

            frame_data = (mvsdk.c_ubyte * frame_head.uBytes).from_address(self.frame_buffer)
            frame = np.frombuffer(frame_data, dtype=np.uint8)
            frame = frame.reshape((frame_head.iHeight, frame_head.iWidth, 3))
            mvsdk.CameraReleaseImageBuffer(self.camera_nh, raw_data)

            return frame

        except mvsdk.CameraException as e:
            if e.error_code != mvsdk.CAMERA_STATUS_TIME_OUT:
                ros.logerr("CameraGetImageBuffer failed({}): {}".format(e.error_code, e.message) )
            return False


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
    ros.loginfo(cam_info)

    cam = Camera(cam_info)
    bridge = CvBridge()

    ros.sleep(2)

    seq = 0
    while not ros.is_shutdown():
        image = cam.grab(cam.exposure_time[seq % 2])
        img_msg = bridge.cv2_to_imgmsg(image, "bgr8")
        img_msg.header.seq = seq
        img_msg.header.stamp = ros.Time.now()
        if seq % 2:
            long_exp_pub.publish(img_msg)
        else:
            short_exp_pub.publish(img_msg)
        seq += 1


if __name__ == "__main__":
        main()
