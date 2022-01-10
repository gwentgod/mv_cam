#!/usr/bin/env python3

import os, cv2
from functools import partial

import rospy as ros
from rospkg import RosPack
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

rospack = RosPack()
BASE_DIR = rospack.get_path("mv_cam") + "/imgs/"
FREQ = 1/4


class ImageSub:
    def __init__(self):
        self.last_seq = -1
        self.msgs = {'l': None, 's': None}

        self.long_sub = ros.Subscriber("/cam/long_exp", Image, partial(self.cb, 'l'), queue_size=1)
        self.short_sub = ros.Subscriber("/cam/short_exp", Image, partial(self.cb, 's'), queue_size=1)

        self.bridge = CvBridge()

        if not os.path.exists(BASE_DIR):
            os.mkdir(BASE_DIR)

    def cb(self, exp, msg):
        self.msgs[exp] = msg

    def save(self):
        if self.msgs['l'] and self.msgs['l'].header.seq > self.last_seq:
            for exp in self.msgs:
                image = self.bridge.imgmsg_to_cv2(self.msgs[exp], "passthrough")
                filename = "{}_{}.png".format(self.msgs[exp].header.stamp.secs, exp)
                cv2.imwrite(BASE_DIR+filename, image)
                ros.loginfo("{} Saved".format(filename))
        else:
            ros.logwarn("No image received, waiting")


def main():
    ros.init_node("img_sub")
    rate = ros.Rate(FREQ)

    sub = ImageSub()

    ros.sleep(1)

    while not ros.is_shutdown():
        sub.save()
        rate.sleep()


if __name__ == "__main__":
    main()
