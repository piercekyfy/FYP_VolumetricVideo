import sys
import getopt
import pyrealsense2 as rs
import numpy as np
import cv2

args = sys.argv[1:]

try:
    arguments, values = getopt.getopt(args, "b:o:", ["Bag=", "Output="])

    for arg, val in arguments:
        print(arg)
        print(val)
except getopt.error as err:
    print(err)