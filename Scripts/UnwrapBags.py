import sys
import getopt
import os.path
import json

import pyrealsense2 as rs
import numpy as np
import cv2

args = sys.argv[1:]

bag_files = []
output_directory = None

try: # TODO: --Help
    arguments, values = getopt.getopt(args, "b:o:", ["Bag=", "Output="])

    for arg, val in arguments:
        if(arg == "-b" or arg == "--Bag"):
            bag_files.append(val)
        elif(arg == "-o" or arg == "--Output"):
            output_directory = val
except getopt.error as err:
    print(err)

if(len(bag_files) <= 0):
    raise Exception(">=1 Bag file must be specified. Specify bag files with -b or --Bag")
if(output_directory == None or output_directory == ""):
    raise Exception("Output directory must be specified. Specify output directory with -o or --Output")

def translate_stream_type(stream_type):
        if(stream_type == 0 or stream_type == 2): # any or color
            return 1 # color
        if(stream_type == 1): # depth
            return 2 #depth
        else:
            return 1 # color. TODO: support ir, etc.

def save_frames(pipeline, out_directory):
    index = 0
    while True:
        try:
            frames = pipeline.wait_for_frames(timeout_ms = 1000)
        except RuntimeError:
            break # No Frames
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()

        if not color_frame or not depth_frame:
            continue

        color_img = np.asanyarray(color_frame.get_data())
        depth_data = np.asanyarray(depth_frame.get_data(), dtype=np.uint16)

        cv2.imwrite(os.path.join(out_directory, str(translate_stream_type(rs.stream.color)), f"{index:06d}.png"), color_img)
        depth_data.tofile(os.path.join(out_directory, str(translate_stream_type(rs.stream.depth)), f"{index:06d}.bin"))

        index += 1
    pipeline.stop()

def process_bag(bag_file, output_directory):
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_device_from_file(bag_file, repeat_playback=False)
    profile = pipeline.start(config)
    profile.get_device().as_playback().set_real_time(False)
    
    serial = profile.get_device().get_info(rs.camera_info.serial_number)

    color_stream = profile.get_stream(rs.stream.color)
    depth_stream = profile.get_stream(rs.stream.depth)
    depth_sensor = profile.get_device().first_depth_sensor()
    os.makedirs(os.path.join(output_directory, serial), exist_ok=True)
    os.makedirs(os.path.join(output_directory, serial, str(translate_stream_type(rs.stream.color))), exist_ok=True)
    os.makedirs(os.path.join(output_directory, serial, str(translate_stream_type(rs.stream.depth))), exist_ok=True)
    save_frames(pipeline, os.path.join(output_directory, serial))

    description = {}

    def get_stream_obj(stream):
        vs = stream.as_video_stream_profile()
        intr = vs.get_intrinsics()
        return {
            "stream_type": translate_stream_type(int(stream.stream_type())),
            "width": vs.width(),
            "height": vs.height(),
            "fps": stream.fps(),
            "bpp": stream.bytes_per_pixel(),
            "intrinsics": { 
                "width": intr.width, 
                "height": intr.height, 
                "ppx": intr.ppx, 
                "ppy": intr.ppy, 
                "fx": intr.fx, 
                "fy": intr.fx, 
                "coeffs": intr.coeffs },
        }

    description['serial'] = serial
    description['depth_scale'] = depth_sensor.get_depth_scale()
    description['streams'] = [ # TODO: support ir, etc.
        get_stream_obj(color_stream),
        get_stream_obj(depth_stream)
    ]
    
    with open(os.path.join(output_directory, serial, "description.json"), 'w') as file:
        json.dump(description, file)

for bag_file in bag_files:
    process_bag(bag_file, output_directory)