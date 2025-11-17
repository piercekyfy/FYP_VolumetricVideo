# Plays any .bag files in command arguments.

import sys
import pyrealsense2 as rs
import cv2
import numpy as np

files = sys.argv[1:]

pipelines = []
windows = []

for file in files:
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_device_from_file(file, repeat_playback=False)
    profile = pipeline.start(config)
    windows.append(profile.get_device().get_info(rs.camera_info.serial_number))
    profile.get_device().as_playback().set_real_time(False)
    pipelines.append(pipeline)

colorizer = rs.colorizer()

while True:
    for i in range(len(pipelines)):
            pipeline = pipelines[i]
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            depth_colorized = colorizer.colorize(depth_frame)
            color_img = np.asanyarray(color_frame.get_data())
            depth_img = np.asanyarray(depth_colorized.get_data())
            stacked = np.hstack((color_img, depth_img))

            cv2.imshow(windows[i], stacked)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    
cv2.destroyAllWindows()