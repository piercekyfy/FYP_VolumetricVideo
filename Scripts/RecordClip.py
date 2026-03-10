# Records a .bag clip of each connected camera

import pyrealsense2 as rs
import cv2
import numpy as np
import time
import threading
import random

ctx = rs.context()
devices = ctx.query_devices()

if(len(devices) <= 0):
    raise Exception("No devices connected.")

def string_input(text, default):
    i = None
    print(text)
    i = input("(String)> ").lower()
    if(i == ""):
        return default
    return i

def bool_input(text, default):
    i = None
    while(i == None):
        print(text)
        i = input("(Y/n)> ").lower()
        if(i == ""):
            return default
        if(i == 'y'):
            i = True
        elif(i == 'n'):
            i = False
        else:
            i = None
    return i

def float_input(text, default):
    i = None
    while(i == None):
        print(text)
        i = input("(Float)> ")
        if(i == ""):
            return default
        try:
            i = float(i)
        except ValueError:
            i = None
    return i

def get_pipelines_configs(configuration_dict):
    serials = []
    pipelines = []
    configs = []
    for device in devices:
        pipeline = rs.pipeline()
        config = rs.config()
        serial = device.get_info(rs.camera_info.serial_number)
        config.enable_device(serial)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 15)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)

        if(configuration_dict['enable_ir']):
            config.enable_stream(rs.stream.infrared, 1, 640, 480, rs.format.y8, 15)

        if(not configuration_dict['preview']):
            config.enable_record_to_file(configuration_dict['file_name'] + "_" + serial + ".bag")

        serials.append(serial)
        pipelines.append(pipeline)
        configs.append(config)
    return serials, pipelines, configs

def configure_sensors(profile, configuration_dict):
    device = profile.get_device()
    sensors = device.query_sensors()
    color_sensor = None
    depth_sensor = None
    for sensor in sensors:
        name = sensor.get_info(rs.camera_info.name)
        if(name == "RGB Camera"):
            color_sensor = sensor
        elif(name == "Stereo Module"):
            depth_sensor = sensor

    color_sensor.set_option(rs.option.enable_auto_exposure, 1)
    color_sensor.set_option(rs.option.enable_auto_white_balance, 1)

    depth_sensor.set_option(rs.option.enable_auto_exposure, 1)
    if(configuration_dict['enable_ir']):
        depth_sensor.set_option(rs.option.emitter_enabled, 0)
    else:
        depth_sensor.set_option(rs.option.emitter_enabled, 1)
        depth_sensor.set_option(rs.option.laser_power, 30)

def get_configuration():
    configuration = {}
    configuration['preview'] = bool_input("Preview? (Don't record to file):", False)
    configuration['file_name'] = string_input("File (No extension):", str(time.time()))
    configuration['enable_ir'] = bool_input("Enable IR? (Will degrade depth quality):", False)
    configuration['duration'] = float_input("Duration (Seconds):", 1)
    return configuration

# def record_bag(configuration_dict):
#     serials, pipelines, configs = get_pipelines_configs(configuration_dict)
#     profiles = []
#     windows = []

#     for i in range(len(pipelines)):
#         profiles.append(pipelines[i].start(configs[i]))
#         configure_sensors(profiles[i], configuration_dict)
#         windows.append(serials[i])
    
#     print("Starting...")
    
#     start = time.time()

#     colorizer = rs.colorizer()
#     while(time.time() - start < configuration_dict['duration']):
#         rows = []
#         frames = [None]
#         while(any(f is None for f in frames)):
#             frames = [None] * len(pipelines)
#             for i in range(len(pipelines)):
#                 frames[i] = pipelines[i].wait_for_frames()
        
#         for frame in frames:
#             color_frame = frame.get_color_frame()
#             depth_frame = frame.get_depth_frame()
#             ir_frame = frame.get_infrared_frame(1)
#             if not color_frame or not depth_frame:
#                 continue
#             depth_colorized = colorizer.colorize(depth_frame)
#             color_img = cv2.cvtColor(np.asanyarray(color_frame.get_data()), cv2.COLOR_RGB2BGR)
#             depth_img = np.asanyarray(depth_colorized.get_data())
#             rows.append(np.hstack((color_img, depth_img)))

#         if rows:
#             cv2.imshow("cameras", np.vstack(rows))
#         if cv2.waitKey(1) & 0xFF == ord('q'):
#             break

#     print("Finishing...")

#     cv2.destroyAllWindows()

#     for pipeline in pipelines:
#         pipeline.stop()

def record_bag(configuration_dict):
    serials, pipelines, configs = get_pipelines_configs(configuration_dict)
    profiles = []

    for i in range(len(pipelines)):
        profiles.append(pipelines[i].start(configs[i]))
        configure_sensors(profiles[i], configuration_dict)

    print("Starting...")
    start = time.time()
    colorizer = rs.colorizer()
    
    latest_frames = [None] * len(pipelines)
    lock = threading.Lock()

    def grab_frames(idx):
        while time.time() - start < configuration_dict['duration']:
            frame = pipelines[idx].wait_for_frames()
            with lock:
                latest_frames[idx] = frame

    threads = [threading.Thread(target=grab_frames, args=(i,), daemon=True)
               for i in range(len(pipelines))]
    for t in threads:
        t.start()

    while time.time() - start < configuration_dict['duration']:
        with lock:
            frames = list(latest_frames)

        if any(f is None for f in frames):
            time.sleep(0.001)
            continue

        rows = []
        for frame in frames:
            color_frame = frame.get_color_frame()
            depth_frame = frame.get_depth_frame()
            if not color_frame or not depth_frame:
                continue
            depth_colorized = colorizer.colorize(depth_frame)
            color_img = cv2.cvtColor(np.asanyarray(color_frame.get_data()), cv2.COLOR_RGB2BGR)
            depth_img = np.asanyarray(depth_colorized.get_data())
            rows.append(np.hstack((color_img, depth_img)))

        if rows:
            cv2.imshow("cameras", np.vstack(rows))
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()
    for pipeline in pipelines:
        pipeline.stop()

configuration = get_configuration()
record_bag(configuration)