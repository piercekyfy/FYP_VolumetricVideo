# Records a .bag clip of each connected camera

import pyrealsense2 as rs
import cv2
import numpy as np
import time
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
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        if(not configuration_dict['preview']):
            config.enable_record_to_file(configuration_dict['file_name'] + "_" + serial + ".bag")

        serials.append(serial)
        pipelines.append(pipeline)
        configs.append(config)
    return serials, pipelines, configs

def configure_sensors(profile):
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
    depth_sensor.set_option(rs.option.emitter_enabled, 1)
    depth_sensor.set_option(rs.option.laser_power, 30)

def get_configuration():
    configuration = {}
    configuration['preview'] = bool_input("Preview? (Don't record to file):", False)
    configuration['file_name'] = string_input("File (No extension):", str(time.time()))
    configuration['duration'] = float_input("Duration (Seconds):", 1)
    return configuration

def record_bag(configuration_dict):
    serials, pipelines, configs = get_pipelines_configs(configuration_dict)
    profiles = []
    windows = []

    for i in range(len(pipelines)):
        profiles.append(pipelines[i].start(configs[i]))
        configure_sensors(profiles[i])
        windows.append(serials[i])
    
    print("Starting...")
    
    for i in range(120): # Let some frames pass for auto exposure
        for pipeline in pipelines:
            pipeline.wait_for_frames()

    start = time.time()

    colorizer = rs.colorizer()
    while(time.time() - start < configuration_dict['duration']):
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

    print("Finishing...")

    cv2.destroyAllWindows()

    for pipeline in pipelines:
        pipeline.stop()

configuration = get_configuration()
record_bag(configuration)