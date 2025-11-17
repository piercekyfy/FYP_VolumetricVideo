# Shows the infrared feed of all connected cameras. Any checkerboard found within the image is highlighted.
# Use this to easily tell if all cameras can see a checkerboard pattern.
import pyrealsense2 as rs
import cv2
import numpy as np

ctx = rs.context()
devices = ctx.query_devices()

if(len(devices) <= 0):
    raise Exception("No devices connected.")


pipelines = []
windows = []

for device in devices:
    pipeline = rs.pipeline()
    config = rs.config()
    serial = device.get_info(rs.camera_info.serial_number)
    config.enable_device(serial)
    config.enable_stream(rs.stream.infrared, 640, 480, rs.format.y8, 30)

    windows.append(serial)
    profile = pipeline.start(config)
    pipelines.append(pipeline)
    
    sensors = device.query_sensors()
    ir_sensor = None
    for sensor in sensors:
        name = sensor.get_info(rs.camera_info.name)
        if(name == "Stereo Module"):
            ir_sensor = sensor
            break

    ir_sensor.set_option(rs.option.enable_auto_exposure, 1)
    ir_sensor.set_option(rs.option.emitter_enabled, 0)

checkerboard_params = [6, 9, 0.253]

for i in range(120): # Let some frames pass for auto exposure
    for pipeline in pipelines:
        pipeline.wait_for_frames()

while True:
    for i in range(len(pipelines)):
        pipeline = pipelines[i]
        frames = pipeline.wait_for_frames()
        ir_frame = frames.get_infrared_frame()
        ir_img = np.asanyarray(ir_frame.get_data())

        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        checkerboard_found = False
        checkerboard_found, corners = cv2.findChessboardCorners(ir_img, (
        checkerboard_params[0], checkerboard_params[1]))

        if checkerboard_found:
            corners = cv2.cornerSubPix(ir_img, corners, (11,11),(-1,-1), criteria)
            cv2.drawChessboardCorners(ir_img, (checkerboard_params[0], checkerboard_params[1]), corners, checkerboard_found)

        cv2.imshow(windows[i], ir_img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break