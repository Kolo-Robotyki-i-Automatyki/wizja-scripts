import pyrealsense2 as rs
import numpy as np
import cv2

pipeline = rs.pipeline()
config = rs.config()

# Enable streams
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

pipeline.start(config)

while True:
    frames = pipeline.wait_for_frames()
    depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()

    if depth_frame and color_frame:
        print("Camera working!")
        print(f"Depth: {depth_frame.get_width()}x{depth_frame.get_height()}")
        print(f"Color: {color_frame.get_width()}x{color_frame.get_height()}")

    depth_image = np.asanyarray(depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())
    print(depth_image.dtype)
    print(depth_image.shape)
    print(depth_image[100:110, 100:110])
    print(depth_image.min(), depth_image.max())
    depth_colormap = cv2.applyColorMap(
        cv2.convertScaleAbs(depth_image, alpha=0.03),
        cv2.COLORMAP_JET
    )
    together = np.hstack([color_image, depth_colormap])
    cv2.imshow('Depth & Color', together)
    # cv2.imshow('Color', color_frame)
    # cv2.waitKey(0)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
pipeline.stop()