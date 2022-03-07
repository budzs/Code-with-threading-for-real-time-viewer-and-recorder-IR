import pyrealsense2 as rs
import numpy as np
import cv2

# Configure IR stream
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.infrared, 640, 480, rs.format.y8, 30)

# Start input streaming
pipeline.start(config)
# Ignore first 1sec for camera warm-up
for i in range(30):
    frames = pipeline.wait_for_frames()
i = 0

try:
    while True:
        # Read image
        frames = pipeline.wait_for_frames()
        infrared_frame = frames.first(rs.stream.infrared)
        IR_image = np.asanyarray(infrared_frame.get_data())

        # Display image
        cv2.imshow('IR image', IR_image)

        # Exit on ESC key
        key = cv2.waitKey(1) % 0x100

        # Screenshot on SPACE key
        if key % 256 == 32:
            img = cv2.imshow('IR screenshot', IR_image)
            filename = "D:\\PycharmProjects\\IR\\Rec 03.03\\" + str(i) + ".png"
            cv2.imwrite(filename, IR_image)
            i += 1
        if key == 27 or key == ord('q'):
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
