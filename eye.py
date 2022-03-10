import pyrealsense2 as rs
import numpy as np
import cv2


def automatic_thresh(picture):
    """
    Automatic threshold value calculation
    :param picture: picture
    :return: lower and upper threshold value
    """
    med = np.median(picture)
    sigma = 0.50
    lower_thresh = int(max(0, (1.0 - sigma) * med))
    upper_thresh = int(min(255, (1.0 + sigma) * med))
    return lower_thresh, upper_thresh


def parameters(detector_params):
    """
    Blob parameters
    :param detector_params: created SimpleBlob parameter
    :return: specific filters and values for the parameters
    """
    detector_params.filterByArea = True
    detector_params.maxArea = 5000
    detector_params.minArea = 100
    detector_params.minThreshold = 2
    detector_params.maxThreshold = 200
    # detector_params.filterByConvexity = True
    # detector_params.minConvexity = 0.40
    detector_params.filterByCircularity = True
    detector_params.minCircularity = 0.5
    # detector_params.filterByColor = True
    # detector_params.blobColor = 0
    return detector_params


def modify(img, detector):
    """
    Morphological modifications
    :param img: input image
    :return: keypoint
    """
    img = cv2.erode(img, None, iterations=2)
    img = cv2.dilate(img, None, iterations=4)
    img = cv2.medianBlur(img, 5)
    return detector.detect(img)


def process(img):
    img = adjust_gamma(img)

    # img = img[180:400, 140:500]  # 480x640
    detector_params = cv2.SimpleBlobDetector_Params()
    detector = cv2.SimpleBlobDetector_create(parameters(detector_params))
    eyes = eye_cascade.detectMultiScale(img, scaleFactor=1.05, minNeighbors=5, maxSize=(400, 400),
                                        minSize=(50, 50))

    for (ex, ey, ew, eh) in eyes:
        img = cv2.rectangle(img, (ex, ey), (ex + ew, ey + eh), (0, 255, 0), 5)

        #eye = img[ey:ey + eh, ex:ex + ew]
        #threshold_min, threshold_max = automatic_thresh(eye)
        #_, l_img = cv2.threshold(eye, 100, 255, cv2.THRESH_BINARY)
        #cv2.imshow("eye", img)
        keypoints = modify(img, detector)
        img = cv2.drawKeypoints(img, keypoints, 0, (255, 0, 0),
                          cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        # cv2.imshow("eye", l_img)

    return img


def adjust_gamma(image, gamma=1.3):
    """
    Use lookup table mapping the pixel values [0, 255] to their adjusted gamma values
    Bigger default gamma value means brighter img
    :param image: input image
    :param gamma: gamma value
    :return:
    """
    invGamma = 1.0 / gamma
    table = np.array([((i / 255.0) ** invGamma) * 255
                      for i in np.arange(0, 256)]).astype("uint8")
    return cv2.LUT(image, table)


eye_cascade = cv2.CascadeClassifier('haarcascade_eye.xml')
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter("video1.avi", fourcc, 10, (640, 480), False)

# Configure IR stream
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.infrared, 640, 480, rs.format.y8, 60)

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
        img = process(IR_image)
        out.write(img)
        cv2.imshow('IR image', img)


        # Exit on ESC key
        key = cv2.waitKey(1) % 0x100

        # Screenshot on SPACE key
        if key % 256 == 32:
            cv2.imwrite("D:\\PycharmProjects\\InfraRed\\0310test\\" + str(i) + ".png", img)
            i += 1
        if key == 27 or key == ord('q'):
            break

finally:
    pipeline.stop()
    out.release()
    cv2.destroyAllWindows()
