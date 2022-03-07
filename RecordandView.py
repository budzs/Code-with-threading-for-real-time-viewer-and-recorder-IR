import pyrealsense2 as rs
import numpy as np
import datetime as dt
import time
import os
import cv2
from queue import Queue
import threading as th
from datetime import datetime


class RecordingJob(th.Thread):

    def __init__(self, video_name, queue):
        super(RecordingJob, self).__init__()
        self.stop_flag = th.Event()
        self.stop_flag.set()
        self.video_name = video_name
        self.queue = queue

    def run(self):
        self.num = 0
        self.fourcc = cv2.VideoWriter_fourcc(*'XVID')
        self.out = cv2.VideoWriter(self.video_name, self.fourcc, 60, (640, 480), False)
        while self.stop_flag.is_set():

            if not self.queue.empty():
                self.out.write(self.queue.get())
                self.num += 1
        time.sleep(1)
        print(self.num)
        self.out.release()
        print('Thread Exitted')

    def stop(self):
        self.stop_flag.clear()


if __name__ == '__main__':

    directory_path = '0307test'
    if not os.path.exists(directory_path):
        os.mkdir(directory_path)
    name = input("Give the recording a name!\n")

    infa_video_name = directory_path + '/' + name +'.avi'
    infra_queue = Queue()

    t = RecordingJob(infa_video_name, infra_queue)
    t.start()

    # Configure IR stream
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.infrared, 640, 480, rs.format.y8, 60)

    """    
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 60)"""

    # Start input streaming
    pipeline.start(config)
    frame_count = 0
try:

    try:
        print('Start Recording %s' % (str(dt.datetime.now())))
        date_start = time.time()

        while True:
            frames = pipeline.wait_for_frames()

            infra_frame = frames.first(rs.stream.infrared)
            if not infra_frame:
                print('No infra frame')

            infra_image = infra_frame.get_data()
            infra_image = np.asanyarray(infra_image)

            cv2.imshow('frame', infra_image)
            cv2.waitKey(1)

            infra_queue.put(infra_image)

            key = cv2.waitKey(1) % 0x100
            # Screenshot on SPACE key
            if key % 256 == 32:
                #img = cv2.imshow('IR screenshot', infra_image)
                filename = "D:\\PycharmProjects\\InfraRed\\0307test\\" + str(frame_count) + ".png"
                cv2.imwrite(filename, infra_image)

            frame_count += 1

            # if frame_count >= 100:
            #    break
            # Exit on ESC or "q" key
            if key == 27 or key == ord('q'):
                break

    except Exception as e:
        print('Error is ', str(e))
finally:
    print(str(dt.datetime.now()))
    pipeline.stop()
    print('frame_count=', str(frame_count))
    t.stop()
    # cv2.destroyAllWindows()
