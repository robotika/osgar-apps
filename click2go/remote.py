"""
    Osgar remote control via pygame
"""
import math
from threading import Thread
from osgar.node import Node
import cv2

#import cv2
import queue
#from osgar.node import Node

# Global or shared queue to pass frames to the main thread
# Using a small maxsize ensures we only show the freshest frame
visualization_queue = queue.Queue(maxsize=1)


class RemoteClient(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
#        self.input_thread = Thread(target=self.run_input, daemon=True)
#        self.bus = bus
        bus.register("cmd")
        self.verbose = False
        self.max_speed = 0.5
        self.max_angle = 45
        self.last_image = None

#    def start(self):
#        self.input_thread.start()

#    def join(self, timeout=None):
#        self.input_thread.join(timeout=timeout)

    def send_speed(self, speed, angle):
        self.publish('cmd', [0, 0])

    def on_image(self, data):
#        print(data)
        with open('tmp.h264', 'wb') as f:
            f.write(data)
        cap = cv2.VideoCapture('tmp.h264')
        img = cap.read()[-1]
        cap.release()
        self.last_image = cv2.resize(img, (img.shape[1] // 2, img.shape[0] // 2))

        # 1. Process your image (data is usually JPEG or raw)
        # frame = cv2.imdecode(data, cv2.IMREAD_COLOR)

        # 2. Push to the visualization queue
        if not visualization_queue.full():
            visualization_queue.put(data)

    def run(self):
        speed = 0
        angle = 0  # degrees
        max_speed = self.max_speed

        self.send_speed(speed, math.radians(angle))

        while self.bus.is_alive():
            self.update()
            self.sleep(0.1)
            """
            if self.last_image is None:
                continue
#            img = cv2.imread('thumb.jpg')
            cv2.imshow('Remote Control', self.last_image)
            key = cv2.waitKey(100)
            print(key)
            if key == 27 or key == ord('q'):
                cv2.destroyAllWindows()
                self.request_stop()
                break
            elif key == ord('a'):
                print(f"{speed:0.1f}, {angle}")
                self.send_speed(speed, math.radians(angle))
"""



###############


class MyOpenCVNode(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        # Register what we want to listen to (e.g., a camera stream)
        bus.register('image')

    def run(self):
        try:
            while self.is_alive():
                # wait() receives data from the OSGAR bus
                dt, channel, data = self.listen()

                if channel == 'image':
                    # 1. Process your image (data is usually JPEG or raw)
                    # frame = cv2.imdecode(data, cv2.IMREAD_COLOR)

                    # 2. Push to the visualization queue
                    if not visualization_queue.full():
                        visualization_queue.put(data)

        except Exception as e:
            print(f"Node error: {e}")


# --- Main Entry Point ---
if __name__ == "__main__":
    from osgar.lib.config import load as config_load
    from osgar.record import Recorder

    # Initialize OSGAR (your config should include MyOpenCVNode)
#    cfg = config_load("my_robot_config.json")
    cfg = config_load("config/rc_client.json")
    recorder = Recorder(config=cfg['robot'])

    # Start all nodes (they run in background threads)
    recorder.start()

    try:
        while True:
            # 3. Main thread pulls from queue and handles GUI
            try:
                frame = visualization_queue.get(timeout=0.1)
                cv2.imshow("OSGAR Visualization", frame)
            except queue.Empty:
                pass

            # IMPORTANT: waitKey must be here on the main thread
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        recorder.stop()
        cv2.destroyAllWindows()
