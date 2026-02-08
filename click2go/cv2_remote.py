"""
  Remote clinet using OpenCV and ZeroMQ I/O
"""
import cv2
import zmq
import numpy as np

from osgar.lib.serialize import serialize, deserialize
from osgar.logger import LogWriter

DOWNSCALE = 2

pending_click = None

def mouse_callback(event, x, y, flags, param):
    global pending_click
    if event == cv2.EVENT_LBUTTONDOWN:
#        pending_click = f"CV2_Client:{x},{y}"
        pending_click = [x, y]

def start_cv_client(server_ip):
    global pending_click
    context = zmq.Context()

    sub_socket = context.socket(zmq.SUB)
    # Set a receive timeout (1000ms = 1s)
    sub_socket.setsockopt(zmq.RCVTIMEO, 1000) 
    sub_socket.connect(f"tcp://{server_ip}:5555")
    sub_socket.setsockopt_string(zmq.SUBSCRIBE, "")

    push_socket = context.socket(zmq.PUB)
    push_socket.connect(f"tcp://{server_ip}:5556")

    window_name = "ZMQ Stream (with Timeout)"
    cv2.namedWindow(window_name)
    cv2.setMouseCallback(window_name, mouse_callback)

    pose2d = [0, 0, 0]
    print("Client started. Press 'q' to exit.")

    with LogWriter('base-viewer-') as log:
        print(f'Logging into {log.filename}')
        fromrobot_id = log.register('fromrobot')
        torobot_id = log.register('torobot')
        try:
            index = 0
            robot_index = None
            while True:
                try:
                    # This will now only block for 1 second
    #                jpg_buffer = sub_socket.recv()
                    channel, raw = sub_socket.recv_multipart()
                    log.write(fromrobot_id, raw)
                    robot_index, robot_time_sec, pose2d, jpg_buffer = deserialize(raw)
                    print(f'Received {channel} {len(jpg_buffer)} data!')

                    nparr = np.frombuffer(jpg_buffer, np.uint8)
                    img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                    if img is None:
                        # try decode video instead
                        with open('tmpX.h26x', 'wb') as f:
                            f.write(jpg_buffer)
                        cap = cv2.VideoCapture('tmpX.h26x')
                        ret, img = cap.read()
                        cap.release()
                        if img is not None:
                            h, w = img.shape[:2]
                            img = cv2.resize(img, (w//DOWNSCALE, h//DOWNSCALE))

                    if img is not None:
                        cv2.imshow(window_name, img)

                except zmq.Again:
                    # This block hits if the 1s timeout is reached
                    print("No image received in the last second... waiting.")
                    # Optional: display a 'Connection Lost' overlay on the last frame

                # This part now runs even if no image was received
                if pending_click:
    #                push_socket.send_string(pending_click)
                    data = pending_click.copy()
                    raw = serialize([index, robot_index, pose2d, [x*DOWNSCALE for x in data]])
                    log.write(torobot_id, raw)
                    index += 1
                    push_socket.send_multipart([bytes('cmd', 'ascii'), raw])
                    pending_click = None

                # Crucial: cv2.waitKey handles the window refresh
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
        finally:
            cv2.destroyAllWindows()
            sub_socket.close()
            push_socket.close()
            context.term()


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ip', help='server/robot IP', required=True)
    args = parser.parse_args()
    start_cv_client(args.ip)
