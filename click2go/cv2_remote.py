import cv2
import zmq
import numpy as np

pending_click = None

def mouse_callback(event, x, y, flags, param):
    global pending_click
    if event == cv2.EVENT_LBUTTONDOWN:
        pending_click = f"CV2_Client:{x},{y}"

def start_cv_client():
    global pending_click
    context = zmq.Context()

    sub_socket = context.socket(zmq.SUB)
    # Set a receive timeout (1000ms = 1s)
    sub_socket.setsockopt(zmq.RCVTIMEO, 1000) 
    sub_socket.connect("tcp://localhost:5555")
    sub_socket.setsockopt_string(zmq.SUBSCRIBE, "") 
    
    push_socket = context.socket(zmq.PUSH)
    push_socket.connect("tcp://localhost:5556")

    window_name = "ZMQ Stream (with Timeout)"
    cv2.namedWindow(window_name)
    cv2.setMouseCallback(window_name, mouse_callback)

    print("Client started. Press 'q' to exit.")

    try:
        while True:
            try:
                # This will now only block for 1 second
                jpg_buffer = sub_socket.recv()
                
                nparr = np.frombuffer(jpg_buffer, np.uint8)
                img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                
                if img is not None:
                    cv2.imshow(window_name, img)
                    
            except zmq.Again:
                # This block hits if the 1s timeout is reached
                print("No image received in the last second... waiting.")
                # Optional: display a 'Connection Lost' overlay on the last frame
            
            # This part now runs even if no image was received
            if pending_click:
                push_socket.send_string(pending_click)
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
    start_cv_client()
