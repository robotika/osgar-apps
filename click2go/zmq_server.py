import zmq, cv2, time

def start_broadcast_server(image_path):
    context = zmq.Context()
    
    # 1. Broadcaster (PUB) - Binds to 5555
    pub_socket = context.socket(zmq.PUB)
    pub_socket.bind("tcp://*:5555")

    # 2. Click Collector (PULL) - Binds to 5556
    pull_socket = context.socket(zmq.PULL)
    pull_socket.bind("tcp://*:5556")

    img = cv2.imread(image_path)
    _, buffer = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 80])
    jpg_bytes = buffer.tobytes()

    print("Broadcasting on 5555, collecting clicks on 5556...")
    
    try:
        while True:
            # Broadcast the frame to ALL subscribers
#            pub_socket.send(jpg_bytes)
            pub_socket.send_multipart([bytes('image', 'ascii'), jpg_bytes])
            
            # Check for clicks from ANY subscriber
            try:
                # Use NOBLOCK to keep the broadcast loop moving
#                click_data = pull_socket.recv_string(flags=zmq.NOBLOCK)
                channel, click_data = pull_socket.recv_multipart(flags=zmq.NOBLOCK)
                print(f"Received: {channel} {click_data}")
            except zmq.Again:
                pass
            
            time.sleep(0.05) # ~20 FPS
    except KeyboardInterrupt:
        print("Server shutting down.")

if __name__ == "__main__":
    start_broadcast_server("test_image.jpg")
