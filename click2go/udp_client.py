import socket
import struct
import time

# Configuration
SERVER_IP = "127.0.0.1"
SERVER_PORT = 5005
TIMEOUT = 0.1  # 100ms timeout. If no response, assume packet lost/dropped.

def start_client():
    # Create UDP socket
    # We do NOT bind to a specific port, so the OS picks a random free one.
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    # Set timeout for fast failure
    sock.settimeout(TIMEOUT)

    counter = 0

    print(f"Targeting Server: {SERVER_IP}:{SERVER_PORT}")
    print("Press Ctrl+C to stop.")

    try:
        while True:
            # Prepare Payload: Sequence Number (8 bytes) + Timestamp (8 bytes)
            send_time = time.time()
            
            # Pack data: ! = Network Endian, Q = unsigned 8-byte int, d = 8-byte float
            payload = struct.pack('!Q d', counter, send_time)

            try:
                # Send data
                sock.sendto(payload, (SERVER_IP, SERVER_PORT))

                # Wait for response (Echo)
                data, server_addr = sock.recvfrom(1024)
                
                # Unpack response
                recv_seq, recv_time = struct.unpack('!Q d', data[:16])
                
                # Calculate Round Trip Time (RTT)
                rtt = (time.time() - send_time) * 1000  # in ms
                
                if counter % 100 == 0:
                    print(f"Frame {recv_seq}: RTT = {rtt:.3f} ms")

            except socket.timeout:
                # This is critical for "fast response" systems.
                # We don't retry. We accept the loss and move to the next frame.
                print(f"Frame {counter} dropped or request timed out.")
            
            except ConnectionResetError:
                # Common in UDP on Windows if the target port is unreachable
                print("Error: Server not reachable (Connection Reset).")
                time.sleep(1)

            # Increment counter for next packet
            counter += 1
            
            # Throttle slightly to prevent flooding local buffer (simulate 60 FPS approx)
            time.sleep(0.016) 

    except KeyboardInterrupt:
        print("\nClient stopping...")
    finally:
        sock.close()

if __name__ == "__main__":
    start_client()
