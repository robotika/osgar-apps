import socket
import struct

# Configuration
SERVER_IP = "127.0.0.1"
SERVER_PORT = 5005
BUFFER_SIZE = 1024

def start_server():
    # Create a UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    # Bind to the specific IP and Port
    try:
        sock.bind((SERVER_IP, SERVER_PORT))
        print(f"Server listening on {SERVER_IP}:{SERVER_PORT}")
    except OSError as e:
        print(f"Error binding to port: {e}")
        return

    expected_seq = None

    while True:
        try:
            # Receive data and address of the client
            data, addr = sock.recvfrom(BUFFER_SIZE)
            
            # Unpack the payload
            # '!Q d' = Network Endian (!), Unsigned Long Long (Q), Double (d)
            # This expects 16 bytes: 8 for counter, 8 for timestamp
            if len(data) < 16:
                continue # Ignore malformed packets

            seq_num, client_timestamp = struct.unpack('!Q d', data[:16])

            # Frame Drop Detection Logic
            if expected_seq is not None:
                if seq_num > expected_seq:
                    lost_frames = seq_num - expected_seq
                    print(f"⚠ LOST {lost_frames} frame(s) | Previous: {expected_seq-1} -> Current: {seq_num}")
                elif seq_num < expected_seq - 1:
                    print(f"⚠ Late/Out-of-order packet received: {seq_num}")

            # Update expected sequence (we expect the next one to be current + 1)
            expected_seq = seq_num + 1

            # Processing simulation (fast response required, so we keep this minimal)
            # Send the same data back as an acknowledgement (Echo)
            sock.sendto(data, addr)
            
            # Optional: Print periodically rather than every frame to save I/O time
            if seq_num % 100 == 0:
                print(f"Received frame {seq_num} from {addr}")

        except KeyboardInterrupt:
            print("\nServer stopping...")
            break
        except Exception as e:
            print(f"Error: {e}")

    sock.close()

if __name__ == "__main__":
    start_server()
