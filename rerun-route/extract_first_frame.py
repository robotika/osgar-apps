import cv2
import os
import subprocess
import tempfile
import sys

def extract_first_frame(log_path, output_name):
    with tempfile.NamedTemporaryFile(suffix='.h265', delete=False) as tmp:
        tmp_name = tmp.name
    
    # Extract raw stream using osgar.logger
    # Use sys.executable to ensure we use the same python interpreter
    cmd = f'"{sys.executable}" -m osgar.logger {log_path} --raw --stream oak.color'
    with open(tmp_name, 'wb') as f:
        subprocess.run(cmd, shell=True, stdout=f, stderr=subprocess.DEVNULL)
    
    cap = cv2.VideoCapture(tmp_name)
    if not cap.isOpened():
        os.unlink(tmp_name)
        return False

    ret, frame = cap.read()
    if ret:
        cv2.imwrite(output_name, frame)
    
    cap.release()
    os.unlink(tmp_name)
    return ret

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: extract_first_frame.py <logfile> <output_image>")
        sys.exit(1)
    
    log = sys.argv[1]
    out = sys.argv[2]
    if extract_first_frame(log, out):
        print(f"Extracted first frame from {log} to {out}")
    else:
        print(f"Failed to extract first frame from {log}")
