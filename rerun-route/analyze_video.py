import cv2
import os
import subprocess
import tempfile
import hashlib

def get_video_info(log_path):
    with tempfile.NamedTemporaryFile(suffix='.h265', delete=False) as tmp:
        tmp_name = tmp.name
    
    # Extract raw stream using osgar.logger
    cmd = f"uv run python -m osgar.logger {log_path} --raw --stream oak.color"
    with open(tmp_name, 'wb') as f:
        subprocess.run(cmd, shell=True, stdout=f, stderr=subprocess.DEVNULL)
    
    cap = cv2.VideoCapture(tmp_name)
    if not cap.isOpened():
        os.unlink(tmp_name)
        return None

    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    
    # Get hash of the first frame to see if they start at the same place
    ret, frame = cap.read()
    frame_hash = None
    if ret:
        frame_hash = hashlib.md5(frame.tobytes()).hexdigest()
    
    cap.release()
    os.unlink(tmp_name)
    
    return {
        'width': width,
        'height': height,
        'count': count,
        'first_frame_hash': frame_hash
    }

logs = [
    "rerun-route/data/m03-matty-go-cam-260330_180834.log",
    "rerun-route/data/m03-matty-go-cam-260330_180950.log",
    "rerun-route/data/m03-matty-go-cam-260330_181028.log"
]

for log in logs:
    info = get_video_info(log)
    print(f"Log: {log}")
    print(f"  Info: {info}")
