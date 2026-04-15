import cv2
import os
import subprocess
import tempfile

def extract_first_frame(log_path, output_name):
    with tempfile.NamedTemporaryFile(suffix='.h265', delete=False) as tmp:
        tmp_name = tmp.name
    
    # Extract raw stream using osgar.logger
    cmd = f"uv run python -m osgar.logger {log_path} --raw --stream oak.color"
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

logs = [
    ("rerun-route/data/m03-matty-go-cam-260330_180834.log", "rerun-route/data/m03-180834.png"),
    ("rerun-route/data/m03-matty-go-cam-260330_180950.log", "rerun-route/data/m03-180950.png"),
    ("rerun-route/data/m03-matty-go-cam-260330_181028.log", "rerun-route/data/m03-181028.png")
]

for log, out in logs:
    if extract_first_frame(log, out):
        print(f"Extracted first frame from {log} to {out}")
    else:
        print(f"Failed to extract first frame from {log}")
