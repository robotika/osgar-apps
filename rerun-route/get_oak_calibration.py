import json
import argparse
import sys

import depthai as dai


def get_calibration(output_file):
    print("Connecting to OAK device...")
    try:
        # Connect to device
        with dai.Device() as device:
            print(f"Connected to {device.getDeviceName()}")
            calibData = device.readCalibration()
            
            data = {
                "deviceName": device.getDeviceName(),
                "mxSerial": device.getDeviceId(),
                "cameras": {}
            }
            
            # Camera mapping
            sockets = {
                "rgb": dai.CameraBoardSocket.CAM_A,
                "left": dai.CameraBoardSocket.CAM_B,
                "right": dai.CameraBoardSocket.CAM_C
            }
            
            # Resolutions to fetch intrinsics for
            resolutions = [
                (1920, 1080),
                (1280, 720),
                (640, 400),
                (640, 360)
            ]
            
            for name, socket in sockets.items():
                try:
                    cam_info = {
                        "socket": str(socket),
                        "fov": calibData.getFov(socket),
                        "distortion": calibData.getDistortionCoefficients(socket),
                        "intrinsics": {}
                    }
                    
                    for w, h in resolutions:
                        res_key = f"{w}x{h}"
                        try:
                            cam_info["intrinsics"][res_key] = calibData.getCameraIntrinsics(socket, w, h)
                        except Exception as e:
                            print(f"Warning: Could not get intrinsics for {name} at {res_key}: {e}")
                            
                    data["cameras"][name] = cam_info
                    
                except Exception as e:
                    print(f"Warning: Could not get data for camera {name}: {e}")

            # Extrinsics (Relative to RGB)
            data["extrinsics"] = {}
            for name in ["left", "right"]:
                try:
                    spec = f"rgb_to_{name}"
                    data["extrinsics"][spec] = calibData.getCameraExtrinsics(sockets["rgb"], sockets[name])
                except Exception as e:
                    print(f"Warning: Could not get extrinsics for {spec}: {e}")

            # Save to JSON
            with open(output_file, 'w') as f:
                json.dump(data, f, indent=4)
            
            print(f"Calibration data saved to {output_file}")

    except Exception as e:
        print(f"Error connecting to or reading from OAK device: {e}")
        sys.exit(1)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Read calibration data from OAK device and save to JSON")
    parser.add_argument("--output", "-o", default="oak_calibration.json", help="Output JSON file name")
    args = parser.parse_args()
    
    get_calibration(args.output)
