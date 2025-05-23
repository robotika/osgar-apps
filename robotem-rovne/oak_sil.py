import cv2
import os
import numpy as np
import depthai as dai
import argparse
import pathlib

WIDTH = 160
HEIGHT = 120

def load_images_from_folder(folder):
    """Load all images from the folder."""
    images = []
    for filename in os.listdir(folder):
        img = cv2.imread(os.path.join(folder, filename))
        if img is not None:
            img = cv2.resize(img, (640, 480))
            images.append(img)
    return images

def process_image(image, nn_pipeline):
    """Process the image as if it was captured by the OAK-D camera."""
    width = WIDTH
    height = HEIGHT
    resized_image = cv2.resize(image, (width, height))
    img_frame = dai.ImgFrame()
    img_frame.setData(to_planar(resized_image, (width, height)))
    img_frame.setType(dai.ImgFrame.Type.BGR888p)
    img_frame.setWidth(width)
    img_frame.setHeight(height)
    q_nn_input.send(img_frame)
    in_nn = q_nn_output.get()
    return in_nn.getLayerFp16('output')
        

def to_planar(arr: np.ndarray, shape=(320, 240)):
    return cv2.resize(arr, shape).transpose(2,0,1).flatten()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Run sengmentation model on real Oak-D camera using replay of recorded images')
    parser.add_argument('--model', type=pathlib.Path, required=True,
                            help='Path to blob model')
    parser.add_argument('--images', type=pathlib.Path, required=True,
                            help='Path to a folder with images')
    args = parser.parse_args()
    print("Press any key to process next image...")
    images = load_images_from_folder(args.images)

    # Setup DepthAI pipeline
    pipeline = dai.Pipeline()
    cam_rgb = pipeline.create(dai.node.ColorCamera)
    nn = pipeline.create(dai.node.NeuralNetwork)
    nn.setBlobPath(args.model)

    # Define input and output queues
    q_nn_input = pipeline.create(dai.node.XLinkIn)
    q_nn_output = pipeline.create(dai.node.XLinkOut)
    q_nn_input.setStreamName("nn_input")
    q_nn_output.setStreamName("nn_output")
    q_nn_input.out.link(nn.input)
    nn.out.link(q_nn_output.input)

    # Connect to the device and start pipeline
    with dai.Device(pipeline) as device:
        # Get input and output queues
        q_nn_input = device.getInputQueue("nn_input")
        q_nn_output = device.getOutputQueue("nn_output")

        for img in images:
            # Process each image
            nn_output = process_image(img, pipeline)
            mask = np.array(nn_output).reshape((2, HEIGHT, WIDTH))
            # Overlay the mask on the frame
            mask = mask.argmax(0).astype(np.uint8)
            #mask = mask[0].astype(np.uint8)
            mask = cv2.resize(mask, (640, 480))
            height, width = mask.shape
            colored_mask = np.zeros((height, width, 3), dtype=np.uint8)
            colored_mask[mask == 1] = [0, 0, 255]
            overlay = cv2.addWeighted(img, 1, colored_mask, 0.7, 0)
            
            cv2.imshow("OAK-D Segmentation", overlay)
            cv2.waitKey(0)
