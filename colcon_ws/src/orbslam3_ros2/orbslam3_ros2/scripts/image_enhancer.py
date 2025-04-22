#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from enlighten_inference import EnlightenOnnxModel
import keras
from keras import layers

#from model import Finetunemodel
# from PIL import Image
import numpy as np
import onnxruntime as ort
import torch
import torch.onnx
import torchvision.transforms as transforms

# Note: 'keras<3.x' or 'tf_keras' must be installed (legacy)
# See https://github.com/keras-team/tf-keras for more details.
from huggingface_hub import from_pretrained_keras

#cv2.namedWindow("Display window", cv2.WINDOW_NORMAL)
#model = from_pretrained_keras("keras-io/lowlight-enhance-mirnet")


def sharpen_image(image):
    # Define a simple sharpening kernel
    kernel = np.array([
        [-1, -1, -1],
        [-1,  4, -1],
        [-1, -1, -1]
    ])

    # Apply the kernel to the image using cv2.filter2D
    sharpened_image = cv2.filter2D(image, -1, kernel)

    return sharpened_image

# def infer(original_image):
#     # original_image is expected to be a NumPy array (e.g., from cv2.imread or cv_bridge)
#     original_height, original_width = original_image.shape[:2]

#     # Preprocess image
#     image = original_image.astype("float32") / 255.0
#     image = np.expand_dims(image, axis=0)

#     # Model inference
#     output = model.predict(image, verbose=0)[0] * 255.0
#     output = np.clip(output, 0, 255).astype("uint8")

#     # Resize to original size if needed
#     output_resized = cv2.resize(output, (original_width, original_height), interpolation=cv2.INTER_CUBIC)

#     return output_resized

def enhance_image_GAN(image):
    # Apply your enhancement model here
    model = EnlightenOnnxModel()
    model = EnlightenOnnxModel(providers = ["CUDAExecutionProvider"])
    #model = EnlightenOnnxModel(providers = ["CPUExecutionProvider"])
    processed = model.predict(image)
    return processed
        

def enhance_image_ll(image):
    # Store the original dimensions
    original_height, original_width = image.shape[:2]

    # Resize the input to the model's expected input size (e.g., 1000x1000)
    image_resized = cv2.resize(image, (1000, 1000), interpolation=cv2.INTER_CUBIC)  # adjust based on model's expected size
    image_rgb = cv2.cvtColor(image_resized, cv2.COLOR_BGR2RGB)

    # Convert to float32 and normalize
    image_float = image_rgb.astype(np.float32) / 255.0
    image_transposed = np.transpose(image_float, (2, 0, 1))  # (H, W, C) -> (C, H, W)
    image_batched = np.expand_dims(image_transposed, axis=0)  # Add batch dimension

    # Load the ONNX model and run inference
    ort_session = ort.InferenceSession("/home/kaushek/MRob/colcon_ws/src/orbslam3_ros2/onnx/model.onnx")
    output = ort_session.run(None, {"input": image_batched})[0]  # Assuming single output

    # Post-process the output: (B, C, H, W) -> (H, W, C)
    output_image = np.transpose(output[0], (1, 2, 0)) * 255.0  # Convert from [0, 1] to [0, 255]
    output_image = np.clip(output_image, 0, 255).astype(np.uint8)

    # Resize the output image back to the original size
    output_resized = cv2.resize(output_image, (original_width, original_height))

    # Convert back to BGR format
    output_bgr = cv2.cvtColor(output_resized, cv2.COLOR_RGB2BGR)
    
    # Show the image
    result_img = sharpen_image(output_bgr)
    return result_img


class ImageEnhancer(Node):
    def __init__(self):
        super().__init__('image_enhancer')
        self.bridge = CvBridge()
        # self.subscription = self.create_subscription(
        #     Image,
        #     '/camera/image_raw',
        #     self.listener_callback,
        #     1)
        self.subscription = self.create_subscription(
            Image,
            '/bcr_bot/kinect_camera/image_raw',
            self.listener_callback,
            1)
        
        self.publisher_ = self.create_publisher(Image, '/bcr_bot/kinect_camera/image_enhanced', 10)

    def listener_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        #enhanced_image = enhance_image_GAN(cv_image)
        enhanced_image = enhance_image_ll(cv_image)
        enhanced_msg = self.bridge.cv2_to_imgmsg(enhanced_image, encoding='bgr8')
        self.publisher_.publish(enhanced_msg)

def main(args=None):
    rclpy.init(args=args)
    image_enhancer = ImageEnhancer()
    rclpy.spin(image_enhancer)
    image_enhancer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
