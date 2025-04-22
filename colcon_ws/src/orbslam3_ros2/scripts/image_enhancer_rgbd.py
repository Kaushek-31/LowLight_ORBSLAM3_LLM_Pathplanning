#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from message_filters import Subscriber, ApproximateTimeSynchronizer
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
        [0, -1, 0],
        [-1,  5, -1],
        [0, -1, 0]
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

def load_images_transform(file):
    # Load and transform the image
    transform = transforms.Compose([transforms.ToTensor()])
    im = Image.open(file).convert("RGB")
    img_tensor = transform(im)

    # Convert to numpy and add a batch dimension
    img_numpy = img_tensor.numpy().astype(np.float32)
    img_batch = np.expand_dims(img_numpy, 0)

    return torch.from_numpy(img_batch)

def enhance_image_ll(image):
    # Store original size
    original_height, original_width = image.shape[:2]

    # Convert OpenCV BGR to RGB and normalize
    image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    image_float = image_rgb.astype(np.float32) / 255.0

    # Convert to (C, H, W) and add batch dimension
    image_transposed = np.transpose(image_float, (2, 0, 1))
    image_batched = np.expand_dims(image_transposed, axis=0).astype(np.float32)

    # Load ONNX model
    ort_session = ort.InferenceSession("/home/colcon_ws/src/orbslam3_ros2/onnx/model.onnx")

    # Run inference
    output = ort_session.run(None, {"input": image_batched})[0]

    # Post-process output: (1, C, H, W) → (H, W, C)
    output_image = np.transpose(output[0], (1, 2, 0)) * 255.0
    output_image = np.clip(output_image, 0, 255).astype(np.uint8)

    # Resize to original resolution
    output_resized = cv2.resize(output_image, (original_width, original_height), interpolation=cv2.INTER_CUBIC)

    # Convert RGB back to BGR for OpenCV
    output_bgr = cv2.cvtColor(output_resized, cv2.COLOR_RGB2BGR)

    # Optional post-filtering (sharpen, etc.)
    result_img = sharpen_image(output_bgr)
    return result_img


class ImageEnhancer(Node):
    def __init__(self):
        super().__init__('image_enhancer')
        self.bridge = CvBridge()
        # Create message_filters Subscribers
        self.rgb_sub = Subscriber(self, Image, '/bcr_bot/kinect_camera/image_raw')
        self.depth_sub = Subscriber(self, Image, '/bcr_bot/kinect_camera/depth/image_raw')

        # Approximate Time Synchronizer
        self.sync = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub],
            queue_size=10,
            slop=0.1  # You can adjust the slop value for time tolerance
        )
        self.sync.registerCallback(self.listener_callback)
        self.publisher_rgb_ = self.create_publisher(Image, '/bcr_bot/kinect_camera/image_enhanced', 10)
        self.publisher_depth_ = self.create_publisher(Image, '/bcr_bot/kinect_camera/depth/image_enhanced', 10)

    def listener_callback(self, rgb_msg, depth_msg):
        cv_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
        #enhanced_image = enhance_image_GAN(cv_image)
        enhanced_image = enhance_image_ll(cv_image)
        enhanced_msg = self.bridge.cv2_to_imgmsg(enhanced_image, encoding='bgr8')
        # Copy the header (including timestamp and frame_id) from the original message
        enhanced_msg.header = rgb_msg.header
        depth_msg.header = rgb_msg.header
        # publishing depth image as well
        self.publisher_depth_.publish(depth_msg)
        self.publisher_rgb_.publish(enhanced_msg)

def main(args=None):
    rclpy.init(args=args)
    image_enhancer = ImageEnhancer()
    rclpy.spin(image_enhancer)
    image_enhancer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
