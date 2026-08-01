#!/usr/bin/env python3
# Copyright (c) 2025-present WATonomous. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""
Load a YOLO model (ONNX by default, optional .pt via torch)
subscribe to /image topic and log detection results
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from cv_bridge import CvBridge
import numpy as np
import cv2
import os
import onnxruntime as ort  # rosdep: python3-onnxruntime
from ament_index_python.packages import get_package_share_directory

try:
    import torch  # rosdep: python3-torch (optional)

    TORCH_OK = True
except ImportError:
    TORCH_OK = False


class YoloInference(Node):
    def __init__(self):
        super().__init__("yolo_inference_node")

        pkg_share = get_package_share_directory("yolo_inference")
        default_model = os.path.join(pkg_share, "models", "yolov8.onnx")

        self.declare_parameter("model_path", default_model)
        self.declare_parameter("input_size", 640)

        self.model_path: str = (
            self.get_parameter("model_path").get_parameter_value().string_value
        )
        self.input_size: int = (
            self.get_parameter("input_size").get_parameter_value().integer_value
        )

        if not self.model_path:
            self.get_logger().fatal("Parameter 'model_path' is required.")
            raise RuntimeError("model_path parameter unset")

        # Load model
        ext = os.path.splitext(self.model_path)[1]
        if ext == ".onnx":
            self.session = ort.InferenceSession(
                self.model_path,
                providers=["CPUExecutionProvider"],
            )
            self.input_name = self.session.get_inputs()[0].name
            self.get_logger().info(f"Loaded ONNX model: {self.model_path}")
            self.run = self.run_onnx
        elif ext in (".pt", ".pth"):
            if not TORCH_OK:
                raise RuntimeError(".pt model requested but 'torch' not available")
            self.model = torch.jit.load(self.model_path).eval()
            self.get_logger().info(f"Loaded TorchScript model: {self.model_path}")
            self.run = self.run_torch
        else:
            raise RuntimeError(f"Unsupported model extension: {ext}")

        self.annotated_pub = self.create_publisher(Image, "/detections_image", 10)
        self.detections_pub = self.create_publisher(
            Detection2DArray, "/yolo_detections", 10
        )

        # Bridge & subscription
        self.bridge = CvBridge()
        self.create_subscription(Image, "/image", self.cb_image, 10)

    ## inference
    def run_onnx(self, img: np.ndarray):
        inp = self.preprocess(img)
        preds = self.session.run(None, {self.input_name: inp})[0]
        return self.postprocess(preds)

    def run_torch(self, img: np.ndarray):
        inp = self.preprocess(img, torch_tensor=True)
        with torch.no_grad():
            preds = self.model(inp)[0].cpu().numpy()
        return self.postprocess(preds)

    ## ros callback
    def cb_image(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        detections = self.run(frame)

        for x1, y1, x2, y2, conf, cls in detections:
            cv2.rectangle(frame, (x1, y1), (x2, y2), (128, 0, 128), 1)
            cv2.putText(
                frame,
                f"{cls}: {conf:.2f}",
                (x1, y1 - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (128, 0, 128),
                1,
            )
            self.get_logger().info(
                f"Found object {cls} at ({x1}, {y1}), ({x2}, {y2}) with {conf:.2f} confidence"
            )

        # Publish the annotated image
        annotated_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.annotated_pub.publish(annotated_msg)

        # Published detection data
        detections_msg = Detection2DArray()  # create a message
        detections_msg.header = msg.header

        for x1, y1, x2, y2, conf, cls in detections:
            # turn into x,y,w,h format
            x = (x1 + x2) / 2
            y = (y1 + y2) / 2
            w = x2 - x1
            h = y2 - y1

            # set bbox
            detection = Detection2D() # create message
            detection.bbox.center.x = x
            detection.bbox.center.y = y
            detection.bbox.size_x = w
            detection.bbox.size_y = h

            # set class and confidence
            hypothesis = ObjectHypothesis() # create message
            hypothesis.id = cls
            hypothesis.score = conf
            detection.results.append(hypothesis)

            detections_msg.detections.append(detection)

        self.detections_pub.publish(detections_msg)

        self.get_logger().info(f"Detected {len(detections)} objects")

    ## helpers
    def letterbox(self, im, new_shape=(640, 640), color=(114, 114, 114)):
        """
        Resize image to fit in a target size with unchanged aspect ratio using padding.

        Args:
            img (np.ndarray): Input image (HWC format).
            new_shape (tuple): Desired output shape (height, width).
            color (tuple): Padding color in (B, G, R) format.

        Returns:
            np.ndarray: Resized and padded image of shape new_shape.
        """
        # calculate new dimensions
        shape = im.shape[:2]  # current height, width
        scale_ratio = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
        new_dims = (
            int(round(shape[1] * scale_ratio)),
            int(round(shape[0] * scale_ratio)),
        )

        # calculate new position
        dw, dh = (new_shape[1] - new_dims[0]) / 2, (new_shape[0] - new_dims[1]) / 2
        top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
        left, right = int(round(dw - 0.1)), int(round(dw + 0.1))

        # create new scaled image
        im = cv2.resize(im, new_dims, interpolation=cv2.INTER_LINEAR)
        im = cv2.copyMakeBorder(
            im, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color
        )

        return im, scale_ratio, dw, dh

    def preprocess(self, img: np.ndarray, torch_tensor=False):
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        # use a letterbox resizing function to scale image to 640x640 while preserving aspect ratio
        img, self.ratio, self.dw, self.dh = self.letterbox(
            img, new_shape=(self.input_size, self.input_size)
        )
        img = img.astype(np.float32) / 255.0
        img = np.transpose(img, (2, 0, 1))  # HWC to CHW
        img = np.expand_dims(img, 0)  # NCHW
        img = np.ascontiguousarray(img)
        if torch_tensor:
            import torch

            return torch.from_numpy(img)
        return img

    def postprocess(self, raw: np.ndarray):
        """
        YOLOv8 output should be already postprocessed now ...
        Format: [x, y, w, h, conf, class_id]
        """
        conf_threshold = 0.25
        results = []

        predictions = raw[0]
        for det in predictions:
            # filter results
            conf = det[4]
            if conf < conf_threshold:
                continue

            # scale model results from letterbox to original
            x1 = int((det[0] - self.dw) / self.ratio)
            y1 = int((det[1] - self.dh) / self.ratio)
            x2 = int((det[2] - self.dw) / self.ratio)
            y2 = int((det[3] - self.dh) / self.ratio)

            cls = "Unknown"
            if int(det[5]) < 2:
                cls = ["Mallet", "Bottle"][int(det[5])]

            results.append((x1, y1, x2, y2, float(conf), cls))

        return results


def main(args=None):
    rclpy.init(args=args)
    try:
        node = YoloInference()
        rclpy.spin(node)
    except RuntimeError as e:
        rclpy.get_logger("yolo_inference_node").error(str(e))
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
