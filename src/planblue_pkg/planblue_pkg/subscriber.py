#!/usr/bin/env python3
import os
import json
from datetime import datetime

import cv2 as cv
import pandas as pd # type: ignore

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class ImageSubscriberSaver(Node):
    def __init__(self):
        super().__init__("image_subscriber_saver")

        # ---- Parameters ----
        self.declare_parameter("topic", "/camera/image_raw")
        self.declare_parameter("output_dir", "subscriber_captures")
        self.declare_parameter("image_folder", "images")  # subfolder for images
        self.declare_parameter("log_folder", "logs")  # subfolder for logs/metadata
        self.declare_parameter("image_format", "png")  # png or jpg/jpeg
        self.declare_parameter("jpeg_quality", 95)

        # Text overlay controls
        self.declare_parameter("text_x", 20)
        self.declare_parameter("text_y", 40)
        self.declare_parameter("text_scale", 1.0)
        self.declare_parameter("text_thickness", 2)

        self.topic = self.get_parameter("topic").value
        output_dir_name = self.get_parameter("output_dir").value

        image_dir_name = self.get_parameter("image_folder").value  
        log_dir_name = self.get_parameter("log_folder").value

        self.image_format = self.get_parameter("image_format").value.lower()
        self.jpeg_quality = int(self.get_parameter("jpeg_quality").value)

        self.text_x = int(self.get_parameter("text_x").value)
        self.text_y = int(self.get_parameter("text_y").value)
        self.text_scale = float(self.get_parameter("text_scale").value)
        self.text_thickness = int(self.get_parameter("text_thickness").value)

        if self.image_format not in ("png", "jpg", "jpeg"):
            raise ValueError("image_format must be 'png' or 'jpg/jpeg'")

        # Save next to this script
        home_dir = os.path.expanduser("~") 
        out_dir  = os.path.join(home_dir, "subscriber_captures")

        # Create a unique folder for this run using timestamp. for future reference purpose
        run_stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        run_folder = f"subscribed_{run_stamp}"
        #Base output dir for this run (contains both images/ and logs/)
        self.base_output_dir = os.path.join(out_dir, output_dir_name, run_folder)
        #Subdirs for images and logs
        self.image_dir_name = os.path.join(self.base_output_dir, image_dir_name)
        self.log_dir_name = os.path.join(self.base_output_dir, log_dir_name)
        os.makedirs(self.image_dir_name, exist_ok=True)
        os.makedirs(self.log_dir_name, exist_ok=True)

        # Per-frame metadata outputs
        self.meta_jsonl_path = os.path.join(self.log_dir_name, f"metadata.jsonl")
        self.csv_path = os.path.join(self.log_dir_name, f"subscriber_log.csv")
        self.xlsx_path = os.path.join(self.log_dir_name, f"subscriber_log.xlsx")

        # ---- Logging rows for CSV/XLSX ----
        self.rows = []
        self.frame_number = 0

        # ---- ROS plumbing (Jazzy MinimalSubscriber style) ----
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            self.topic,
            self.listener_callback,
            qos
        )
        self.subscription  # prevent unused variable warning

        # ---- Log subscription start ----
        start_iso = datetime.now().isoformat(timespec="milliseconds")
        self.rows.append({
            "event": "subscribe_start",
            "frame_number": None,
            "timestamp_iso": start_iso,
            "save_status": True,
            "filename": None,
            "full_path": None,
            "jsonl_write_status": True,
        })

        self.get_logger().info(
            f"[SUBSCRIBER START] {start_iso} | topic={self.topic} | out_dir={self.image_dir_name}"
        )

    def overlay_timestamp(self, frame_bgr, timestamp_text: str):
        # Black outline + white text for readability
        cv.putText(
            frame_bgr, timestamp_text, (self.text_x, self.text_y),
            cv.FONT_HERSHEY_SIMPLEX, self.text_scale, (0, 0, 0),
            self.text_thickness + 2, cv.LINE_AA
        )
        cv.putText(
            frame_bgr, timestamp_text, (self.text_x, self.text_y),
            cv.FONT_HERSHEY_SIMPLEX, self.text_scale, (255, 255, 255),
            self.text_thickness, cv.LINE_AA
        )

    def listener_callback(self, msg: Image):
        self.frame_number += 1

        # Timestamp used for overlay (wall time)
        ts = datetime.now()
        ts_iso = ts.isoformat(timespec="milliseconds")

        # Convert ROS Image -> OpenCV image
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"[ERROR] frame_number={self.frame_number} timestamp={ts_iso} | imgmsg_to_cv2 failed: {e}")
            self.rows.append({
                "event": "frame_save",
                "frame_number": self.frame_number,
                "timestamp": ts_iso,
                "save_status": False,
                "filename": None,
                "full_path": None,
                "jsonl_write_status": False,
            })
            return

        # Overlay timestamp
        try:
            self.overlay_timestamp(frame, ts_iso)
        except Exception as e:
            self.get_logger().error(f"[ERROR] frame_number={self.frame_number} timestamp={ts_iso} | overlay failed: {e}")
            self.rows.append({
                "event": "frame_save",
                "frame_number": self.frame_number,
                "timestamp": ts_iso,
                "save_status": False,
                "filename": None,
                "full_path": None,
                "jsonl_write_status": False,
            })
            return

        # Build output filename/path
        ext = "jpg" if self.image_format in ("jpg", "jpeg") else "png"
        filename = f"sub_{self.frame_number:06d}_{ts.strftime('%Y%m%d_%H%M%S_%f')}.{ext}"
        filepath = os.path.join(self.image_dir_name, filename)

        # Save image
        try:
            if ext == "jpg":
                ok = cv.imwrite(filepath, frame, [int(cv.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
            else:
                ok = cv.imwrite(filepath, frame)
        except Exception as e:
            self.get_logger().error(f"[ERROR] frame_number={self.frame_number} timestamp={ts_iso}  cv.imwrite exception: {e}")
            ok = False

        if not ok:
            self.get_logger().error(f"[ERROR] frame_number={self.frame_number} timestamp={ts_iso}  failed to save image: {filepath}")
            self.rows.append({
                "event": "frame_save",
                "frame_number": self.frame_number,
                "timestamp": ts_iso,
                "save_status": False,
                "filename": filename,
                "full_path": filepath,
                "jsonl_write_status": False,
            })
            return

        # Write JSONL metadata
        meta = {"filename": filename, "timestamp": ts_iso}
        jsonl_ok = True
        try:
            with open(self.meta_jsonl_path, "a", encoding="utf-8") as f:
                f.write(json.dumps(meta) + "\n")
        except Exception as e:
            jsonl_ok = False
            self.get_logger().error(f"[ERROR] frame_number={self.frame_number} timestamp={ts_iso}   metadata JSONL write failed: {e}")

        # Store row for CSV/XLSX
        self.rows.append({
            "event": "frame_save",
            "frame_number": self.frame_number,
            "timestamp": ts_iso,
            "save_status": True,
            "filename": filename,
            "full_path": filepath,
            "jsonl_write_status": jsonl_ok,
        })

        # Console log (required)
        self.get_logger().info(f"[SAVED] frame_number={self.frame_number} timestamp={ts_iso}   path={filepath} status={'SUCCESS' if ok else 'FAIL'}")

    def destroy_node(self):
        # Add an explicit end row
        end_iso = datetime.now().isoformat(timespec="milliseconds")
        self.rows.append({
            "event": "subscribe_end",
            "frame_number": self.frame_number,
            "timestamp": end_iso,
            "save_status": True,
            "filename": None,
            "full_path": None,
            "jsonl_write_status": True,
        })

        # Write CSV + XLSX (like publisher)
        try:
            df = pd.DataFrame(self.rows)
            df.to_csv(self.csv_path, index=False)
            df.to_excel(self.xlsx_path, index=False)  # requires openpyxl
            print(f"[image_subscriber_saver] Excel saved to: {self.xlsx_path}")
            print(f"[image_subscriber_saver] CSV saved to: {self.csv_path}")
        except Exception as e:
            print(f"[image_subscriber_saver][WARN] Failed to write CSV/XLSX logs: {e}")

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    image_subscriber = None
    try:
        image_subscriber = ImageSubscriberSaver()
        rclpy.spin(image_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        if image_subscriber is not None:
            image_subscriber.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

  


if __name__ == "__main__":
    main()
