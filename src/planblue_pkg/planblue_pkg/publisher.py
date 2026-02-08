#!/usr/bin/env python3
import os
from datetime import datetime
import cv2 as cv
import pandas as pd
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class CameraPublisher(Node):
    def __init__(self):
        super().__init__("camera_publisher")

        # ---- Parameters (defaults) ----
        self.declare_parameter("device", "/dev/video0")
        self.declare_parameter("fps", 5.0)
        self.declare_parameter("width", 1280)
        self.declare_parameter("height", 720)
        self.declare_parameter("topic", "/camera/image_raw")
        self.declare_parameter("folder_name", "publisher_captures")
        self.declare_parameter("save_images", False)

        self.device = self.get_parameter("device").value
        self.fps = float(self.get_parameter("fps").value)
        self.width = int(self.get_parameter("width").value)
        self.height = int(self.get_parameter("height").value)
        self.topic = self.get_parameter("topic").value
        folder_name = self.get_parameter("folder_name").value
        self.save_images = bool(self.get_parameter("save_images").value)

        ''' ---- Output paths ---- use these code lines if you want to save next to this script
        script_dir = os.path.dirname(os.path.abspath(__file__))
        self.out_dir = os.path.join(script_dir, self.folder_name)
        os.makedirs(self.out_dir, exist_ok=True)'''


        home_dir = os.path.expanduser("~") 
        out_dir  = os.path.join(home_dir, "publisher_captures")
        
        # Create a unique folder for this run using timestamp. for future reference purpose
        run_stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        run_folder = f"published_{run_stamp}"
       
        
        self.out_dir = os.path.join(out_dir, folder_name, run_folder)
        os.makedirs(self.out_dir, exist_ok=True)

        self.xlsx_path = os.path.join(self.out_dir, f"publish_log.xlsx")
        self.csv_path = os.path.join(self.out_dir, f"publish_log.csv")

        # ---- Publisher ----
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.publisher_ = self.create_publisher(Image, self.topic, qos)
        self.bridge = CvBridge()

        # ---- Camera ----
        self.cap = cv.VideoCapture(self.device, cv.CAP_V4L2)
        if not self.cap.isOpened():
            raise RuntimeError(
                f"Could not open camera device {self.device}. "
                f"Try /dev/video0 or /dev/video1 and ensure permissions."
            )
        self.cap.set(cv.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv.CAP_PROP_FRAME_HEIGHT, self.height)

        # ---- Logging ----
        self.rows = []
        self.i = 0

        start_iso = datetime.now().isoformat(timespec="milliseconds")
        self.rows.append({
            "event": "publish_start",
            "frame_number": None,
            "timestamp": start_iso,
            "publish_status": True,
            "filename": None,
            "full_path": None,
        })

        self.get_logger().info(
            f"Publishing started: {start_iso} | device={self.device} | topic={self.topic} | fps={self.fps} "
            f"(Press Ctrl+C to stop)"
        )

        # ---- Timer (like Jazzy example) ----
        timer_period = 1.0 / self.fps if self.fps > 0 else 0.2
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        # Capture frame
        ret, frame = self.cap.read()
        ts_iso = datetime.now().isoformat(timespec="milliseconds")

        if not ret or frame is None:
            self.rows.append({
                "event": "frame_publish",
                "frame_number": self.i + 1,
                "timestamp": ts_iso,
                "publish_status": False,
                "filename": None,
                "full_path": None,
            })
            self.get_logger().warn(f'Publishing: frame_number={self.i + 1} timestamp={ts_iso} status=FAIL (camera read failed)')
            return

        # Increment counter (like example)
        self.i += 1

        # Publish
        publish_ok = True
        try:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera"
            self.publisher_.publish(msg)
        except Exception as e:
            publish_ok = False
            self.get_logger().error(f"Publish failed: {e}")

        # Optional: save raw frames from publisher
        filename = None
        filepath = None
        if self.save_images:
            ts = datetime.now()
            filename = f"frame_{self.i:06d}_{ts.strftime('%Y%m%d_%H%M%S_%f')}.png"
            filepath = os.path.join(self.out_dir, filename)
            _ = cv.imwrite(filepath, frame)

        # Log row
        self.rows.append({
            "event": "frame_publish",
            "frame_number": self.i,
            "timestamp": ts_iso,
            "publish_status": publish_ok,
            "filename": filename,
            "full_path": filepath,
        })

        self.get_logger().info(
            f'Publishing: frame_number={self.i} timestamp={ts_iso} status={"SUCCESS" if publish_ok else "FAIL"}'
        )

    def destroy_node(self):
        end_iso = datetime.now().isoformat(timespec="milliseconds")
        self.rows.append({
           "event": "publish_end",
           "frame_number": self.i,
           "timestamp": end_iso,
           "publish_status": True,
           "filename": None,
           "full_path": None,
        })
        print(f"[camera_publisher] Publishing stopped: {end_iso} | frames_published={self.i}")
    
        try:
            if self.cap is not None:
                self.cap.release()
        except Exception:
                    pass
                
                
        try:
            df = pd.DataFrame(self.rows)
            df.to_csv(self.csv_path, index=False)
            df.to_excel(self.xlsx_path, index=False)
            print(f"[camera_publisher] Log saved to: {self.xlsx_path}")
            print(f"[camera_publisher] CSV saved to: {self.csv_path}")
        except Exception as e:
            print(f"[camera_publisher][WARN] Failed to write logs: {e}")
                
            super().destroy_node()

def main(args=None):
    rclpy.init(args=args)

    camera_publisher = CameraPublisher()

    try:
        rclpy.spin(camera_publisher)
    except KeyboardInterrupt:
        pass

    # Destroy the node explicitly (like Jazzy example)
    camera_publisher.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()




#Refrences:
#ROS2 AND OpenCV: cv-bridge, link: https://github.com/ros-perception/vision_opencv/tree/rolling/cv_bridge
