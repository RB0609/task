#!/usr/bin/env python3
import os
import json
from datetime import datetime
import cv2 as cv
import pandas as pd 
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class ImageSubscriberSaver(Node):
    def __init__(self):
        super().__init__("image_subscriber_saver")

        #-----TO DECLARE ROS2 PARAMETERS-----
        self.declare_parameter("topic", "/camera/image_raw")         #ROS topic to subscribe to for images
        self.declare_parameter("output_dir", "subscriber_captures")  #Base output folder name (will be created in user's home directory)
        self.declare_parameter("image_folder", "images")             #subfolder for images
        self.declare_parameter("log_folder", "logs")                 #subfolder for logs/metadata
        self.declare_parameter("image_format", "png")                #png or jpg/jpeg
        self.declare_parameter("jpeg_quality", 95)                   #JPEG quality (1-100, higher is better quality but larger file size)
        self.declare_parameter("text_x", 20)                         #X coordinate for timestamp overlay text
        self.declare_parameter("text_y", 40)                         #Y coordinate for timestamp overlay text
        self.declare_parameter("text_scale", 1.0)                    #Font scale for timestamp overlay text
        self.declare_parameter("text_thickness", 2)                  #Thickness for timestamp overlay text
        #----- TO READ THE VALUE AND STORE IT IN A VARIABLE-----
        self.topic = self.get_parameter("topic").value
        self.output_dir_name = self.get_parameter("output_dir").value
        self.image_dir_name = self.get_parameter("image_folder").value  
        self.log_dir_name = self.get_parameter("log_folder").value
        self.image_format = self.get_parameter("image_format").value.lower()
        self.jpeg_quality = int(self.get_parameter("jpeg_quality").value)
        self.text_x = int(self.get_parameter("text_x").value)
        self.text_y = int(self.get_parameter("text_y").value)
        self.text_scale = float(self.get_parameter("text_scale").value)
        self.text_thickness = int(self.get_parameter("text_thickness").value)

        if self.image_format not in ("png", "jpg", "jpeg"):
            raise ValueError("image_format must be 'png' or 'jpg/jpeg'")

        # Save the files in the user's home directory
        home_dir = os.path.expanduser("~")                        #Get the user's home directory (e.g., /home/username)
        out_dir  = os.path.join(home_dir, "subscriber_captures")  #Base output directory (e.g., /home/username/subscriber_captures)

        # Create a unique folder for this run using timestamp. for future reference purpose
        run_stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        run_folder = f"subscribed_{run_stamp}"
        #Base output dir for this run (contains both images/ and logs/)
        self.base_output_dir = os.path.join(out_dir, self.output_dir_name, run_folder)
        #Sub-folders for images and logs
        self.image_dir_name = os.path.join(self.base_output_dir, self.image_dir_name)
        self.log_dir_name = os.path.join(self.base_output_dir, self.log_dir_name)
        os.makedirs(self.image_dir_name, exist_ok=True)
        os.makedirs(self.log_dir_name, exist_ok=True)

        # Per-frame metadata outputs
        self.meta_jsonl_path = os.path.join(self.log_dir_name, f"metadata.jsonl")  #JSON Lines file for metadata (one JSON object per line)
        self.csv_path = os.path.join(self.log_dir_name, f"subscriber_log.csv")     #CSV file.
        self.xlsx_path = os.path.join(self.log_dir_name, f"subscriber_log.xlsx")   #Excel file.

        # ---- Logging rows for CSV/XLSX ----
        self.rows = []
        self.frame_number = 0

        #-----ROS2 SUBSCRIBER SETUP-----
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

        #-----lOGGING THE START OF SUBSCRIBER-----
        start_iso = datetime.now().isoformat(timespec="milliseconds")
        self.rows.append(
            {
               "event": "subscribe_start",
               "frame_number": None,
               "timestamp_iso": start_iso,
               "save_status": True,
               "filename": None,
               "full_path": None,
               "jsonl_write_status": True,
           }
        )
        #-----DISPLAY THE LOG INFO ON THE CONSOLE-----
        self.get_logger().info(
            f"[SUBSCRIBER START] {start_iso} | topic={self.topic} | out_dir={self.image_dir_name}"
        )
    #-----FUNCTION TO OVERLAY TIMESTAMP ON THE IMAGE FRAME-----
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
        '''
        Callback function that gets called every time a new image message is received on the subscribed topic.
        '''
        self.frame_number += 1

        # Timestamp used for overlay (wall time)
        ts = datetime.now()
        ts_iso = ts.isoformat(timespec="milliseconds")

        # Convert ROS Image -> OpenCV image
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            # Log the error and display it on the console, but continue running to process future frames
            self.get_logger().error(f"[ERROR] frame_number={self.frame_number} timestamp={ts_iso} | imgmsg_to_cv2 failed: {e}")
            self.rows.append(
                {
                   "event": "frame_save",
                   "frame_number": self.frame_number,
                   "timestamp": ts_iso,
                   "save_status": False,
                   "filename": None,
                   "full_path": None,
                   "jsonl_write_status": False,
               }
            )
            return

        #-----OVERLAY TIMESTAMP ON THE IMAGE FRAME-----
        try:
            self.overlay_timestamp(frame, ts_iso)
        except Exception as e:
            self.get_logger().error(f"[ERROR] frame_number={self.frame_number} timestamp={ts_iso} | overlay failed: {e}")
            self.rows.append(
                {
                   "event": "frame_save",
                   "frame_number": self.frame_number,
                   "timestamp": ts_iso,
                   "save_status": False,
                   "filename": None,
                   "full_path": None,
                   "jsonl_write_status": False,
               }
            )
            return

        #-----BUILD THE FILENAME AND FILEPATH-----
        ext = "jpg" if self.image_format in ("jpg", "jpeg") else "png"
        filename = f"sub_{self.frame_number:06d}_{ts.strftime('%Y%m%d_%H%M%S_%f')}.{ext}"
        filepath = os.path.join(self.image_dir_name, filename)

        #-----SAVE THE IMAGE TO DISK-----
        try:
            if ext == "jpg":
                ok = cv.imwrite(filepath, frame, [int(cv.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
            else:
                ok = cv.imwrite(filepath, frame)
        except Exception as e:
            self.get_logger().error(f"[ERROR] frame_number={self.frame_number} timestamp={ts_iso}  cv.imwrite exception: {e}")
            ok = False

        if not ok:
            # Log the error and display it on the console.
            self.get_logger().error(f"[ERROR] frame_number={self.frame_number} timestamp={ts_iso}  failed to save image: {filepath}")
            self.rows.append(
                {
                   "event": "frame_save",
                   "frame_number": self.frame_number,
                   "timestamp": ts_iso,
                   "save_status": False,
                   "filename": filename,
                   "full_path": filepath,
                   "jsonl_write_status": False,
                }
            )
            return

        meta = {"frame_number": self.frame_number, "filename": filename, "timestamp": ts_iso, "status": "SUCCESS"}  #Metadata to store in JSONL.
        jsonl_ok = True
        try:
            with open(self.meta_jsonl_path, "a", encoding="utf-8") as f:
                f.write(json.dumps(meta) + "\n")
        except Exception as e:
            jsonl_ok = False
            self.get_logger().error(f"[ERROR] frame_number={self.frame_number} timestamp={ts_iso}   metadata JSONL write failed: {e}")

        #-----LOGGING THE FRAME SAVE EVENT IN THE ROWS FOR CSV/XLSX-----
        self.rows.append(
            {
               "event": "frame_save",
               "frame_number": self.frame_number,
               "timestamp": ts_iso,
               "save_status": True,
               "filename": filename,
               "full_path": filepath,
               "jsonl_write_status": jsonl_ok,
           }
        )

        #-----DISPLAY THE LOG INFO ON THE CONSOLE-----
        self.get_logger().info(f"[SAVED] frame_number={self.frame_number} timestamp={ts_iso}   path={filepath} status={'SUCCESS' if ok else 'FAIL'}")

    def destroy_node(self):
        # Add an explicit end row
        end_iso = datetime.now().isoformat(timespec="milliseconds")
        self.rows.append(
            {
               "event": "subscribe_end",
               "frame_number": self.frame_number,
               "timestamp": end_iso,
               "save_status": True,
               "filename": None,
               "full_path": None,
               "jsonl_write_status": True,
           }
        )

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
