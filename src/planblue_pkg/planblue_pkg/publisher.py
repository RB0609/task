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

        #-----TO DECLARE ROS2 PARAMETERS-----
        self.declare_parameter("device", "/dev/video0")             #Camera device path (e.g., /dev/video0 or /dev/video1)
        self.declare_parameter("fps", 5.0)                          #Frames per second for publishing
        self.declare_parameter("width", 1280)                       #Width of the video frame to capture and publish
        self.declare_parameter("height", 720)                       #Height of the video frame to capture and publish
        self.declare_parameter("topic", "/camera/image_raw")        #ROS topic to publish the images to
        self.declare_parameter("folder_name", "publisher_captures") #Name of the folder to save logs and images (will be created in the user's home directory)
        self.declare_parameter("save_images", False)
        #----- TO READ THE VALUE AND STORE IT IN A VARIABLE-----
        self.device = self.get_parameter("device").value            
        self.fps = float(self.get_parameter("fps").value)
        self.width = int(self.get_parameter("width").value)
        self.height = int(self.get_parameter("height").value)
        self.topic = self.get_parameter("topic").value
        self.folder_name = self.get_parameter("folder_name").value
        self.save_images = bool(self.get_parameter("save_images").value)

        ''' ---- Output paths ---- use these code lines if you want to save next to this script
        script_dir = os.path.dirname(os.path.abspath(__file__))
        self.out_dir = os.path.join(script_dir, self.folder_name)
        os.makedirs(self.out_dir, exist_ok=True)
        '''

        # Save the files in the user's home directory
        home_dir = os.path.expanduser("~")                       #Get the user's home directory (e.g., /home/username)
        out_dir  = os.path.join(home_dir, "publisher_captures")  #Create a folder named "publisher_captures" in the home directory to store logs and images
        os.makedirs(out_dir, exist_ok=True)                      #Create the folder if it doesn't exist
        
        # Create a sub-folder inside the main folder to save the log info for each run. Easy for future reference
        run_stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        run_folder = f"published_{run_stamp}"
        self.out_dir = os.path.join(out_dir, self.folder_name, run_folder)
        os.makedirs(self.out_dir, exist_ok=True)
        self.xlsx_path = os.path.join(self.out_dir, f"publish_log.xlsx") #Path to save the log file in Excel format
        self.csv_path = os.path.join(self.out_dir, f"publish_log.csv")   #Path to save the log file in CSV format

        #-----ROS2 PUBLISHER SETUP-----
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.publisher_ = self.create_publisher(Image, self.topic, qos)
        self.bridge = CvBridge()

        #-----CAMERA SETUP FROM OPENCV EXAMPLE-----
        '''
        You can try cv.CAP_DSHOW or cv.CAP_V4L2 based on your system and camera compatibility. 
        V4L2 is often more compatible with USB webcams on Linux.
        cv.CAP_V4L2 is a backend for Video4Linux2, which is commonly used for camera capture on Linux systems.
        cv.CAP_DSHOW is a backend for DirectShow, which is commonly used on Windows systems.'''
        #to open the camera using OpenCV. 
        self.cap = cv.VideoCapture(self.device, cv.CAP_V4L2)    #create a VideoCapture ogject           
        if not self.cap.isOpened():
            raise RuntimeError(
                f"Could not open camera device {self.device}. "
                f"Try /dev/video0 or /dev/video1 and ensure permissions."
            )
        self.cap.set(cv.CAP_PROP_FRAME_WIDTH, self.width)                 #Set width
        self.cap.set(cv.CAP_PROP_FRAME_HEIGHT, self.height)               #Set height

        #-----LOGGING-----
        self.rows = []
        self.i = 0
        start_iso = datetime.now().isoformat(timespec="milliseconds")
        #----- TO LOG THE DATA OF PUBLISH START-----
        self.rows.append(
            {
               "event": "publish_start",
               "frame_number": None,
               "timestamp": start_iso,
               "publish_status": True,
               "filename": None,
               "full_path": None,
            } 
        )
        #-----DISPLAY THE LOG INFO ON THE CONSOLE-----
        self.get_logger().info(
            f"Publishing started: {start_iso} | device={self.device} | topic={self.topic} | fps={self.fps} "
            f"(Press Ctrl+C to stop)"
        )
        #-----TIMER-----
        '''We set up a timer to call the timer_callback function 
                        at a rate defined by the fps parameter.'''
        timer_period = 1.0 / self.fps if self.fps > 0 else 0.2                 #We will publish a message every 1/fps seconds.
        self.timer = self.create_timer(timer_period, self.timer_callback)      # create the timer and specify the callback function to be called at each timer event
    


    def timer_callback(self):
        '''
        Callback function.
        This function gets called periodically based on the timer we set up earlier.
        '''
        ret, frame = self.cap.read()                                            #Capture frame-by-frame. returns True/False and the frame itself as well as video frame.
        ts_iso = datetime.now().isoformat(timespec="milliseconds")              #Gets the current local time.
        #-----HANDLE CAMERA READ FAILURE-----
        if not ret or frame is None:
            self.rows.append(
                {
                   "event": "frame_publish",
                   "frame_number": self.i + 1,
                   "timestamp": ts_iso,
                   "publish_status": False,
                   "filename": None,
                   "full_path": None,
                }
            )
            self.get_logger().warn(
                f'Publishing: frame_number={self.i + 1} timestamp={ts_iso} status=FAIL (camera read failed)')
            return
        #-----PUBLISH FRAME, IF CAPTURE FRAME IS SUCCESSFUL-----
        self.i += 1
        publish_ok = True

        try:
            #cv2_to_imgmsg converts an OpenCV image (numpy array) to a ROS Image message.
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")   
            msg.header.stamp = self.get_clock().now().to_msg()        #Assign timestamp
            msg.header.frame_id = "camera"                            #Assign frame id
            self.publisher_.publish(msg)                              #Publish the ROS message
        except Exception as e:
            publish_ok = False
            self.get_logger().error(f"Publish failed: {e}")           #display the messge on the console if publish fails

        
        filename = None
        filepath = None
        #Optional: save raw frames from publisher
        '''if self.save_images:
            ts = datetime.now()
            filename = f"frame_{self.i:06d}_{ts.strftime('%Y%m%d_%H%M%S_%f')}.png"
            filepath = os.path.join(self.out_dir, filename)
            _ = cv.imwrite(filepath, frame)
        '''

        # -----LOGGING EACH PUBLISH ATTEMPT-----
        self.rows.append(
            {
               "event": "frame_publish",
               "frame_number": self.i,
               "timestamp": ts_iso,
               "publish_status": publish_ok,
               "filename": filename,
               "full_path": filepath,
           }
        )
        #-----DISPLAY THE LOG INFO ON THE CONSOLE-----
        self.get_logger().info(
            f'Publishing: frame_number={self.i} timestamp={ts_iso} status={"SUCCESS" if publish_ok else "FAIL"}'
        )

    def destroy_node(self):
        end_iso = datetime.now().isoformat(timespec="milliseconds") #Setting the end time, when it is interrupted by the user
        self.rows.append(
            {
               "event": "publish_end",
               "frame_number": self.i,
               "timestamp": end_iso,
               "publish_status": True,
               "filename": None,
               "full_path": None,
            }
        )
        #Display the log info on the console when the publishing is stopped by the user (Ctrl+C)
        print(f"[camera_publisher] Publishing stopped: {end_iso} | frames_published={self.i}") 
        
        #Release the camera resource when the node is destroyed. 
        #This is important to free up the camera for other applications and 
        #to ensure that it is properly closed when the node shuts down.
        try:
            if self.cap is not None:
                self.cap.release()
        except Exception:
                    pass
                
                
        try:
            df = pd.DataFrame(self.rows)                                      #Convert log list to table using pandas
            df.to_csv(self.csv_path, index=False)                             #Save CSV
            df.to_excel(self.xlsx_path, index=False)                          #Save XLSX
            print(f"[camera_publisher] Log saved to: {self.xlsx_path}")       #Display the log info on the console when the log file is saved successfully
            print(f"[camera_publisher] CSV saved to: {self.csv_path}")        
        except Exception as e:
            print(f"[camera_publisher][WARN] Failed to write logs: {e}")
            super().destroy_node()

def main(args=None):
    rclpy.init(args=args)                   #Initialize the rclpy library.
    camera_publisher = CameraPublisher()    #create the node.

    try:
        rclpy.spin(camera_publisher)        #Keep the node running and processing callbacks until it is interrupted (e.g., by Ctrl+C).
    except KeyboardInterrupt:
        pass

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    camera_publisher.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()    #Shutdown the ROS2 client library for python.


if __name__ == "__main__":
    main()




'''Refrences:
1) ROS2 AND OpenCV: cv-bridge, link:   https://github.com/ros-perception/vision_opencv/tree/rolling/cv_bridge
2) ROS2 pub/sub example:               https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html
2) ros2 and open-cv code reference:    https://automaticaddison.com/getting-started-with-opencv-in-ros-2-foxy-fitzroy-python/
3) GeeksforGeeks:                      https://www.geeksforgeeks.org/python-opencv-cv2-imwrite-method/
4) GeeksforGeeks:                      https://www.geeksforgeeks.org/python/extract-images-from-video-in-python/
4) Pandas to save log in Excel format: https://pandas.pydata.org/docs/reference/api/pandas.DataFrame.to_excel.html
5) Pandas to save log in CSV format:   https://pandas.pydata.org/docs/reference/api/pandas.DataFrame.to_csv.html
6) Python datetime:                    https://docs.python.org/3/library/datetime.html
7) ROS2 logging:                       https://docs.ros.org/en/rolling/api/rclpy.logging.html
8) ROS2 parameters:                    https://docs.ros.org/en/rolling/api/rclpy.parameter.html
'''
