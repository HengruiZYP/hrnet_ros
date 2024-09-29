import os, sys

upper_level_path = os.path.abspath(os.path.dirname(__file__))
sys.path.append(upper_level_path)
print(upper_level_path)
from .hrnet import KeyPointDetector, VisKeypointDetection
import yaml
import cv2
import numpy as np
import argparse
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

def argsparser():
    """
    argparser
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        default="./model/config.json",
        type=str,
        help=("Path of deploy config.json.Detection config comes first"),
    )
    parser.add_argument(
        "--infer_yml",
        default="./model/infer_cfg.yml",
        type=str,
        help=("Path of infer_yml.Detection config comes first"),
    )
    parser.add_argument(
        "--test_image",
        type=str,
        default="./test_images/0.jpeg",
        help="Path of test image file.",
    )
    parser.add_argument(
        "--visualize", action="store_true",help="whether to visualize."
    )
    parser.add_argument(
        "--with_profile",
        action="store_true",
        help="whether to predict with profile.",
    )
    return parser



class HrnetPublisher(Node):
    def __init__(self, args):
        super().__init__("Hrnet_pub")
        self.publisher = self.create_publisher(Int32MultiArray, "chatter", 10)

        # init PPNCDetector

        deploy_config = os.path.join(upper_level_path, args.config)
        assert os.path.exists(deploy_config), "config does not exist."
        
        deploy_infer_yml = os.path.join(upper_level_path, args.infer_yml)
        assert os.path.exists(deploy_infer_yml), "infer_yml does not exist."
        with open(deploy_infer_yml, "r") as f:
            infer_yml = yaml.safe_load(f)

        net = KeyPointDetector(deploy_config, infer_yml)
        cap = cv2.VideoCapture(0)

        
        if cap.isOpened():
            print("video opened")
            
        else:
            print("video not opened")
            return
            
        # fps base args    
        prev_time = time.time()
        frame_count = 0
        fps = 0
        
        while True:

            ret, image = cap.read()
            if not ret:
                break

            # 计算帧率        
            current_time = time.time()        
            frame_count += 1        
            elapsed_time = current_time - prev_time
            
            if elapsed_time > 1:            
                fps = frame_count / elapsed_time            
                prev_time = current_time            
                frame_count = 0        
                
            # 在帧上绘制帧率        
            cv2.putText(image, f'FPS: {fps:.2f}', (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0),2)
            
            with_profile = args.with_profile
            if with_profile:
                # return with time consumption for each stage
                res = net.predict_profile(image)
                print("preprocess time: ", net.preprocess_time)
                print("predict time: ", net.predict_time)
                print("postprocess time:", net.postprocess_time)

                total_time = (
                    net.preprocess_time + net.predict_time + net.postprocess_time
                )
                print("total time: ", total_time)
            else:
                res = net.predict_image(image)
                
            visualize = args.visualize
            if visualize:

                render_img, arm_info_list = VisKeypointDetection(image, res, 0.6)

                for arm_info in arm_info_list:
                    if arm_info['right_lower_arm'] is None:
                            right_arm_angle = 0
                    if arm_info['right_lower_arm'] is None:
                            left_arm_angle = 0

                    if arm_info['right_lower_arm'] is not None:
                        angle_msg = Int32MultiArray()
                        right_arm_angle = int(arm_info['right_lower_arm']['angle'])
                        angle_msg.data = [right_arm_angle, left_arm_angle]
                        self.publisher.publish(angle_msg)

                    if arm_info['left_lower_arm'] is not None:
                        angle_msg = Int32MultiArray()
                        left_arm_angle = int(arm_info['left_lower_arm']['angle'])
                        angle_msg.data = [right_arm_angle, left_arm_angle]
                        self.publisher.publish(angle_msg)


                cv2.imshow('camera', render_img)
                
            if cv2.waitKey(5) & 0xff == 27:
                break
                
        cap.release()
        cv2.destroyAllWindows()


def main(args=None):
    """main"""
    parser = argsparser()
    args_shell = parser.parse_args()

    command = f"sudo chmod 666 /dev/i2c-3"
    os.system(command)
    command = f"sudo chmod 666 /dev/video0"
    os.system(command)
    command = f"sudo chmod 666 /dev/synap"
    os.system(command)
    
    os.chdir(upper_level_path)

    rclpy.init(args=args)
    yolo_publisher = HrnetPublisher(args_shell)
    rclpy.spin(yolo_publisher)
    yolo_publisher.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()
