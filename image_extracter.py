import rosbag
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import argparse

def extract(input_path, output_path, topic_list):
    bridge = CvBridge()
    cnt = 0
    path = '/home/autoware/shared_dir/robot_vision/dataset/carla/image_raw_640/'
    bag = rosbag.Bag('/home/autoware/shared_dir/bag/carla2.bag')
    for topic, msg, t in bag.read_messages(topics=topic_list):
        cv_image = bridge.imgmsg_to_cv2(msg, 'bgr8')
        name = str(cnt).zfill(6)
        cv2.imwrite(path+name+'.png',cv_image)
        cnt = cnt + 1
    bag.close()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    parser.add_argument('-i', '--input_path', 
        default='input',
        help='folder with input images'
    )

    parser.add_argument('-o', '--output_path', 
        default='output',
        help='folder for output images'
    )

    args = parser.parse_args()
    
    topic_list = ['/carla/ego_vehicle/camera/rgb/front/image_color']
    extract(args.input_path, args.output_path, topic_list)



