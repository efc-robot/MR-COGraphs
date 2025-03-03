import rosbag
import sys
import os
import numpy as np
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import rospy
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import CameraInfo
import json
import glob
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
import liblzfse

from scipy.spatial.transform import Rotation as R

desired_width = 960
desired_height = 720

# desired_width = 720
# desired_height = 960

bridge = CvBridge()
init_time = 10.0

def get_info(filepath):
    metadata = None
    with open(filepath, 'r') as f:
        metadata = json.load(f)
    height = metadata['h']
    width = metadata['w']
    K = metadata['K']
    time_stamp = metadata['frameTimestamps']
    poses = metadata['poses']
    return height, width, K, time_stamp, poses

def create_camera_info(K, h, w, seq, time_stamp):
    info = CameraInfo()
    info.header.stamp = rospy.Time.from_sec(time_stamp + init_time)  # 修改这行
    info.header.seq = seq
    info.header.frame_id = "carter_camera_stereo_left "
    info.height = h
    info.width = w
    info.distortion_model = "plumb_bob"
    info.D = []  # 默认值
    K_new = [0, 0, 0, 0, 0, 0, 0, 0, 0]
    K_new[0] = K[0] # fx
    K_new[2] = K[4] # fy
    K_new[4] = K[6] # cx
    K_new[5] = K[7] # cy
    fx = K[0]
    fy = K[4]
    cx = K[6]
    cy = K[7]
    K = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
    # 将x和y对调
    # K[0], K[2] = K[2], K[0]
    # K[4], K[5] = K[5], K[4]
    info.K = K  # 从输入参数获取
    info.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]  # 默认值
    # info.P = [K[0], 0.0, K[2], 0.0, 0.0, K[4], K[5], 0.0, 0.0, 0.0, 1.0, 0.0]  # 假设没有镜头畸变
    info.P = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]  # 假设没有镜头畸变
    

    # info.K = K  # 从输入参数获取
    # info.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]  # 默认值
    # info.P = [K[0], 0.0, K[2], 0.0, 0.0, K[4], K[5], 0.0, 0.0, 0.0, 1.0, 0.0]  # 假设没有镜头畸变
    
    info.binning_x = 0
    info.binning_y = 0
    info.roi.do_rectify = False
    return info

def camerainfo_to_rosbag(filepath, bag_path):
    height, width, K, time_stamps, _ = get_info(filepath)
    bag_out = rosbag.Bag(bag_path, 'w')
    for (index, stamp) in enumerate(time_stamps):
        info = create_camera_info(K, height, width, index, stamp)
        bag_out.write("/camera_info", info, stamp)
    bag_out.close()

def find_files(root_dir, filter_type, reverse=False):
    """
    在指定目录查找指定类型文件 -> paths, names, files
    :param root_dir: 查找目录
    :param filter_type: 文件类型
    :param reverse: 是否返回倒序文件列表，默认为False
    :return: 路径、名称、文件全路径
    """

    separator = os.path.sep
    paths = []
    names = []
    files = []
    for parent, dirname, filenames in os.walk(root_dir):
        for filename in filenames:
            if filename.endswith(filter_type):
                paths.append(parent + separator)
                names.append(filename.split('.')[0])
    for i in range(paths.__len__()):
        files.append(paths[i] + names[i] + filter_type)
    print(names.__len__().__str__() + " files have been found.")
    
    paths = np.array(paths)
    names = np.array(names)
    files = np.array(files)
    # names 从str转为int
    names = names.astype(int)

    index = np.argsort(names)

    paths = paths[index]
    names = names[index]
    files = files[index]

    paths = list(paths)
    names = list(names)
    files = list(files)
    
    if reverse:
        paths.reverse()
        names.reverse()
        files.reverse()
    return paths, names, files

def image_to_rosmsg(img_path, seq, stamp):
    img = cv2.imread(img_path)
    
    img_msg = bridge.cv2_to_imgmsg(img, encoding="bgr8")
    img_msg.header.stamp = rospy.Time.from_sec(stamp + init_time)
    img_msg.header.seq = seq
    img_msg.header.frame_id = "carter_camera_stereo_left " 
    return img_msg

def rgb_to_rosbag(img_dir, bag_path):
    bag_out = rosbag.Bag(bag_path, 'w')

    # img_dir = "/home/mrasamu/ubuntu/rosbag/ipad_record_3D/ipad1/rgbd"
    paths, names, files = find_files(img_dir,".jpg")
    filepath = "/home/mrasamu/ubuntu/rosbag/ipad_record_3D/ipad1/metadata"
    height, width, K, time_stamps, _ = get_info(filepath)
    for (index, file) in enumerate(files):
        img_msg = image_to_rosmsg(file, index, time_stamps[index])
        bag_out.write("/camera/image_raw", img_msg, img_msg.header.stamp)
    bag_out.close()

def pose_to_rosbag(pose_path, bag_path):
    _, _, _, time_stamps, poses = get_info(pose_path)
    bag_out = rosbag.Bag(bag_path, 'w')
    for (index, stamp) in enumerate(time_stamps):
        pose = poses[index]
        tf_msg = TFMessage()
        transform = TransformStamped()
        transform.header.stamp = rospy.Time.from_sec(stamp+init_time)
        # transform.header.seq = index
        transform.header.frame_id = "world"
        transform.child_frame_id = "camera"
        transform.transform.translation = Vector3(pose[4], pose[5], pose[6])
        transform.transform.rotation.x = pose[0]
        transform.transform.rotation.y = pose[1]
        transform.transform.rotation.z = pose[2]
        transform.transform.rotation.w = pose[3]
        tf_msg.transforms.append(transform)
        bag_out.write("/tf", tf_msg, transform.header.stamp)
    bag_out.close()

def rgb_pose_to_rosbag(img_dir, pose_path, bag_path):
    bag_out = rosbag.Bag(bag_path, 'w')
    paths, names, files = find_files(img_dir, ".jpg")
    _, _, _, time_stamps, poses = get_info(pose_path)
    for (index, file) in enumerate(files):
        img_msg = image_to_rosmsg(file, index, time_stamps[index])
        bag_out.write("/camera/image_raw", img_msg, img_msg.header.stamp)
        pose = poses[index]
        tf_msg = TFMessage()
        transform = TransformStamped()
        transform.header.stamp = rospy.Time.from_sec(time_stamps[index] + init_time)
        transform.header.frame_id = "world"
        transform.child_frame_id = "camera"
        transform.transform.translation = Vector3(pose[4], pose[5], pose[6])
        transform.transform.rotation.x = pose[0]
        transform.transform.rotation.y = pose[1]
        transform.transform.rotation.z = pose[2]
        transform.transform.rotation.w = pose[3]
        tf_msg.transforms.append(transform)
        bag_out.write("/tf", tf_msg, transform.header.stamp)
    bag_out.close()

def resize_depth(depth_img, desired_width, desired_height):
    return cv2.resize(depth_img, (desired_width, desired_height), interpolation=cv2.INTER_NEAREST)

def depth_to_rosmsg(filepath, seq, stamp):
    with open(filepath, 'rb') as depth_fh:
        raw_bytes = depth_fh.read()
        decompressed_bytes = liblzfse.decompress(raw_bytes)
        depth_img = np.frombuffer(decompressed_bytes, dtype=np.float32)
        # depth_img = depth_img.reshape((256, 192))  # Original resolution
        depth_img = depth_img.reshape((192, 256))  # Original resolution
        depth_img = resize_depth(depth_img, desired_width, desired_height)
        depth_msg = bridge.cv2_to_imgmsg(depth_img, encoding="32FC1")
        depth_msg.header.stamp = rospy.Time.from_sec(stamp + init_time)
        depth_msg.header.seq = seq
        depth_msg.header.frame_id = "carter_camera_stereo_left "
        return depth_msg
    
def depth_to_rosbag(depth_dir, bag_path):
    bag_out = rosbag.Bag(bag_path, 'w')
    paths, names, files = find_files(depth_dir, ".depth")
    filepath = "/home/mrasamu/ubuntu/rosbag/ipad_record_3D/ipad1/metadata"
    _, _, _, time_stamps, _ = get_info(filepath)
    for (index, file) in enumerate(files):
        depth_msg = depth_to_rosmsg(file, index, time_stamps[index])
        bag_out.write("/camera/depth/image_raw", depth_msg, depth_msg.header.stamp)
    bag_out.close()

# 将四元数转换为旋转矩阵
def quaternion_to_rotation_matrix(quaternion):
    r = R.from_quat(quaternion)
    return r.as_matrix()

# 将旋转矩阵转换为四元数
def rotation_matrix_to_quaternion(matrix):
    r = R.from_matrix(matrix)
    return r.as_quat()

# 沿Z轴旋转90度的旋转矩阵
def rotation_z_90():
    theta = np.radians(90)
    return np.array([
        [np.cos(theta), -np.sin(theta), 0],
        [np.sin(theta), np.cos(theta), 0],
        [0, 0, 1]
    ])

# 沿Y轴旋转90度的旋转矩阵
def rotation_y_90():
    theta = np.radians(-90)
    return np.array([
        [np.cos(theta), 0, np.sin(theta)],
        [0, 1, 0],
        [-np.sin(theta), 0, np.cos(theta)]
    ])

# 沿X轴旋转90度的旋转矩阵
def rotation_x_90():
    theta = np.radians(90)
    return np.array([
        [1, 0, 0],
        [0, np.cos(theta), -np.sin(theta)],
        [0, np.sin(theta), np.cos(theta)]
    ])

def rotation_x_90_4x4():
    theta = np.radians(90)
    matrix_x = np.zeros((4, 4))
    matrix_x[3, 3] = 1
    matrix_x[0:3, 0:3] = rotation_x_90()
    matrix_x[0:3, 3] = np.array([0, 0, 0])
    return matrix_x

def rotation_x_in_90():
    theta = np.radians(90)
    return np.array([
        [1, 0, 0],
        [0, np.cos(theta), -np.sin(theta)],
        [0, np.sin(theta), np.cos(theta)]
    ])

import math

def rgb_pose_depth_to_rosbag(img_dir, depth_dir, pose_path, bag_path):
    bag_out = rosbag.Bag(bag_path, 'w')
    _, _, rgb_files = find_files(img_dir, ".jpg")
    _, _, _, time_stamps, poses = get_info(pose_path)
    _, _, depth_files = find_files(depth_dir, ".depth")
    height, width, K, time_stamps, _ = get_info(pose_path)
    index_pic = 0

    # original_quaternion_init = np.array([-0.5,0.5, -0.5,0.5])
    # rotation_matrix_init = quaternion_to_rotation_matrix(original_quaternion_init)
    # # 绕x轴旋转90度
    # rotation_matrix_init = np.dot(rotation_matrix_init, rotation_x_in_90())
    # new_quaternion_init = rotation_matrix_to_quaternion(rotation_matrix_init)

    # # 绕z轴旋转90度
    # rotation_matrix_init = np.dot(rotation_matrix_init, rotation_z_90())
    # new_quaternion = rotation_matrix_to_quaternion(rotation_matrix_init)

    # print(new_quaternion_init)
    for (index, rgb_file) in enumerate(rgb_files):
        # RGB图像
        img_msg = image_to_rosmsg(rgb_file, index, time_stamps[index])
        bag_out.write("/rgb_left", img_msg, img_msg.header.stamp)

        # 深度图像
        depth_file = depth_files[index]
        depth_msg = depth_to_rosmsg(depth_file, index, time_stamps[index])
        bag_out.write("/depth_left", depth_msg, depth_msg.header.stamp)
        
        # 相机信息
        info = create_camera_info(K, height, width, index, time_stamps[index])
        bag_out.write("/camera_info_left", info, info.header.stamp)

        # tf
        pose = poses[index]
        tf_msg = TFMessage()
        transform = TransformStamped()
        transform.header.stamp = rospy.Time.from_sec(time_stamps[index] + init_time)
        transform.header.frame_id = "world"
        transform.child_frame_id = "base_link"

        # 竖屏录像
        # transform.transform.translation = Vector3(pose[4], -pose[6], pose[5])
        # original_quaternion = np.array([pose[0], -pose[2], pose[1], pose[3]])
        # # rotation_matrix = quaternion_to_rotation_matrix(original_quaternion)
        # # rotation_matrix = np.dot(rotation_matrix, rotation_x_90())
        # # new_quaternion = rotation_matrix_to_quaternion(rotation_matrix)
        # transform.transform.rotation.x = original_quaternion[0]
        # transform.transform.rotation.y = original_quaternion[1]
        # transform.transform.rotation.z = original_quaternion[2]
        # transform.transform.rotation.w = original_quaternion[3]

        # 横屏录像
        # transform.transform.translation = Vector3(pose[4], pose[5], pose[6])
        original_quaternion = np.array([pose[0], pose[1], pose[2] , pose[3]]) # 四元数
        original_matrix = np.zeros((4, 4))
        original_matrix[3, 3] = 1
        original_matrix[0:3, 0:3] = quaternion_to_rotation_matrix(original_quaternion) # 变换矩阵
        original_matrix[0:3, 3] = np.array([pose[4], pose[5], pose[6]])
        
        rotation_matrix = np.dot(rotation_x_90_4x4(), original_matrix) # 新变换矩阵，绕x轴旋转90度
        # rotation_matrix = quaternion_to_rotation_matrix(original_quaternion)
        # rotation_matrix = np.dot(rotation_matrix, rotation_x_90())
        new_quaternion = rotation_matrix_to_quaternion(rotation_matrix[0:3, 0:3]) # 新四元数
        transform.transform.rotation.x = new_quaternion[0]
        transform.transform.rotation.y = new_quaternion[1]
        transform.transform.rotation.z = new_quaternion[2]
        transform.transform.rotation.w = new_quaternion[3]
        transform.transform.translation = Vector3(rotation_matrix[0, 3], rotation_matrix[1, 3], rotation_matrix[2, 3]) 
        tf_msg.transforms.append(transform)

        
        transform_camera = TransformStamped()
        transform_camera.header.stamp = rospy.Time.from_sec(time_stamps[index] + init_time)
        transform_camera.header.frame_id = "base_link"
        transform_camera.child_frame_id = "carter_camera_stereo_left"
        # 设置旋转为绕x轴旋转180度
        roll = math.pi
        pitch =  0
        yaw = 0
        # 获取四元数
        quat = R.from_euler('xyz', [roll, pitch, yaw]).as_quat()
        transform_camera.transform.rotation.x = quat[0]
        transform_camera.transform.rotation.y = quat[1]
        transform_camera.transform.rotation.z = quat[2]
        transform_camera.transform.rotation.w = quat[3]
        transform_camera.transform.translation = Vector3(0, 0, 0)
        tf_msg.transforms.append(transform_camera)

        # transform_camera1 = TransformStamped()
        # transform_camera1.header.stamp = rospy.Time.from_sec(time_stamps[index] + init_time)
        # transform_camera1.header.frame_id = "camera0"
        # transform_camera1.child_frame_id = "camera"

        # # 设置旋转为绕y轴旋转180度
        # roll = 0
        # pitch =  math.pi
        # yaw = 0
        # # 获取四元数
        # quat = R.from_euler('xyz', [roll, pitch, yaw]).as_quat()
        # transform_camera1.transform.rotation.x = quat[0]
        # transform_camera1.transform.rotation.y = quat[1]
        # transform_camera1.transform.rotation.z = quat[2]
        # transform_camera1.transform.rotation.w = quat[3]

        # transform_camera1.transform.translation = Vector3(0, -1, 0)

        
        # tf_msg.transforms.append(transform_camera1)
        bag_out.write("/tf", tf_msg, transform.header.stamp)
        
        print("Processed frame: ", index)    

        # if(index > 10000):
        #     break
    bag_out.close()
    

def main():
    # filepath = "/home/mrasamu/ubuntu/rosbag/ipad_record_3D/ipad1/metadata"
    # height, width, K, time_stamps = get_info(filepath)
    # print(height, width, K, time_stamps)
    # for (index, stamp) in enumerate(time_stamps):
    #     info = create_camera_info(K, height, width, index, stamp)
    # print(info)
    
    # bag_path = "cp5_test.bag"
    # img_dir = "/home/mrasamu/ubuntu/rosbag/ipad_record_3D/cp5/rgbd"
    # filepath = "/home/mrasamu/ubuntu/rosbag/ipad_record_3D/cp5/metadata"
    bag_path = "cp_1robot_no_ns_0213.bag"
    img_dir = "/root/sharedocker/rosbags/cp_0213/rgbd"
    filepath = "/root/sharedocker/rosbags/cp_0213/metadata"

    # pose_to_rosbag(filepath, bag_path)
    # rgb_pose_to_rosbag(img_dir, filepath, bag_path)
    # depth_to_rosbag(img_dir, bag_path)
    rgb_pose_depth_to_rosbag(img_dir, img_dir, filepath, bag_path)
    roll = math.pi
    pitch =  0
    yaw = 0
    # 获取四元数
    quat = R.from_euler('xyz', [roll, pitch, yaw]).as_quat()
    print(quat)


if __name__ == '__main__':
    main()