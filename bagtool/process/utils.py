# Copyright 2024 Nimrod Curtis
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Standard library imports
import os
from dataclasses import dataclass, field
from typing import Any, List

# Third party library imports
import numpy as np
import cv2
import matplotlib.pyplot as plt
import moviepy.video.io.VideoFileClip as mp
from matplotlib import animation

# ROS libraries
from cv_bridge import CvBridge, CvBridgeError
bridge = CvBridge()
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CompressedImage
from zed_interfaces.msg import *


class TrajViz():
    """
    Methods:
    visualization: Generates a single frame for the trajectory video.
    save_animation: Saves an animation to a file.
    """
    def __init__(self) -> None:
        pass
    
    @staticmethod
    def visualization(robot_position,
                    yaw,
                    curr_image,
                    time,
                    frame_idx,
                    ax_image,
                    ax_trajectory,
                    target_position=None,
                    corners = None):

        # Frame=[]
        
        # ax_trajectory.legend(["Robot Location"],loc="upper right")
        # # Swap red and blue channels
        # curr_image_red_blue_swapped = curr_image[:, :, ::-1]
        
        # # plot_0=ax_image.imshow(curr_image)
        # title = ax_image.text((curr_image.shape[1]-100),0, "", bbox={'facecolor':'w', 'alpha':0.7, 'pad':5},
        #         fontsize=12,
        #         ha="left",
        #         va="top")
        # title.set_text(f"Frame: {frame_idx} | Time: {time:.4f} [sec]")
        
        # Frame.append(title)
        # Frame.append(ax_image.imshow(curr_image_red_blue_swapped))
        # Frame.append(ax_trajectory.plot(robot_position[:,1],robot_position[:,0],
        #                                 c="r",linewidth=2, markersize=8)[0])

        # if(len(target_position)>0):
        #     Frame.append(ax_trajectory.plot(target_position[:,1],target_position[:,0],
        #                                     c="b",linewidth=2 , markersize=16)[0])

        #     if corners is not None:
                
        #         # inverted axes
        #         x = corners[:,1]
        #         y = corners[:,0]
        #         Frame.append(ax_trajectory.plot([x[0], x[1], x[2], x[3], x[0]],
        #                                         [y[0], y[1], y[2], y[3], y[0]],
        #                                         'b-', marker='o',linewidth=2, markersize=16)[0])
        
        Frame=[]
        
        
        # Swap red and blue channels
        curr_image_red_blue_swapped = curr_image[:, :, ::-1]
        
        # plot_0=ax_image.imshow(curr_image)
        title = ax_image.text((curr_image.shape[1]-100),0, "", bbox={'facecolor':'w', 'alpha':0.7, 'pad':5},
                fontsize=12,
                ha="left",
                va="top")
        title.set_text(f"Frame: {frame_idx} | Time: {time:.4f} [sec]")
        

        
        Frame.append(title)
        Frame.append(ax_image.imshow(curr_image_red_blue_swapped))
        
        
        handles = []
        robot_pos_plot = ax_trajectory.plot(robot_position[:,1],robot_position[:,0],
                                        c="r",linewidth=4, markersize=8,
                                        label = 'Robot position [m]')[0]
        handles.append(robot_pos_plot)
        Frame.append(robot_pos_plot)

        if(len(target_position)>0):
            target_pos_plot = ax_trajectory.plot(target_position[:,1],target_position[:,0],
                                            c="b",linewidth=4 , markersize=16,
                                            label = 'Target Object Position [m]')[0]
            handles.append(target_pos_plot)
            Frame.append(target_pos_plot)
            if corners is not None:
                
                # inverted axes
                x = corners[:,1]
                y = corners[:,0]
                target_box_plot = ax_trajectory.plot([x[0], x[1], x[2], x[3], x[0]],
                                                [y[0], y[1], y[2], y[3], y[0]],
                                                'b-', marker='o',linewidth=4, markersize=16)[0]
                handles.append(target_box_plot)
                Frame.append(target_box_plot)
                
        ax_trajectory.legend(handles = handles)

        return Frame

    @staticmethod
    def save_animation(ani, dest_dir, file_name):
        """
        Save an animation object to a file.

        Args:
            ani: The animation object to be saved. 
                This is typically an instance of a Matplotlib animation.
            dest_dir (str): The directory where the animation file will be saved.
                It should be a valid directory path.
            file_name (str): The name of the file to which the animation will be saved. The file name should include the extension.

        Returns:
            None: This function does not return anything. It saves the animation to the specified file.
        """
        print("[INFO]  Saving animation")

        gif_file_path = os.path.join(dest_dir, f'{file_name}.gif')
        mp4_file_path = os.path.join(dest_dir, f'{file_name}.mp4')

        writergif = animation.PillowWriter(fps=10)
        ani.save(gif_file_path, writer=writergif)
        
        clip = mp.VideoFileClip(gif_file_path)
        clip.write_videofile(mp4_file_path)
        os.remove(gif_file_path)
        print("[INFO]  Animation saved")



@dataclass
class ObjDet:
    label: str = "None"
    label_id: int = -1
    instance_id: int = -1
    confidence: float = -1.0
    tracking_state: int = -1
    tracking_available: bool = False
    position: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    position_covariance: List[float] = field(default_factory=lambda: [0.0]*6)
    velocity: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    bbox3d: List[List[float]] = field(default_factory=lambda: [[0.0, 0.0, 0.0]]*8)
    bbox2d: List[List[float]] = field(default_factory=lambda: [[0.0, 0.0]]*4)


def get_key_by_value(my_dict, value):
    """
    Finds the first key in a dictionary that corresponds to the given value.

    Args:
        my_dict (dict): The dictionary to search.
        value: The value for which to find the corresponding key.

    Returns:
        The key corresponding to the input value. Returns None if the value is not found.

    """
    
    key = None
    # Iterate through each key-value pair in the dictionary
    for k, v in my_dict.items():
        # Check if the current value matches the input value
        if v == value:
            key = k  # If found, set the key to the current key
            break  # Exit the loop as soon as the key is found
    return key

def odom_to_numpy(msg:Odometry):
    """
    Converts an Odometry message to numpy arrays representing position, orientation, linear velocity, and angular velocity.

    Args:
        msg (Odometry): An instance of Odometry.

    Returns:
        A list of numpy arrays: [position, orientation, linear velocity, angular velocity], where each array represents a different aspect of the Odometry message.
    """
    
    # Extract position and orientation from Odometry message
    pos = [msg.pose.pose.position.x,
        msg.pose.pose.position.y,
        msg.pose.pose.position.z]
    ori = [msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w]

    # Extract linear and angular velocity
    lin_vel = [msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z]
    angular_vel = [msg.twist.twist.angular.x,
                msg.twist.twist.angular.y,
                msg.twist.twist.angular.z]

    # Convert the extracted values to numpy arrays and return them
    return [np.array(pos), np.array(ori), np.array(lin_vel), np.array(angular_vel)]


def np_odom_to_xy_yaw(np_odom:np.ndarray, prev_data, Ao)->Any:
    """
    Extracts x, y coordinates and yaw angle from a numpy representation of Odometry.

    Args:
        np_odom (np.ndarray): Numpy representation of Odometry, typically output of `odom_to_numpy` function.
        t (int): step 
    Returns:
        A dictionary containing 'pos', a list of x and y coordinates, and 'yaw', the yaw angle [radians].
    """
    
    x_gt, y_gt = np_odom[0][0], np_odom[0][1]
    yaw_gt = quat_to_yaw(np_odom[1])
    
    
    vt =  np_odom[2][0]
    wt = np_odom[3][2] if np_odom[3][2] != 0 else 10**-7
    
    if(prev_data is not None):
        pos_in_odom = Ao @ np.array([x_gt, y_gt, 1]).T
        x_in_odom, y_in_odom = pos_in_odom[0], pos_in_odom[1]
        prev_x_in_odom, prev_y_in_odom = prev_data['odom_frame']['position'][0], prev_data['odom_frame']['position'][1]
        dx_in_odom, dy_in_odom = x_in_odom - prev_x_in_odom,y_in_odom - prev_y_in_odom
        
        dyaw = normalize_angle(yaw_gt) - normalize_angle(prev_data['gt_frame']['yaw'])
        prev_yaw_in_odom = prev_data['odom_frame']['yaw']
        yaw_in_odom = prev_yaw_in_odom + dyaw
        yaw_in_odom = normalize_angle(yaw_in_odom)
        
        Ar = get_transform_to_start(prev_x_in_odom,prev_y_in_odom,prev_yaw_in_odom)
        
        pos_rel_to_prev = Ar @ np.array([x_in_odom, y_in_odom, 1]).T
        x_rel, y_rel =  pos_rel_to_prev[0], pos_rel_to_prev[1]
        yaw_rel = dyaw
        
        
    else:
        x_in_odom, y_in_odom = 0. , 0.
        yaw_in_odom = 0.
        dx_in_odom, dy_in_odom = 0. , 0.
        dyaw = 0
        
        x_rel, y_rel =  0. , 0.
        yaw_rel = 0.


    return {'gt_frame':{'position':[x_gt, y_gt], 'yaw': yaw_gt},
            'odom_frame':{'position':[x_in_odom,y_in_odom], 'yaw': yaw_in_odom, 'dpos':[dx_in_odom, dy_in_odom],'dyaw': dyaw},
            'relative_frame':{'position':[x_rel,y_rel], 'yaw': yaw_rel}}

def get_transform_to_start(x_start, y_start, yaw_start):
    
    A = np.array([[np.cos(yaw_start), -np.sin(yaw_start), x_start],
                [  np.sin(yaw_start), np.cos(yaw_start) , y_start],
                [  0             , 0              ,1]])
    Ainv = np.linalg.inv(A)
    
    return Ainv


def quat_to_yaw(quat: np.ndarray) -> float:
    """
    Converts a quaternion to a yaw angle [radians].

    Args:
        quat (np.ndarray): A numpy array representing a quaternion in the order [x, y, z, w].

    Returns:
        float: The yaw angle derived from the quaternion.
    """
    
    # Unpack the quaternion components
    x, y, z, w = quat[0], quat[1], quat[2], quat[3]
    # Compute the yaw from the quaternion
    t3 = 2.0 * (w * z + x * y)
    t4 = 1.0 - 2.0 * (y * y + z * z)
    yaw = np.arctan2(t3, t4)
    return yaw


def normalize_angle(angle):
    if -np.pi < angle <= np.pi:
        return angle
    if angle > np.pi:
        angle = angle - 2 * np.pi
    if angle <= -np.pi:
        angle = angle + 2 * np.pi
    return normalize_angle(angle)


def normalize_angles_array(angles):
    z = np.zeros_like(angles)
    for i in range(angles.shape[0]):
        z[i] = normalize_angle(angles[i])
    return z

## msg structure:
## https://github.com/stereolabs/zed-ros-interfaces/tree/main/msg
def object_detection_to_dic(msg: ObjectsStamped):

    obj = ObjDet()

    # check there is detection
    if len(msg.objects)>0 :
        obj_ros = msg.objects[0]

        obj.label = obj_ros.label
        obj.label_id = obj_ros.label_id
        obj.instance_id = obj_ros.instance_id
        obj.confidence = obj_ros.confidence
        obj.tracking_state = obj_ros.tracking_state
        obj.tracking_available = obj_ros.tracking_available
        obj.position = list(obj_ros.position)
        obj.position_covariance = list(obj_ros.position_covariance)
        obj.velocity = list(obj_ros.velocity)
        obj.bbox3d = bbox_to_list(obj_ros.bounding_box_3d)
        obj.bbox2d = bbox_to_list(obj_ros.bounding_box_2d)

    return {
        "label": obj.label, "label_id": obj.label_id, "instance_id": obj.instance_id,
        "confidence": obj.confidence, "tracking_state": obj.tracking_state,"tracking_available": obj.tracking_available,
        "position": obj.position, "position_covariance": obj.position_covariance, "velocity": obj.velocity, 
        "bbox3d": obj.bbox3d,"bbox2d": obj.bbox2d           
    }

def bbox_to_list(msg):
    corners = []
    for corner in msg.corners:
        corners.append(list(corner.kp))
    return corners

def image_compressed_to_numpy(msg:CompressedImage)->np.ndarray:
    """
    Converts a ROS CompressedImage message to a numpy array.

    Args:
        msg (CompressedImage): A ROS CompressedImage message.

    Returns:
        np.ndarray: A numpy array representing the image.
    """
    
    # Convert CompressedImage to OpenCV Image using np.frombuffer
    np_img = np.frombuffer(msg.data, dtype=np.uint8)
    np_img = cv2.imdecode(np_img, cv2.IMREAD_COLOR)

    return np_img

def image_to_numpy(
    msg, empty_value=None, output_resolution=None,max_depth=5000, use_bridge=False
):
    """
    Converts a ROS image message to a numpy array, with options for data manipulation.

    Args:
        msg: A ROS image message.
        empty_value: A value to identify and replace missing data in the image.
        output_resolution (tuple, optional): Desired resolution of output image.

    Returns:
        np.ndarray: The image represented as a numpy array.
    """

    # Set default output resolution if not provided
    if output_resolution is None:
        output_resolution = (msg.width, msg.height)

    # Check if image encoding is RGB
    is_rgb = "8" in msg.encoding
    is_depth16 = "16" in msg.encoding
    is_depth32 = "32" in msg.encoding
    
    
    # Convert the image data to the appropriate format
    if not use_bridge:
        
        if is_rgb:
            nchannels=3
            data = np.frombuffer(msg.data, dtype=np.uint8).copy()
            data = data.reshape(msg.height, msg.width, 4)[:,:,:nchannels]
        elif is_depth16:
            data = np.frombuffer(msg.data, dtype=np.uint16).copy()
            data = data.reshape(msg.height, msg.width)
            
            if max_depth:
                data = 1 - (np.clip(data, a_min=0, a_max=max_depth) / max_depth)
                data = np.array(255*data.astype(np.float32),dtype=np.uint8)
        
        elif is_depth32: ## currently not good decoding 
            data = np.frombuffer(msg.data, dtype=np.float32).copy()
            data = data.reshape(msg.height, msg.width)
    else:
        # convert image msgs to opencv format
            try:
                cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough") # "passthrough" = keep the same encoding format of the image
                if is_depth16:
                    if max_depth:
                        data = 1 - (np.clip(data, a_min=0, a_max=max_depth) / max_depth)
                        data = np.array(255*data.astype(np.float32),dtype=np.uint8)
                    else:
                        data = np.array(256*data.astype(np.float32)/0x0fff,dtype=np.uint8)

                if is_rgb:
                    data = np.array(cv_img[:,:,:3], dtype=np.int32)
            except CvBridgeError as e:
                print(e)

    # Replace empty values if specified
    if empty_value:
        mask = np.isclose(abs(data), empty_value)
    else:
        mask = np.isnan(data)
    
    fill_value = np.percentile(data[~mask], 99)
    data[mask] = fill_value

    # Resize the image to the desired output resolution
    data = cv2.resize(
        data,
        dsize=(output_resolution[0], output_resolution[1]),
        interpolation=cv2.INTER_AREA,
    )


    # Create a mask to filter out NaN and Inf values
    # valid_mask = np.isfinite(depth_image)

    # # Replace NaN and Inf values with a value outside the desired range (e.g., 0)
    # depth_image[~valid_mask] = 0

    # # Normalize the depth values to the range [0, 255]
    # min_depth = np.min(depth_image[valid_mask])
    # max_depth = np.max(depth_image[valid_mask])
    # normalized_depth = ((depth_image - min_depth) / (max_depth - min_depth) * 255).astype(np.uint8)

    # # Convert the normalized depth image to an 8-bit unsigned integer image
    # depth_8bit = cv2.convertScaleAbs(normalized_depth)
    
    return data