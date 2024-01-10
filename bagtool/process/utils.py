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
from typing import Any
import os

# Third party library imports
import numpy as np
import cv2
import matplotlib.pyplot as plt
import moviepy.video.io.VideoFileClip as mp
from matplotlib import animation

# ROS libraries
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CompressedImage



class TrajViz():
    """
    Methods:
    visualization: Generates a single frame for the trajectory video.
    save_animation: Saves an animation to a file.
    """
    def __init__(self) -> None:
        pass
    
    @staticmethod
    def visualization(location,
                    yaw,
                    curr_image,
                    time,
                    frame_idx,
                    ax_image,
                    ax_trajectory):

        Frame=[]
        
        ax_trajectory.legend([f"Location"],loc="upper right")
        ax_trajectory.grid()

        plot_0=ax_image.imshow(curr_image)
        Frame.append(plot_0)

        plot_1,=ax_trajectory.plot(location[:,1],location[:,0],c="r")
        Frame.append(plot_1)
        
        # if location.shape[0]>2:
        #     plot_2 = ax_trajectory.arrow(location[-1,1],location[-1,0],
        #                             location[-1,1]+np.cos(yaw)*0.001,#(abs(location[-1,1]-location[-2,1])),
        #                             location[-1,0]+np.sin(-yaw)*0.001,#(abs(location[-1,0]-location[-2,0])),
        #                             head_width = 0.000005,
        #                             head_length = 0.000005)
        #     Frame.append(plot_2)


        title = ax_image.text((curr_image.shape[1])+10,0, "", bbox={'facecolor':'w', 'alpha':0.7, 'pad':5},
                fontsize=12,
                ha="left",
                va="top")
        title.set_text(f"Frame: {frame_idx} | Time: {time:.4f} [sec]")
        Frame.append(title)
        
        return Frame

    @staticmethod
    def save_animation(ani, dest_dir, file_name):
        """
        save animation function
        :param ani: animation object
        :param dest_dir: the parent dir of the animation file.
        :param file_name: the animation name
        :return: None
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
    

def np_odom_to_xy_yaw(np_odom:np.ndarray)->Any:
    """
    Extracts x, y coordinates and yaw angle from a numpy representation of Odometry.

    Args:
        np_odom (np.ndarray): Numpy representation of Odometry, typically output of `odom_to_numpy` function.

    Returns:
        A dictionary containing 'pos', a list of x and y coordinates, and 'yaw', the yaw angle.
    """
    x, y = np_odom[0][0], np_odom[0][1]
    yaw = quat_to_yaw(np_odom[1])
    return {'pos':[x,y], 'yaw': yaw}


def quat_to_yaw(quat: np.ndarray) -> float:
    """
    Converts a quaternion to a yaw angle.

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
    msg, nchannels=3, empty_value=None, output_resolution=None, aggregate="none"
):
    """
    Converts a ROS image message to a numpy array, with options for data manipulation.

    Args:
        msg: A ROS image message.
        nchannels (int): Number of channels in the image.
        empty_value: A value to identify and replace missing data in the image.
        output_resolution (tuple, optional): Desired resolution of output image.
        aggregate (str): Method for aggregating color channels.

    Returns:
        np.ndarray: The image represented as a numpy array.
    """
    
    # Set default output resolution if not provided
    if output_resolution is None:
        output_resolution = (msg.width, msg.height)

    # Check if image encoding is RGB
    is_rgb = "8" in msg.encoding
    # Convert the image data to the appropriate format
    if is_rgb:
        data = np.frombuffer(msg.data, dtype=np.uint8).copy()
    else:
        data = np.frombuffer(msg.data, dtype=np.float32).copy()

    # Reshape the data array according to image dimensions and number of channels
    data = data.reshape(msg.height, msg.width, nchannels)

    # Replace empty values if specified
    if empty_value:
        mask = np.isclose(abs(data), empty_value)
        fill_value = np.percentile(data[~mask], 99)
        data[mask] = fill_value

    # Resize the image to the desired output resolution
    data = cv2.resize(
        data,
        dsize=(output_resolution[0], output_resolution[1]),
        interpolation=cv2.INTER_AREA,
    )

    # Aggregate the color channels if specified
    if aggregate == "littleendian":
        # Aggregate channels in little-endian order
        data = sum([data[:, :, i] * (256**i) for i in range(nchannels)])
    elif aggregate == "bigendian":
        # Aggregate channels in big-endian order
        data = sum([data[:, :, -(i + 1)] * (256**i) for i in range(nchannels)])

    # Adjust the shape of the array based on the number of channels
    if len(data.shape) == 2:
        data = np.expand_dims(data, axis=0)
    else:
        data = np.moveaxis(data, 2, 0)  # Switch to channels-first format

    # Normalize the data if it is RGB
    if is_rgb:
        data = data.astype(np.float32) / (255.0 if aggregate == "none" else 255.0**nchannels)

    return data