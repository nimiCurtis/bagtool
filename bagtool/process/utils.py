import numpy as np

import cv2
from typing import Any, Tuple, List, Dict
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CompressedImage

def get_key_by_value(my_dict, value):
    key = None
    for k, v in my_dict.items():
            if v == value:
                key = k
                break  # Break after finding the key
    return key

def odom_to_numpy(msg:Odometry):
    pos = [ msg.pose.pose.position.x,
        msg.pose.pose.position.y,
        msg.pose.pose.position.z
    ]
    ori = [ msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w]
    lin_vel = [msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z]
    angular_vel = [msg.twist.twist.angular.x,
            msg.twist.twist.angular.y,
            msg.twist.twist.angular.z]
    
    return [np.array(pos), np.array(ori), np.array(lin_vel), np.array(angular_vel)] 

def np_odom_to_xy_yaw(np_odom:np.ndarray)->Any:
    x, y = np_odom[0][0], np_odom[0][1]
    yaw = quat_to_yaw(np_odom[1])
    # return [x,y], yaw
    return {'pos':[x,y], 'yaw': yaw}


def quat_to_yaw(quat: np.ndarray) -> float:
    x, y, z, w = quat[0], quat[1], quat[2], quat[3]
    t3 = 2.0 * (w * z + x * y)
    t4 = 1.0 - 2.0 * (y * y + z * z)
    yaw = np.arctan2(t3, t4)
    return yaw

def image_compressed_to_numpy(msg:CompressedImage)->np.ndarray:
    # Convert CompressedImage to OpenCV Image using np.frombuffer
    np_img = np.frombuffer(msg.data, dtype=np.uint8)
    np_img = cv2.imdecode(np_img, cv2.IMREAD_COLOR)

    return np_img

def image_to_numpy(
    msg, nchannels=3, empty_value=None, output_resolution=None, aggregate="none"
):

    if output_resolution is None:
        output_resolution = (msg.width, msg.height)

    is_rgb = "8" in msg.encoding
    if is_rgb:
        data = np.frombuffer(msg.data, dtype=np.uint8).copy()
    else:
        data = np.frombuffer(msg.data, dtype=np.float32).copy()

    data = data.reshape(msg.height, msg.width, nchannels)

    if empty_value:
        mask = np.isclose(abs(data), empty_value)
        fill_value = np.percentile(data[~mask], 99)
        data[mask] = fill_value

    data = cv2.resize(
        data,
        dsize=(output_resolution[0], output_resolution[1]),
        interpolation=cv2.INTER_AREA,
    )

    if aggregate == "littleendian":
        data = sum([data[:, :, i] * (256**i) for i in range(nchannels)])
    elif aggregate == "bigendian":
        data = sum([data[:, :, -(i + 1)] * (256**i) for i in range(nchannels)])

    if len(data.shape) == 2:
        data = np.expand_dims(data, axis=0)
    else:
        data = np.moveaxis(data, 2, 0)  # Switch to channels-first

    if is_rgb:
        data = data.astype(np.float32) / (
            255.0 if aggregate == "none" else 255.0**nchannels
        )

    return data