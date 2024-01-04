import numpy as np
import io
import os
import cv2
from typing import Any, Tuple, List, Dict
from sensor_msgs.msg import CompressedImage

def get_key_by_value(my_dict, value):
    key = None
    for k, v in my_dict.items():
            if v == value:
                key = k
                break  # Break after finding the key
    return key


def CompressedImage2Cv(msg:CompressedImage) -> np.ndarray:
    """
    """
    
    # Convert CompressedImage to OpenCV Image using np.frombuffer
    np_img = np.frombuffer(msg.data, dtype=np.uint8)
    cv_img = cv2.imdecode(cv_img, cv2.IMREAD_COLOR)

    return cv_img

def Odom2xy_yaw(msg):
    pass