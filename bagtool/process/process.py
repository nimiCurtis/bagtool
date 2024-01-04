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
from pathlib import Path
import sys
import os
import yaml
import rosbag
from cv_bridge import CvBridge

from utils import *

class BadReader:

    def __init__(self, bagfile) -> None:
        self.bagfile = bagfile
        self.cv_bridge = CvBridge()
        
        self.metadata = {}
        
        parts = bagfile.split('/')
        
        # If the bag_file contains '/', parts will have more than one element
        if len(parts) > 1:
            self.filename = parts[-1]
            self.dir = '/'.join(parts[:-1])
        else:
            self.filename = bagfile
            self.dir = './'
        
        self.datafolder = bagfile[0:-4]

        self.metadata["source_filename"] = self.filename
        self.metadata["source_dir"] = self.dir
        self.metadata["data_dir"] = self.datafolder
        
        record_config_file = os.path.join(self.dir,
                                        "configs",
                                        "record.yaml")
        # Open the record config file
        with open(record_config_file, 'r') as file:
            record_config = yaml.safe_load(file)
        
        
        try:
            self.reader = rosbag.Bag(self.bagfile)
        except rosbag.ROSBagException as e:
            print(e)
            print(f"Error loading {self.bagfile}.")
            sys.exit("Exiting program.")

        info = self.reader.get_type_and_topic_info()
        self.topic_tuple = info.topics.values()
        self.topics = info.topics.keys()
        
        self.message_types = []
        for t1 in self.topic_tuple: self.message_types.append(t1.msg_type)

        self.n_messages = []
        for t1 in self.topic_tuple: self.n_messages.append(t1.message_count)

        self.frequency = []
        for t1 in self.topic_tuple: self.frequency.append(t1.frequency)
        
        keys=['Topic', 'Type', 'Message Count', 'Frequency']
        self.topics_zipped = list(zip(self.topics,self.message_types, self.n_messages, self.frequency))
        
        # Initialize the 'topics' dictionary in metadata
        self.metadata['topics'] = {}

        # Iterate over topics_zipped and populate the metadata
        for topic_data in self.topics_zipped:
            topic_key = get_key_by_value(record_config["topics"],topic_data[0])
            # Create a key like 'topic1', 'topic2', etc.
            self.metadata['topics'][topic_key] = dict(zip(keys, topic_data))
        
        if os.path.exists(self.datafolder):
                print(f"[INFO]  Data folder {self.datafolder} already exists. Not creating.")
        else:
            try:
                os.mkdir(self.datafolder)
            except OSError:
                print(f"[ERROR] Failed to create the data folder {self.datafolder}.")
            else:
                print(f"[INFO]  Successfully created the data folder {self.datafolder}.")

    def get_aligned_data(self):
        data_aligned = {}
        
        synced_imdata = []
        synced_odomdata = []
        # get start time of bag in seconds
        currtime = self.reader.get_start_time()
        starttime = currtime

        curr_imdata = None
        curr_odomdata = None
        times = []
        a = 0
        for topic, msg, t in self.reader.read_messages(topics=self.topics):
            a+=1
            # if topic == 
            #     curr_imdata = msg
            # # elif topic == odomtopic:
            # #     curr_odomdata = msg
            # # if (t.to_sec() - currtime) >= 1.0 / rate:
            # #     if curr_imdata is not None and curr_odomdata is not None:
            # #         synced_imdata.append(curr_imdata)
            # #         synced_odomdata.append(curr_odomdata)
            # #     currtime = t.to_sec()
            # #     times.append(currtime - starttime)

        return data_aligned
    
    def save_raw_data(self):
        pass


    
class BagProcess:
    
    def __init__(self) -> None:
        pass

    @staticmethod
    def process_batch(bag_folder_path: str):
        pass
    
    @staticmethod
    def process_folder(folder_path: str):
        pass





def main():
    bag_file = '/home/roblab20/catkin_ws/src/zion_ros/zion_zed_ros_interface/bag/bag_batch_2024-01-04-13-52-32/2024-01-04-13-54-12.bag'
    
    br = BadReader(bagfile=bag_file)
    s = br.get_aligned_data()

if __name__ == "__main__":
    main()