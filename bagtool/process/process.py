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
import json
import sys
import os
import yaml
import rosbag
from cv_bridge import CvBridge
import h5py
from .utils import *

class BadReader:

    def __init__(self, bagfile, dst_dataset = None, dst_datafolder_name=None) -> None:
        
        print(f"[INFO]  Reading {bagfile}.")

        self.bagfile = bagfile
        self.cv_bridge = CvBridge()
        

        
        parts = bagfile.split('/')
        
        # If the bag_file contains '/', parts will have more than one element
        if len(parts) > 1:
            self.filename = parts[-1]
            self.dir = '/'.join(parts[:-1])
        else:
            self.filename = bagfile
            self.dir = './'

        data_folder_name = dst_datafolder_name if dst_datafolder_name is not None \
                                            else 'bag-'+self.filename[0:-4]+'-data' 

        if dst_dataset is not None:
            self.dst_datafolder = os.path.join(dst_dataset,data_folder_name)
        else:
            self.dst_datafolder = os.path.join(self.dir,data_folder_name)

        metadata_file_p = os.path.join(self.dst_datafolder,"metadata.json")
        self.metadata = {}
        self.metadata["source_filename"] = self.filename
        self.metadata["source_dir"] = self.dir
        self.metadata["data_dir"] = self.dst_datafolder
        
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
        self.topics_to_keys = {}
        
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
            self.topics_to_keys[topic_data[0]] = topic_key

        if os.path.exists(self.dst_datafolder):
                print(f"[INFO]  Data folder {self.dst_datafolder} already exists. Not creating.")
        else:
            try:
                os.mkdir(self.dst_datafolder)
            except OSError:
                print(f"[ERROR] Failed to create the data folder {self.dst_datafolder}.")
            else:
                print(f"[INFO]  Successfully created the data folder {self.dst_datafolder}.")

        self.raw_data = self._init_raw_data()
        self.aligned_data = self._init_aligned_data()
        self.process_data()
        
        self.metadata['num_of_synced_msgs'] = len(self.aligned_data['dt'])

        print(f"[INFO]  Saving metadata.")
        with open(metadata_file_p, 'w') as file:
            json.dump(self.metadata, file, indent=4)
        


    def _init_raw_data(self):
        dic = {}
        topics_recorded_keys = self.metadata['topics'].keys()
        
        for topic in topics_recorded_keys:
            dic[topic] = {}
            dic[topic]['time'] = []
            dic[topic]['data'] = []

        return dic

    def _init_aligned_data(self, aligned_topics=['odom','rgb']):
        dic = {}
        dic['dt'] = []
        dic['time_elapsed'] = []
        dic['topics'] = {}
        topics_aligned_keys = aligned_topics
        
        for tk in topics_aligned_keys:
            dic['topics'][tk] = []

        return dic

    def process_data(self):

        # get start time of bag in seconds
        currtime = self.reader.get_start_time()
        starttime = currtime
        rate = 20 # <<<<<<<<<<<<<< change it

        for topic, msg, t in self.reader.read_messages(topics=self.topics):
            topic_key = self.topics_to_keys[topic]
            # raw_data{topic_key} = function
            self.raw_data[topic_key]['time'].append(t.to_sec())
            data = self.get_raw_element(topic=topic_key,
                                msg=msg)
            self.raw_data[topic_key]['data'].append(data)

            # aligned data
            if (t.to_sec() - currtime) >= 1.0 / rate:
                # if topic_key in self.aligned_data.keys():
                #     curr_aligned_data[topic_key] = data
                for tk in self.aligned_data['topics'].keys():
                    data_aligned = self.raw_data[tk]['data'][-1]
                    data_aligned = self.get_aligned_element(topic=tk,
                                                    data=data_aligned)
                    
                    self.aligned_data['topics'][tk].append(data_aligned)

                currtime = t.to_sec()
                if len(self.aligned_data['time_elapsed']) > 0:
                    prevtime = self.aligned_data['time_elapsed'][-1]
                    self.aligned_data['dt'].append(currtime-prevtime)
                else:
                    self.aligned_data['dt'].append(0)

                self.aligned_data['time_elapsed'].append(currtime - starttime)

    def get_raw_element(self,topic,msg):
        switcher = {
            "depth": image_compressed_to_numpy,
            "rgb": image_compressed_to_numpy,
            "odom": odom_to_numpy
        }

        case_function = switcher.get(topic)
        return case_function(msg)

    def get_aligned_element(self,topic,data):
        switcher = {
            "depth": lambda x: x,
            "rgb": lambda x: x,
            "odom": np_odom_to_xy_yaw
        }

        case_function = switcher.get(topic)
        return case_function(data)
    
    def get_raw_dataset(self):
        return self.raw_data
    
    def get_aligned_dataset(self):
        return self.aligned_data

    def save_raw(self, data:dict):
        
        folder_path = os.path.join(self.dst_datafolder,'raw_data')
        if not os.path.exists(folder_path):
            os.mkdir(folder_path)
            for tk,v in data.items():
                
                file_path = os.path.join(folder_path,f'raw_{tk}.h5')
                # Create a HDF5 file
                with h5py.File(file_path, 'w') as h5file:
                    
                    # Store the time
                    h5file.create_dataset('time', data=v['time'])
                    
                    # Store the data depend on the topic
                    if tk == 'odom':
                        pos = np.array([vi[0] for vi in v['data']])
                        ori = np.array([vi[1] for vi in v['data']])
                        lin_vel = np.array([vi[2] for vi in v['data']])
                        ang_vel = np.array([vi[3] for vi in v['data']])
                        
                        h5file.create_dataset('pose', data=pos)
                        h5file.create_dataset('ori', data=ori)
                        h5file.create_dataset('lin_vel', data=lin_vel)
                        h5file.create_dataset('ang_vel', data=ang_vel)


                    elif tk in ['rgb','depth']:
                        h5file.create_dataset('data', data=np.array(v['data']))
                
                print(f"[INFO]  {tk} raw data successfully saved.")


    def save_aligned(self, data:dict):
        
        for tk in data['topics'].keys():
            if tk == 'odom':
                filename = 'traj_data'
                file_path = os.path.join(self.dst_datafolder,filename+'.json')
                with open(file_path, 'w') as file:
                    json.dump(data['topics']['odom'], file, indent=4)

            elif tk in ['rgb','depth']:
                folder_path = os.path.join(self.dst_datafolder,'visaul_data')
                if not os.path.exists(folder_path):
                    os.mkdir(folder_path)
                    for i, img in enumerate(self.aligned_data['topics'][tk]):
                        img_name =  os.path.join(folder_path,f'{i}.jpg')
                        cv2.imwrite(img_name, img)

            print(f"[INFO]  {tk} data successfully saved.")


class BagProcess:
    
    def __init__(self) -> None:
        pass

    @staticmethod
    def process_batch(bag_folder_path: str,
                    dst_dataset: str = None,
                    dst_datafolder_name: str = None,
                    save_raw: bool = False):
        
        print(f"[INFO] processing batch - {bag_folder_path}")

        metadata_file_p = os.path.join(bag_folder_path,"metadata.json")

        try:
            if not os.path.exists(metadata_file_p):
                print(f"'metadata.json' not found in {bag_folder_path}.")
                sys.exit("Exiting program.")
        except FileNotFoundError as e:
            print(e)

        # If the file exists, read and return its contents
        with open(metadata_file_p, 'r') as file:
            metadata = json.load(file)

        # Loop through each file in the folder
        for filename in os.listdir(bag_folder_path):
            if (filename.endswith(".bag")):
                if not(filename in metadata["processed_bags"]):
                    # Compress the file using rosbag compress command
                    bagfile = os.path.join(bag_folder_path,filename)
                    bag_reader = BadReader(bagfile=bagfile,
                                        dst_dataset=dst_dataset, 
                                        dst_datafolder_name=dst_datafolder_name)
                    raw_data = bag_reader.get_raw_dataset()
                    aligned_data = bag_reader.get_aligned_dataset()

                    bag_reader.save_aligned(aligned_data)
                    
                    if save_raw:
                        bag_reader.save_raw(raw_data)
                    
                    # add to metadata
                    metadata["processed_bags"].append(filename)
                else:
                    print(f"[INFO] Bag {filename} already processed")

        # Writing the updated data back to the file
        with open(metadata_file_p, 'w') as file:
            json.dump(metadata, file, indent=4)

    @staticmethod
    def process_folder(folder_path: str,
                    dst_dataset: str = None,
                    dst_datafolder_name: str = None,
                    save_raw: bool = False):
        
        print(f"[INFO] processing folder - {folder_path}")
        # Loop through each folder
        for filename in os.listdir(folder_path):
            if filename.startswith("bag_batch"):
                batch_path = os.path.join(folder_path, filename)
                BagProcess.process_batch(bag_folder_path=batch_path,
                                        dst_dataset=dst_dataset,
                                        dst_datafolder_name = dst_datafolder_name,
                                        save_raw=save_raw)

def main():
    bag_batch = '/home/roblab20/catkin_ws/src/zion_ros/zion_zed_ros_interface/bag/bag_batch_2024-01-04-13-52-32'
    dst = '/home/roblab20/dev/bagtool'
    bp = BagProcess()
    bp.process_batch(bag_folder_path=bag_batch,
                    dst_dataset=dst)

if __name__ == "__main__":
    main()