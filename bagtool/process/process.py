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
import json
import sys
import os
import yaml

# Third party libraries
from cv_bridge import CvBridge
import h5py
import matplotlib.pyplot as plt
import moviepy.video.io.VideoFileClip as mp
from tqdm import tqdm
from matplotlib import animation

# ROS libraries
import rosbag

# Custom libraries
from bagtool.process.utils import *


class BadReader:
    """
    A class to read and process data from a ROS bag file.

    Attributes:
        bagfile (str): Path to the ROS bag file.
        filename (str): Name of the bag file.
        dir (str): Directory of the bag file.
        dst_datafolder (str): Destination folder for processed data.
        metadata (dict): Metadata information of the bag file.
        topics (list): List of topics in the bag file.
        topics_to_keys (dict): Mapping of topics to their respective keys.
        message_types (list): List of message types in the topics.
        n_messages (list): List of message counts for each topic.
        frequency (list): List of frequencies for each topic.
        raw_data (dict): Raw data extracted from the bag file.
        aligned_data (dict): Aligned data based on synchronization rate.
        sync_rate (float): Synchronization rate for data alignment.
    
    Methods:
        __init__: Initializes the BadReader instance.
        _init_raw_data: Initializes the raw data structure.
        _init_aligned_data: Initializes the aligned data structure.
        _get_sync_rate: Calculates the synchronization rate.
        _process_data: Processes the data from the bag file.
        get_raw_element: Retrieves raw data elements.
        get_aligned_element: Retrieves aligned data elements.
        get_raw_dataset: Returns the raw data dataset.
        get_aligned_dataset: Returns the aligned data dataset.
        save_raw: Saves raw data to a file.
        save_aligned: Saves aligned data to files.
        save_traj_video: Generates and saves a trajectory video.
    """
    def __init__(self, bagfile, dst_dataset = None, dst_datafolder_name=None, config = None) -> None:
        """
        Initialize the BadReader object with a ROS bag file and optional destination dataset and folder names.

        This method sets up the necessary attributes for reading and processing the bag file, including extracting metadata,
        topics information, and initializing data struct
            bagfile (str): Path to the ures for raw and aligned data.

        Args:ROS bag file to be read.
            dst_dataset (str, optional): Destination dataset path. Default is None.
            dst_datafolder_name (str, optional): Name of the folder to store processed data. Default is None.
        """
        
        print(f"\n[INFO]  Reading {bagfile}.")

        self.bagfile = bagfile

        parts = bagfile.split('/')
        
        # If the bag_file contains '/', parts will have more than one element
        if len(parts) > 1:
            self.filename = parts[-1]
            self.dir = '/'.join(parts[:-1])
        else:
            self.filename = bagfile
            self.dir = './'

        data_folder_name_suffix = 'bag-'+self.filename[0:-4]+'-data'
        data_folder_name = dst_datafolder_name + '_' + data_folder_name_suffix if dst_datafolder_name is not None \
                                            else data_folder_name_suffix 

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
        topic_tuple = info.topics.values()
        self.topics = info.topics.keys()
        self.topics_to_keys = {}
        
        self.message_types = []
        for t1 in topic_tuple: self.message_types.append(t1.msg_type)

        self.n_messages = []
        for t1 in topic_tuple: self.n_messages.append(t1.message_count)

        self.frequency = []
        for t1 in topic_tuple: self.frequency.append(t1.frequency)
        
        keys=['Topic', 'Type', 'Message Count', 'Frequency']
        topics_zipped = list(zip(self.topics,self.message_types, self.n_messages, self.frequency))

        # Initialize the 'topics' dictionary in metadata
        self.metadata['topics'] = {}

        # Iterate over topics_zipped and populate the metadata
        for topic_data in topics_zipped:
            topic_key = get_key_by_value(record_config["topics"],topic_data[0])

            # get info per topic
            self.metadata['topics'][topic_key] = dict(zip(keys, topic_data))
            self.topics_to_keys[topic_data[0]] = topic_key

        if os.path.exists(self.dst_datafolder):
                print(f"[INFO]  Data folder {self.dst_datafolder} already exists. Not creating.")
        else:
            try:
                os.mkdir(self.dst_datafolder)
            except OSError:
                print(f"[ERROR] Failed to create the data folder {self.dst_datafolder}.")
                sys.exit("Exiting program.")
            else:
                print(f"[INFO]  Successfully created the data folder {self.dst_datafolder}.")


        self.raw_data = self._init_raw_data()
        
        self.A = None
        self.aligned_data = self._init_aligned_data(aligned_topics = config.get("aligned_topics") if config is not None \
                                                    else None)
        
        self.sync_rate = config.get("sync_rate") if config is not None else self._get_sync_rate() 
        self.metadata['sync_rate'] = self.sync_rate

        self._process_data(sync_rate=self.sync_rate,
                        pre_truncated=config.get("pre_truncated"),
                        post_truncated=config.get("post_truncated"))

        self.metadata['num_of_synced_msgs'] = len(self.aligned_data['dt'])
        self.metadata['demonstrator'] = record_config['recording']['demonstrator']

        print(f"[INFO]  Saving metadata.")
        with open(metadata_file_p, 'w') as file:
            json.dump(self.metadata, file, indent=4)


    def _init_raw_data(self):
        """
        Initialize the structure for storing raw data extracted from the bag file.

        This method sets up a dictionary to store time and data for each recorded topic in the bag file.

        Returns:
            dict: A dictionary with keys for each topic and sub-keys for 'time' and 'data'.
        """
        dic = {}
        topics_recorded_keys = self.metadata['topics'].keys()
        
        for topic in topics_recorded_keys:
            dic[topic] = {}
            dic[topic]['time'] = []
            dic[topic]['data'] = []

        return dic

    def _init_aligned_data(self, aligned_topics=['odom','rgb']):
        """
        Initialize the structure for storing aligned data based on specified topics.

        This method sets up a dictionary to store delta time, time elapsed, and data for each aligned topic.

        Args:
            aligned_topics (list of str): List of topics to be aligned. Defaults to ['odom', 'rgb'].

        Returns:
            dict: A dictionary with keys for delta time, time elapsed, and each aligned topic.
        """
        dic = {}
        dic['dt'] = []
        dic['time_elapsed'] = []
        dic['topics'] = {}
        topics_aligned_keys = aligned_topics

        for tk in topics_aligned_keys:
            dic['topics'][tk] = []

        return dic

    def _get_sync_rate(self):
        """
        Determine the synchronization rate for aligning data from different topics.

        Returns:
            float: The synchronization rate for data alignment.
        """
        min_freq = np.inf
        
        for tk in self.aligned_data['topics'].keys():
            if tk in self.metadata['topics'].keys():
                freq = self.metadata['topics'][tk]['Frequency']
                
                if freq < min_freq:
                    min_freq = freq

        return min_freq

    def _process_data(self, sync_rate, pre_truncated, post_truncated):
        """
        Process data from the ROS bag file, aligning it according to the synchronization rate.

        This method reads messages from the bag file, extracts raw data, and aligns them based on the sync rate.
        It updates the raw_data and aligned_data attributes of the object.
        """

        # get start time of bag in seconds
        currtime = self.reader.get_start_time()
        starttime = currtime
        end_time = self.reader.get_end_time()
        
        for topic, msg, t in self.reader.read_messages(topics=self.topics):
            topic_key = self.topics_to_keys[topic]
            # raw_data{topic_key} = function

            if (topic_key in self.aligned_data["topics"].keys()): ## change to all topics!!! 
                self.raw_data[topic_key]['time'].append(t.to_sec())
                data = self.get_raw_element(topic=topic_key,
                                    msg=msg)
                self.raw_data[topic_key]['data'].append(data)

                # aligned data
            if (topic_key in self.aligned_data["topics"].keys()): ## here just the aligned
                if ((t.to_sec() - currtime) >= 1.0 / sync_rate):
                    
                    if((currtime > starttime + pre_truncated) and (currtime < end_time - post_truncated)    \
                        and (t.to_sec() > starttime + pre_truncated) and (t.to_sec() < end_time - post_truncated)):

                        for tk in self.aligned_data['topics'].keys():
                            data_entry = self.raw_data[tk]['data'][-1]
                            data_aligned = self.get_aligned_element(topic=tk,
                                                            data=data_entry)

                            self.aligned_data['topics'][tk].append(data_aligned)

                    currtime = t.to_sec()
                    if len(self.aligned_data['time_elapsed']) > 0:
                        prevtime = self.aligned_data['time_elapsed'][-1]
                        self.aligned_data['dt'].append(currtime-prevtime)
                    else:
                        self.aligned_data['dt'].append(0)

                    self.aligned_data['time_elapsed'].append(currtime - starttime)

        # def init_aligned_dic(self,dic,tk):
    #     if(tk == 'odom'):
    #         dic = {tk:{}}
    #         for frame in ['gt_frame','odom_frame','relative_frame']:
    #             dic['topics'][tk].update({frame:{'position':[],'yaw':[]}})

    #     else:
    #         dic = {tk:[]}
            
    # def update_aligned_dic(self,tk,data):
        
    #     if(tk == 'odom'):
    #         for type in ['position','yaw']:
    #             for frame in ['gt_frame','odom_frame','relative_frame']:
    #                 self.aligned_data['topics'][tk][frame][type].append(data[frame][type])
    #     else:
    #         self.aligned_data['topics'][tk].append(data)
        
    def get_raw_element(self,topic,msg):
        """
        Retrieve a raw data element based on the topic and message.

        This method processes a ROS message from a given topic and converts it into a usable format.

        Args:
            topic (str): The topic of the data.
            msg (rosbag message): The ROS message to be processed.

        Returns:
            Varies: The processed data element, the type depends on the topic.
        """
        
        switcher = {
            # "depth": image_compressed_to_numpy,
            "depth": image_to_numpy,
            "rgb": image_to_numpy,
            "odom": odom_to_numpy,
            "target_object": object_detection_to_dic
        }

        case_function = switcher.get(topic)
        return case_function(msg)

    def get_aligned_element(self,topic,data):
        """
        Process and align a data element based on the topic.

        Args:
            topic (str): The topic of the data.
            data: The data element to be processed.

        Returns:
            Varies: The aligned data element, the type depends on the topic.
        """
        switcher = {
            "depth": lambda x: x,
            "rgb": lambda x: x,
            "odom": np_odom_to_xy_yaw,
            "target_object": lambda x:x
        }

        case_function = switcher.get(topic)
        
        if(topic == "odom"):
            prev_data = self.aligned_data['topics'][topic][-1] if len(self.aligned_data['topics'][topic])>0 else None
            
            if prev_data is None:
                x_start, y_start, yaw_start = data[0][0], data[0][1], quat_to_yaw(data[1])
                self.A = get_transform_to_start(x_start, y_start, yaw_start)

            return case_function(data, prev_data, self.A)
        else:
            return case_function(data)

    def get_raw_dataset(self):
        """
        Return the entire raw data set.

        Returns:
            dict: The raw data set.
        """
        return self.raw_data

    def get_aligned_dataset(self):
        """
        Return the entire aligned data set.

        This method provides access to the data aligned based on the synchronization rate.

        Returns:
            dict: The aligned data set.
        """
        return self.aligned_data

    def save_raw(self, data:dict):
        """
        Save the raw data to files in a specified format.

        This method writes the raw data to files, organizing them based on the topics.

        Args:
            data (dict): The raw data to be saved.
        """

        folder_path = os.path.join(self.dst_datafolder,'raw_data')
        if not os.path.exists(folder_path):
            os.mkdir(folder_path)
            for tk,v in data.items():
                file_path = os.path.join(folder_path,f'{tk}_raw.h5')

                # Create a HDF5 file
                with h5py.File(file_path, 'w') as h5file:
                    # Store the time
                    h5file.create_dataset('time', data=v['time'])
                    
                    # Store the data depend on the topic
                    if tk == 'odom':
                        
                        data_arrays = {}
                        num_keys = len(v['data'][0])  # Assuming all data entries have the same number of elements
                        
                        # Initialize arrays for each key with the correct length
                        # indexes correspond to : ["position","orientation","linear_vel","angular_vel"]
                        for i in range(num_keys):
                            data_arrays[i] = []

                        # Process each entry in v['data'] only once
                        for data_entry in v['data']:
                            for i in range(num_keys):
                                data_arrays[i].append(data_entry[i])

                        # Create datasets for each key
                        for i in range(num_keys):
                            h5file.create_dataset(f'data_{i}', data=np.array(data_arrays[i]))

                    elif tk in ['rgb', 'depth']: ## change to depth
                        h5file.create_dataset('data', data=np.array(v['data']))

                    elif tk == "target_object":
                        if len(v['data'])>0:
                            data_arrays = {}
                            keys = list(v['data'][0].keys())  # Assuming all data entries have the same number of elements
                            
                            # Initialize arrays for each key with the correct length
                            # indexes correspond to : ["position","orientation","linear_vel","angular_vel"]
                            for key in keys:
                                data_arrays[key] = []

                            # Process each entry in v['data'] only once
                            for data_entry in v['data']:
                                for key in keys:
                                    data_arrays[key].append(data_entry[key])

                            # Create datasets for each key
                            for key in keys:
                                h5file.create_dataset(f'data_{key}', data = data_arrays[key] if type(data_arrays[key][0]==str) else np.array(data_arrays[key]) )

                h5file.close()
                print(f"[INFO]  {tk} raw data successfully saved.")

    def save_aligned(self, data:dict):
        """
        Save the aligned data to files in a specified format.

        This method writes the aligned data to files, organizing them based on the topics.

        Args:
            data (dict): The aligned data to be saved.
        """

        for tk in data['topics'].keys():
            if tk == 'odom':
                
                filename = 'traj_robot_data'
                file_path = os.path.join(self.dst_datafolder,filename+'.json')
                
                dic_to_save = {}
                for frame in ['gt_frame','odom_frame','relative_frame']:
                    dic_to_save.update({frame:{'position':[],'yaw':[]}})
                    for type in ['position','yaw']:
                        for i in range(len(data['topics'][tk])):
                            dic_to_save[frame][type].append(data['topics']['odom'][i][frame][type])

                with open(file_path, 'w') as file:
                    json.dump(dic_to_save, file, indent=4)

            if tk == 'target_object':
                
                filename = 'traj_target_data'
                file_path = os.path.join(self.dst_datafolder,filename+'.json')
                
                with open(file_path, 'w') as file:
                    json.dump(data['topics'][tk], file, indent=4)

            elif tk in ['rgb','depth']:
                
                parent_folder_path = os.path.join(self.dst_datafolder,'visual_data')
                if not os.path.exists(parent_folder_path):
                    os.mkdir(parent_folder_path)
                
                child_folder_path = os.path.join(parent_folder_path,tk)
                if not os.path.exists(child_folder_path):
                    os.mkdir(child_folder_path)
                
                for i, img in enumerate(self.aligned_data['topics'][tk]):
                    img_name =  os.path.join(child_folder_path,f'{i}.jpg')
                    cv2.imwrite(img_name, img)

            print(f"[INFO]  {tk} data successfully saved.")

    def save_traj_video(self,data:dict, rate=10):
        """
        Generate and save a video visualizing the trajectory data.

        This method creates a video to visualize the trajectory using the aligned data and saves it to a file.

        Args:
            data (dict): The aligned data used for creating the trajectory video.
            rate (int): The rate at which frames are sampled from the data. Defaults to 10.
        """

        keys = data['topics'].keys()

        images = None
        times = np.array(data['time_elapsed'])
        
        for key in keys:

            if key == 'odom':    
                # Extracting position and yaw values into separate numpy arrays
                odom_traj = data['topics'][key]
                positions = np.array([item['odom_frame']['position'] for item in odom_traj])
                
                yaws = np.array([item['odom_frame']['yaw'] for item in odom_traj])
                
            if key == 'target_object':
                target_in_cam = data['topics'][key]
                target_positions = np.array([item.get('position') for item in target_in_cam])
                target_bbox3d = np.array([item.get('bbox3d') for item in target_in_cam])
                
                # Take just the x-y coordinates of a box from top view 
                target_bbox3d = target_bbox3d[:,:4,:2]
            if key == "rgb":
                
                if images is not None:
                    images = np.concatenate((images, data['topics'][key]), axis=1)
                else:
                    images = np.array(data['topics'][key])

            if key == "depth":
                depth_images = np.array(data['topics'][key])
                shape = depth_images.shape
                depth_rgb_images = np.zeros((shape[0], shape[1], shape[2], 3), dtype=np.uint8)
                
                # Copy grayscale values into each color channel
                depth_rgb_images[:,:,:,0] = depth_images  # Red channel
                depth_rgb_images[:,:,:,1] = depth_images  # Green channel
                depth_rgb_images[:,:,:,2] = depth_images  # Blue channel

                if images is not None:
                    images = np.concatenate((images, depth_rgb_images), axis=2)
                else:
                    images = depth_rgb_images


        fig = plt.figure(figsize=[16, 12])
        grid = plt.GridSpec(12, 17, hspace=0.2, wspace=0.2)
        
        ax_image = fig.add_subplot(grid[1:6, :], title=f"Scene Image")
        ax_trajectory = fig.add_subplot(grid[7:, :], title="Trajectory", xlabel="Y [m]", ylabel="X [m]")
        
        
        x_lim = np.max([abs(np.max(target_positions[:,1] + positions[:,1])),  abs(np.min(target_positions[:,1] + positions[:,1]) )])
        ax_trajectory.set_xlim(xmin=-x_lim-0.5,xmax=x_lim + 0.5)
        
        y_lim = np.max([abs(np.max(target_positions[:,0] + positions[:,0])),  abs(np.min(target_positions[:,0] + positions[:,0]) )])
        ax_trajectory.set_ylim(ymin=-y_lim-0.5,ymax=y_lim + 0.5)

        ax_trajectory.invert_xaxis()
        ax_trajectory.grid(True)

        aggregated_positions = []
        aggregated_target_positions = []
        Frames = []
        for i in range(len(images)):
            
            if i % rate == 0:
                
                aggregated_positions.append(positions[i])
                
                corners = None
                if((target_positions[i]!=np.zeros_like(target_positions[i])).all()):

                    
                    aggregated_target_positions.append(np.array([[np.cos(yaws[i]), 0],[0, np.sin(yaws[i])]]) @ positions[i].T + target_positions[i][:2])
                    # aggregated_target_positions.append(target_positions[i][:2]+positions[i])
                    
                    
                    # corners = target_bbox3d[i] + positions[i]
                    corners = np.array([[np.cos(yaws[i]), 0],[0, np.sin(yaws[i])]]) @ positions[i].T + target_bbox3d[i] 

                frame = TrajViz.visualization(robot_position = np.array(aggregated_positions),
                                yaw=yaws[i],
                                curr_image=images[i],
                                time=times[i],
                                frame_idx = i,
                                ax_image=ax_image,
                                ax_trajectory=ax_trajectory,
                                target_position=np.array(aggregated_target_positions),
                                corners = corners)

                Frames.append(frame)

        ani = animation.ArtistAnimation(fig=fig,
                                        artists=Frames,
                                        blit=True,
                                        interval=200)

        TrajViz.save_animation(ani=ani,dest_dir=self.dst_datafolder,file_name="traj_sample")


class BagProcess:
    """
    A class for processing ROS bag files in batches or entire folders.

    This class provides static methods to process ROS bag files from a given folder path. It supports processing
    individual batches of bag files and entire folders containing multiple batches.

    Methods:
        process_batch: Processes a batch of ROS bag files from a specified folder.
        process_folder: Processes multiple batches of ROS bag files from a specified folder.
    """
    
    def __init__(self) -> None:
        pass

    @staticmethod
    def process_batch(bag_folder_path: str,
                    dst_dataset: str = None,
                    dst_datafolder_name: str = None,
                    save_raw: bool = False,
                    save_video: bool = True,
                    config: dict = None):
        """
        Process a batch of ROS bag files located in a specified folder.

        This method reads the metadata from the specified folder and processes each unprocessed bag file found. It can
        optionally save raw data and generate trajectory videos from the processed data.

        Args:
            bag_folder_path (str): Path to the folder containing ROS bag files.
            dst_dataset (str, optional): Destination dataset path. Defaults to None.
            dst_datafolder_name (str, optional): Name of the folder to store processed data. Defaults to None.
            save_raw (bool, optional): Flag to save raw data from the bag files. Defaults to False.
            save_video (bool, optional): Flag to save trajectory videos from the bag files. Defaults to True.
        """

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

        # Get all filenames in the folder path
        filenames = [filename for filename in os.listdir(bag_folder_path) if filename.endswith(".bag")]

        # Loop through each file in the folder
        for filename in tqdm(filenames, desc="Processing files"):
            # if (filename.endswith(".bag")):
                if not(filename in metadata["processed_bags"]):
                    # Compress the file using rosbag compress command
                    bagfile = os.path.join(bag_folder_path,filename)
                    bag_reader = BadReader(bagfile=bagfile,
                                        dst_dataset=dst_dataset, 
                                        dst_datafolder_name=dst_datafolder_name,
                                        config=config)
                    
                    raw_data = bag_reader.get_raw_dataset()
                    aligned_data = bag_reader.get_aligned_dataset()

                    bag_reader.save_aligned(aligned_data)
                    
                    is_save_raw = save_raw
                    if is_save_raw:
                        bag_reader.save_raw(raw_data)
                    
                    save_video = save_video if config is None else config.get("save_vid", True)
                    if save_video:
                        bag_reader.save_traj_video(aligned_data)
                        
                    # add to metadata
                    metadata["processed_bags"].append(filename)
                else:
                    print(f"[INFO] Bag {filename} already processed")


        # Writing the updated data back to the file
        with open(metadata_file_p, 'w') as file:
            json.dump(metadata, file, indent=4)


    @staticmethod
    def process_folder(folder_path: str =None,
                    dst_dataset: str = None,
                    dst_datafolder_name: str = None,
                    save_raw: bool = False,
                    config:dict = None
                    ):
        
        """
        Process multiple batches of ROS bag files from a specified folder.

        This method iterates through each subfolder within the specified folder, identified as a batch, and processes
        the ROS bag files within using the `process_batch` method.

        Args:
            folder_path (str): Path to the folder containing multiple batches of ROS bag files.
            dst_dataset (str, optional): Destination dataset path. Defaults to None.
            dst_datafolder_name (str, optional): Name of the folder to store processed data. Defaults to None.
            save_raw (bool, optional): Flag to save raw data from the bag files. Defaults to False.
            config (dict, optional): Configuration dictionary containing necessary arguments. Defaults to None.

        """
        print(f"[INFO] processing folder - {folder_path}")

        # Ensure that either config is provided or the other arguments, but not both
        assert (config is None) != (folder_path is None), "Either provide a config dictionary or the individual arguments, but not both"

        if config:
            # Extract arguments from config
            folder_path = config.get('bags_folder')
            dst_dataset = config.get('destination_folder')
            save_raw = config.get('save_raw', True)  # Default to False if not in config

        # Loop through each folder
        batches = [batch for batch in os.listdir(folder_path)]

        # if config is not none:
        # use the bag_folder_path
        # use the destination_path
        # use save_raw 

        for batch in tqdm(batches, desc="Processing batches"):
                batch_path = os.path.join(folder_path, batch)
                record_config_file = os.path.join(batch_path,
                                        "configs",
                                        "record.yaml")

                # Open the record config file
                with open(record_config_file, 'r') as file:
                    record_config = yaml.safe_load(file)
                
                demonstrator = record_config["recording"]["demonstrator"]
                dst_datafolder_name = demonstrator if dst_datafolder_name is None else dst_datafolder_name 
                
                BagProcess.process_batch(bag_folder_path=batch_path,
                                        dst_dataset=dst_dataset,
                                        dst_datafolder_name = dst_datafolder_name,
                                        save_raw=save_raw,
                                        config=config.get(demonstrator)) ## why I did it like this ????????


def main():

    bp = BagProcess()

    config_path = 'process_bag_config.yaml'
    with open(config_path, 'r') as file:
            process_config = yaml.safe_load(file)
    
    bp.process_folder(config=process_config)

if __name__ == "__main__":
    main()