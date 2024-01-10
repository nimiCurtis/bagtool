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
import rosbag
from cv_bridge import CvBridge
import h5py
import matplotlib.pyplot as plt
import moviepy.video.io.VideoFileClip as mp
from tqdm import tqdm
from matplotlib import animation

from bagtool.process.utils import *
# from utils import *
# from .utils import *

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
        
        self.sync_rate = self._get_sync_rate() ## <<<<<< or from a params file 
        self.metadata['sync_rate'] = self.sync_rate
        
        self._process_data()
        self.metadata['num_of_synced_msgs'] = len(self.aligned_data['dt'])
        self.metadata['demonstrator'] = record_config['recording']['demonstrator']

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
    
    def _get_sync_rate(self):
        min_freq = np.inf
        
        for tk in self.aligned_data['topics'].keys():
            if tk in self.metadata['topics'].keys():
                freq = self.metadata['topics'][tk]['Frequency']
                
                if freq < min_freq:
                    min_freq = freq
        
        return min_freq

    def _process_data(self):

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
            if (t.to_sec() - currtime) >= 1.0 / self.sync_rate:
                # if topic_key in self.aligned_data.keys():
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
            # "depth": image_compressed_to_numpy,
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

                    elif tk in ['rgb']: ## change to depth
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

    
    def save_traj_video(self,data:dict, rate=10):
        
        # Extracting position and yaw values into separate numpy arrays

        odom_traj = data['topics']['odom']
        positions = np.array([item['pos'] for item in odom_traj])
        yaws = np.array([item["yaw"] for item in odom_traj])
        times = np.array(data['time_elapsed'])
        images = np.array(data['topics']['rgb'])

        fig = plt.figure(figsize=[16, 12])
        grid = plt.GridSpec(12, 17, hspace=0.2, wspace=0.2)
        
        ax_image = fig.add_subplot(grid[1:6, :], title=f"Scene Image")
        ax_trajectory = fig.add_subplot(grid[7:, :], title="Trajectory", xlabel="Y [m]", ylabel="X [m]")
        ax_trajectory.invert_xaxis()

        # ax_title = fig.add_subplot(grid[:1, :])
        
        aggregated_positions = []
        Frames = []
        for i in range(len(images)):
            
            if i % rate == 0:
                aggregated_positions.append(positions[i])
                frame = self.visualization(location = np.array(aggregated_positions),
                                yaw=yaws[i],
                                curr_image=images[i],
                                time=times[i],
                                frame_idx = i,
                                ax_image=ax_image,
                                ax_trajectory=ax_trajectory)# ,
                                # ax_title = ax_title)
                Frames.append(frame)
        
        ani = animation.ArtistAnimation(fig=fig,
                                        artists=Frames,
                                        interval=200)
        
        self.save_animation(ani=ani,dest_dir=self.dst_datafolder,file_name="traj_vid")
        
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
        :param basedir: the parent dir of the animation dir.
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

class BagProcess:
    
    def __init__(self) -> None:
        pass

    @staticmethod
    def process_batch(bag_folder_path: str,
                    dst_dataset: str = None,
                    dst_datafolder_name: str = None,
                    save_raw: bool = False,
                    save_video: bool = True):
        
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
                                        dst_datafolder_name=dst_datafolder_name)
                    raw_data = bag_reader.get_raw_dataset()
                    aligned_data = bag_reader.get_aligned_dataset()

                    bag_reader.save_aligned(aligned_data)
                    
                    save_raw = True
                    if save_raw:
                        bag_reader.save_raw(raw_data)
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
    def process_folder(folder_path: str,
                    dst_dataset: str = None,
                    dst_datafolder_name: str = None,
                    save_raw: bool = False):
        
        print(f"[INFO] processing folder - {folder_path}")
        
        # Loop through each folder
        batches = [batch for batch in os.listdir(folder_path) if batch.startswith("bag_batch")]

        for batch in tqdm(batches, desc="Processing batches"):
                batch_path = os.path.join(folder_path, batch)
                BagProcess.process_batch(bag_folder_path=batch_path,
                                        dst_dataset=dst_dataset,
                                        dst_datafolder_name = dst_datafolder_name,
                                        save_raw=save_raw)

def main():
    bag_batch_16comp = '/home/roblab20/catkin_ws/src/zion_ros/zion_zed_ros_interface/bag/bag_batch_2024-01-04-13-52-32'
    bag_batch_16uncomp = '/home/roblab20/catkin_ws/src/zion_ros/zion_zed_ros_interface/bag/bag_batch_16_uncomp_2024-01-10-12-02-02'
    bag_batch_32uncomp = '/home/roblab20/catkin_ws/src/zion_ros/zion_zed_ros_interface/bag/bag_batch_32_uncomp_2024-01-10-12-03-11'
    dst = '/home/roblab20/dev/bagtool/dataset'
    bp = BagProcess()
    
    # bp.process_batch(bag_folder_path=bag_batch_16comp,
    #                 dst_dataset=dst)
    bp.process_folder(folder_path='/home/roblab20/catkin_ws/src/zion_ros/zion_zed_ros_interface/bag',
                    dst_dataset=dst)
    a=1

if __name__ == "__main__":
    main()