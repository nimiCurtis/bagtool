# bagtool: Useful ROS Bag Management for Dataset Collection

--- 

## Overview

TBD...

---

## Installation

### Prerequisites

- [Ubuntu 20.04 (Focal Fossa)](https://releases.ubuntu.com/focal/)
- [ROS Noetic](https://wiki.ros.org/noetic/Installation/Ubuntu)

### Build the package

Clone the repo:

```bash
$ git clone https://github.com/nimiCurtis/bagtool
```

Install the dependencies by:

```bash
$ pip install -r requirements.txt 
```

Build the package:
```bash
$ pip install -e . 
```

---

## Usage

**Important: This package is designed to work with bag files structured in accordance with the recording conventions established by the [zion_zed_ros_interface](https://github.com/nimiCurtis/zion_zed_ros_interface). Please ensure compatibility with this structure for optimal performance. for more info on the structure use the given link**

### Main Tool - Process

The main tool of this package is the process tool, which process ROS bag files from a given folder path and saving the data in useful formats and structures for future works.

#### You can use the tool in two ways:

- From the CLI : 



<!-- #### Using from the command-line -->
    bagtool process [-h] [-b BATCH] [-f FOLDER] [-d DST] [-n NAME]
                        [--no_raw]

    optional arguments:
    -h, --help            show this help message and exit
    -b BATCH, --batch BATCH
                            path to a bag batch folder
    -f FOLDER, --folder FOLDER
                            path to a bag folder consisting bag batches
    -d DST, --dst DST     path to a dataset destination folder
    -n NAME, --name NAME  a custom name for the datafolder
    --no_raw              not saving raw data


- And From a custom script using the bagtool.process module 

#### Example usecase:

Assuming you have a folder with a bag batch folder (see the *important* note) at `<some_path>/bag` folder. And you'd like to set your entire dataset on `<some_path>/dataset`. 

**Using CLI:**
<!-- #### Using from the command-line -->
    bagtool process -f <some_path>/bag --dst <some_path>/dataset

**Using custom script:**
```python
from bagtool.process.process import BagProcess as bp

source = '<some_path>/bag'
dest = '<some_path>/dataset'
bp.process_folder(source,dest)
```

**Dataset structure:**

The resulted dataset from the example above will be as follow:


```
├── dataset
│   ├── <name_of_bag_batch1>
|   |    ├── raw_data
│   │       ├── raw_<topic1>.h5
│   │       ├── ...
│   │       └── raw_<topicN>.h5
|   |    ├── visual_data
│   │       ├── 0.jpg
│   │       ├── ...
│   │       └── T.jpg
│   │    ├── metadata.json
│   │    ├── traj_data.json
│   │    └── traj_sample.mp4
│   ...
└── └── <name_of_bag_batchN>
         ├── raw_data
            ├── raw_<topic1>.h5
            ├── ...
            └── raw_<topicN>.h5
         ├── visual_data
            ├── 0.jpg
            ├── ...
            └── T.jpg
         ├── metadata.json
         ├── traj_data.json
         └── traj_sample.mp4
```  

---

### TODO: 
By priority Top-Down:

- [x] Check ros-to-numpy images encoding decoding include depth images.
- [x] Check using the regular Image sensor msgs
- [ ] Process based on params.yaml file
- [ ] Check using the compressed Image sensor msgs
- [ ] Clean the main code in process.py
- [ ] Add arguments for saving animation
- [ ] Syncing more then two topics?
- [ ] Support more topics types
- [ ] Add arrow to the traj animation indicating the yaw direction
- [ ] Add deltas of positions and yaw to the traj_data.json ? 
