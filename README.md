# MR-COGraphs
## Code
Coming soon!

## Video
🎥 [Download the video](https://github.com/efc-robot/MR-COGraphs/blob/main/videos/MR-COGraphs_video.mp4)

<details>
  <summary>Watch the demo video</summary>

[![Watch the video](https://github.com/user-attachments/assets/9f9e7811-f0fe-4df8-9776-a64e666f1750)](https://github.com/user-attachments/assets/20a00b0f-ec10-4b30-a2d3-5c6a9f5ea157)

</details>

## Dataset
### Isaac Small & Large Environment

<img src="https://github.com/efc-robot/MR-COGraphs/blob/main/pictures/isaac_env.png" alt="描述文本" width="500">

We provide both small and large environments as USD files, which can be [downloaded](https://cloud.tsinghua.edu.cn/f/b4e29359c3d245339fcc/?dl=1)  and opened in the [Isaac Sim](https://developer.nvidia.com/isaac/sim) platform.

Once loaded, you can generate rosbag files using the following command:
```bash
# single robot example
rosbag record /clock /robot1/camera_info_left \
/robot1/depth_left /robot1/odom /robot1/rgb_left \
/robot1/imu /robot1/scan /tf -o rosbag_name.bag
```

```bash
# two robots example
rosbag record /clock /robot1/camera_info_left \
/robot1/depth_left /robot1/odom /robot1/rgb_left \
/robot1/imu /robot1/scan /robot2/camera_info_left \
/robot2/depth_left /robot2/odom /robot2/rgb_left \
/robot2/imu /robot2/scan /tf -o rosbag_name.bag
```

### Replica Apartment2 Environment
We develop a [ROS wrapper](https://github.com/efc-robot/replica-ros-wrapper) to extract RGB-D sequences and
ground-truth poses from the [Replica Dataset](https://github.com/facebookresearch/Replica-Dataset), transforming them into ROS bag files.

For the replica apartment2 environment, you can directly download the [single-robot rosbag](https://cloud.tsinghua.edu.cn/f/e0ac84f0059142a48cd6/) and the [two-robot rosbag](https://cloud.tsinghua.edu.cn/f/960960e5dafc45fba511/?dl=1).


### Real-world Environment
We integrate iPhones (iPhone 12 Pro or later) as sensors in our framework in two ways:

* Data Collection & Conversion: Captured data is processed and converted into rosbag files, as demonstrated in /r3d_to_ROS/r3d_to_rosbag.py.
* Real-time Streaming: RGB-D and pose information are continuously transformed into ROS messages and published to the corresponding ROS topics. This enables real-time COGraph construction, implemented in /r3d_to_ROS/record3d_ros.zip.

<img src="https://github.com/efc-robot/MR-COGraphs/blob/main/pictures/real_world1.png" alt="描述文本" width="300">

Below are the rosbag files collected from our real-world environment, a 9m × 9m space with three rooms:
* Single-robot: [download](https://cloud.tsinghua.edu.cn/f/aaa58ad3a9dd4257933c/)
* Two-robots: [robot1](https://cloud.tsinghua.edu.cn/f/ac2816e245c74ed0b487/ ) and [robot2](https://cloud.tsinghua.edu.cn/f/d3c97b8064a948a5bd52/).


## Appendix
### GPU usage information
<img src="https://github.com/efc-robot/MR-COGraphs/blob/main/pictures/gpu_usage.jpeg" alt="描述文本" width="500">
The GPU utilization during the COGraph generation process is shown above. 
For detailed metrics, please refer to the log file gpu_usage_log.txt.

### More Demonstrations
<img src="https://github.com/efc-robot/MR-COGraphs/blob/main/pictures/real_world2.png" alt="描述文本" width="500">
We have also conducted tests of our system in a more expansive real-world setting, featuring a corridor and three rooms. 
The illustration below depicts the nodes created by robot1 in the COGraph, along with the merged nodes contributed by robot1 and robot2.
<img src="https://github.com/efc-robot/MR-COGraphs/blob/main/pictures/visualization.png" alt="描述文本" width="500">

### How to train the encoder and decoder

1.Download the dataset from the following URL: https://www.kaggle.com/c/imagenet-object-localization-challenge/data.

2.Input the file LOC_synset_mapping.txt into a large language model (GPT/Kimi), with the prompt: "Based on the information in the dataset, each line begins with a serial number, followed by the word it represents. Select the serial numbers and words that will definitely appear in a room, and output the results in the format of 'serial number + word (reason)'."
The output file obtained is imagenet_classes_in_house_last.txt.

3.Utilizing the list of household objects from imagenet_classes_in_house_last.txt, in conjunction with the annotation files from ILSVRC/Annotations, obtain the cropped images.

4.Input the cropped images into CLIP to acquire features, which will subsequently be used for training the encoder and decoder.

### How to generate content for inquiry purposes
1.Ranking Node Labels by Frequency in COGraph: Sort the node labels in the COGraph dataset based on their occurrence frequency and select the top 10 labels, which will be referred to as "Appeared" in the Query Type.

2.Synonym Generation Using Large Language Models: Input the 10 labels identified in step 1 into a large language model (GPT/Kimi) with the prompt "Find synonyms for these words" to generate a list of synonyms for each label, denoted as "Similar" in the Query Type.

3.Descriptive Phrase Generation Using Large Language Models: Input the 10 labels from step 1 into a large language model (GPT/Kimi) with the prompt "Provide brief descriptions in English for these terms" to obtain a set of brief descriptions for each label, labeled as "Descriptive" in the Query Type.
