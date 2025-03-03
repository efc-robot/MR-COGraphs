import rosbag
import os
import rospy
from argparse import ArgumentParser
from tf2_msgs.msg import TFMessage


# 把rosbag文件中的/tf断掉部分关联
def delete_rosbag_tf(input_bag, output_bag):
    # 打开输入bag文件
    input_bag = rosbag.Bag(input_bag, 'r')

    # 获取所有topic
    topics = input_bag.get_type_and_topic_info()[1].keys()

    # 创建输出bag文件
    output_bag = rosbag.Bag(output_bag, 'w')
    start_time = input_bag.get_start_time()
    end_time = input_bag.get_end_time()
    
    # 遍历输入bag文件中的消息
    for topic, msg, t in input_bag.read_messages():
        print(t)
        if topic == '/tf':
            # 新的 tf 变换列表
            new_transforms = []
            for transform in msg.transforms:
                # 如果 frame_id 不是 'world'，则保留
                # if transform.header.frame_id == "world" or transform.header.frame_id == "robot1/base_footprint" or transform.header.frame_id == "robot2/base_footprint" or transform.header.frame_id == "robot2/odom" or transform.header.frame_id == "robot1/odom"  :
                #     pass
                # elif transform.header.frame_id == "robot1/base_link":
                #     transform.header.frame_id = "robot1/base_link1"
                #     new_transforms.append(transform)
                # elif transform.header.frame_id == "robot2/base_link":
                #     transform.header.frame_id = "robot2/base_link2"
                #     new_transforms.append(transform)
                # elif transform.child_frame_id == "robot1/base_link":
                #     transform.child_frame_id = "robot1/base_link1"
                #     new_transforms.append(transform)
                # elif transform.child_frame_id == "robot2/base_link":
                #     transform.child_frame_id = "robot2/base_link2"
                #     new_transforms.append(transform)
                if transform.header.frame_id == "world" or transform.header.frame_id == "robot1/base_footprint" or transform.header.frame_id == "robot2/base_footprint" or transform.header.frame_id == "robot3/base_footprint":
                # if transform.header.frame_id == "world" or transform.header.frame_id == "robot1/base_footprint":
                    pass
                else:
                    new_transforms.append(transform)

            # 如果新的 transform 列表不为空，则写入新的 bag 文件
            if new_transforms:
                new_msg = TFMessage(transforms=new_transforms)
                output_bag.write('/tf', new_msg, t)
        else:
            # 非 /tf 消息直接写入新的 bag 文件
            output_bag.write(topic, msg, t)

    # 关闭输入和输出bag文件
    input_bag.close()
    output_bag.close()

# 把rosbag文件中的消息时间倒序
def reverse_rosbag(input_bag, output_bag):
    # 打开输入bag文件
    input_bag = rosbag.Bag(input_bag, 'r')

    # 获取所有topic
    topics = input_bag.get_type_and_topic_info()[1].keys()

    # 创建输出bag文件
    output_bag = rosbag.Bag(output_bag, 'w')
    start_time = input_bag.get_start_time()
    end_time = input_bag.get_end_time()
    # 遍历输入bag文件中的消息
    for topic, msg, t in input_bag.read_messages(topics=topics):
        
        offset = end_time - t.to_sec()
        t_ = rospy.Time.from_sec(start_time) + rospy.Duration.from_sec(offset)

        if topic == '/tf':
            for transform in msg.transforms:
                transform.header.stamp = t_
        elif topic == '/tf_static':
            for transform in msg.transforms:
                transform.header.stamp = t
            output_bag.write(topic, msg, t)
            continue
        elif topic == '/clock' :
            msg.clock = t_

        elif topic == '/rosout' or topic == '/rosout_agg':
            msg.header.stamp = t_
        else:
            # 更新消息时间
            msg.header.stamp = t_

        # 写入消息到输出bag文件
        output_bag.write(topic, msg, t_)
        print("offset =,", offset)

    # 关闭输入和输出bag文件
    input_bag.close()
    output_bag.close()

# 根据duration--持续时间裁剪rosbag
def split_rosbag(input_bag, output_bag, duration=10, namespace=''):
    # 打开输入bag文件
    input_bag = rosbag.Bag(input_bag, 'r')

    # 获取所有topic
    topics = input_bag.get_type_and_topic_info()[1].keys()

    # 创建输出bag文件
    output_bag = rosbag.Bag(output_bag, 'w')

    start_time = input_bag.get_start_time()

    # 计算裁剪的结束时间
    end_time = input_bag.get_start_time() + duration

    # 遍历输入bag文件中的消息
    for topic, msg, t in input_bag.read_messages(topics=topics):
        # 如果消息时间超过了裁剪的结束时间，则停止写入消息
        if t.to_sec() > end_time:
            break

        # 计算消息时间相对于第一个消息的偏移量
        offset = t.to_sec() - input_bag.get_start_time()

        # t_ = rospy.Time.from_sec(start_time) + rospy.Duration.from_sec(offset)
        print("offset = ", offset)

        if topic == '/tf':
            for transform in msg.transforms:
                transform.child_frame_id = namespace +'/'+ transform.child_frame_id
        elif topic == '/tf_static':
            for transform in msg.transforms:
                transform.child_frame_id = namespace +'/'+ transform.child_frame_id
                transform.header.frame_id = namespace +'/'+ transform.header.frame_id
        
        elif topic == '/clock' or topic == '/rosout' or topic == '/rosout_agg':
            pass
        else:
            # 更新消息时间
            msg.header.frame_id = namespace +'/' + msg.header.frame_id
            topic = namespace + topic

        print(offset)
        # # 对tf单独处理
        # if topic == '/tf' or topic == '/tf_static':
        #     for transform in msg.transforms:
        #         transform.header.stamp = t_
            
        # elif topic == '/clock':
        #     # 更新消息时间
        #     msg.clock = t_
        # else:
        #     # 更新消息时间
        #     msg.header.stamp = t_

        # 写入消息到输出bag文件
        output_bag.write(topic, msg, t)

    # 关闭输入和输出bag文件
    input_bag.close()
    output_bag.close()

# 根据起始时间裁剪rosbag，并增加namespace
def split_rosbag_start_time(input_bag, output_bag, start_duration=250, duration=300, namespace=''):
    # 打开输入bag文件
    input_bag = rosbag.Bag(input_bag, 'r')

    # 获取所有topic
    topics = input_bag.get_type_and_topic_info()[1].keys()

    # 创建输出bag文件
    output_bag = rosbag.Bag(output_bag, 'w')

    init_time = input_bag.get_start_time()

    # 计算裁剪的结束时间
    
    start_time = init_time + start_duration
    end_time = start_time + duration
    print("start_time = ", start_time)
    print("end_time = ", end_time)
    print("start")

    # 遍历输入bag文件中的消息
    for topic, msg, t in input_bag.read_messages(topics=topics):
        # 如果消息时间超过了裁剪的结束时间，则停止写入消息
        print("t = ", t.to_sec())
        if t.to_sec() > start_time and t.to_sec() < end_time:
            
        # 计算消息时间相对于第一个消息的偏移量
            offset = t.to_sec() - start_time

            t_ = rospy.Time.from_sec(init_time) + rospy.Duration.from_sec(offset)
            if topic == '/tf':
                for transform in msg.transforms:
                    transform.header.stamp = t_
                    transform.child_frame_id = namespace +'/'+ transform.child_frame_id
            elif topic == '/tf_static':
                for transform in msg.transforms:
                    transform.header.stamp = t_
                    transform.child_frame_id = namespace +'/'+ transform.child_frame_id
                    transform.header.frame_id = namespace +'/'+ transform.header.frame_id
            elif topic == '/clock' :
                msg.clock = t_
            elif topic == '/rosout' or topic == '/rosout_agg':
                msg.header.stamp = t_
            else:
                # 更新消息时间
                msg.header.stamp = t_
                # 更新消息时间
                msg.header.frame_id = namespace +'/' + msg.header.frame_id
                topic = namespace + topic
            # 写入消息到输出bag文件
            output_bag.write(topic, msg, t_)
            print("offset =,", offset)

    # 关闭输入和输出bag文件
    input_bag.close()
    output_bag.close()

def exchange_namespace(input_bag, output_bag, namespace):
    # 打开输入bag文件
    input_bag = rosbag.Bag(input_bag, 'r')

    # 获取所有topic
    topics = input_bag.get_type_and_topic_info()[1].keys()

    # 创建输出bag文件
    output_bag = rosbag.Bag(output_bag, 'w')

    start_time = input_bag.get_start_time()
    print("start_time = ", start_time)

    # 遍历输入bag文件中的消息
    for topic, msg, t in input_bag.read_messages(topics=topics):

        # print("topic = ", topic)

        if topic == '/tf':
            for transform in msg.transforms:
                transform.child_frame_id = namespace +'/'+ transform.child_frame_id
        elif topic == '/tf_static':
            for transform in msg.transforms:
                transform.child_frame_id = namespace +'/'+ transform.child_frame_id
                transform.header.frame_id = namespace +'/'+ transform.header.frame_id
        
        elif topic == '/clock' or topic == '/rosout' or topic == '/rosout_agg':
            pass
        else:
            # 更新消息时间
            msg.header.frame_id = namespace +'/' + msg.header.frame_id
            topic = namespace + topic

        print(t.to_sec())
        

        # 写入消息到输出bag文件
        output_bag.write(topic, msg, t)

    # 关闭输入和输出bag文件
    input_bag.close()
    output_bag.close()

def cp_add_namespace(input_bag, output_bag, namespace):
    # 打开输入bag文件
    input_bag = rosbag.Bag(input_bag, 'r')

    # 获取所有topic
    topics = input_bag.get_type_and_topic_info()[1].keys()

    # 创建输出bag文件
    output_bag = rosbag.Bag(output_bag, 'w')

    start_time = input_bag.get_start_time()
    print("start_time = ", start_time)

    # 遍历输入bag文件中的消息
    for topic, msg, t in input_bag.read_messages(topics=topics):
        # print("topic = ", topic)

        if topic == '/tf':
            for transform in msg.transforms:
                if transform.header.frame_id == 'world':
                    transform.child_frame_id = namespace +'/'+ transform.child_frame_id
                elif transform.header.frame_id == 'base_link':
                    transform.header.frame_id = namespace +'/'+ transform.header.frame_id
                    transform.child_frame_id = namespace +'/'+ transform.child_frame_id
        elif topic == '/clock' or topic == '/rosout' or topic == '/rosout_agg':
            pass
        else:
            # 更新消息时间
            msg.header.frame_id = namespace +'/' + msg.header.frame_id
            topic = namespace + topic

        print(t.to_sec())
        

        # 写入消息到输出bag文件
        output_bag.write(topic, msg, t)

    # 关闭输入和输出bag文件
    input_bag.close()
    output_bag.close()

if __name__ == '__main__':
    parser = ArgumentParser()
 
    parser.add_argument('--input_bag', default="/root/sharedocker/rosbags/changping/cp_2robots_no_ns_tf_static.bag")
    parser.add_argument('--output_bag', default="/root/sharedocker/rosbags/changping/cp_2robots_robot1.bag")

    args = parser.parse_args()

    input_bag = args.input_bag  # 输入bag文件路径
    output_bag = args.output_bag  # 输出bag文件路径


    # delete_rosbag_tf(input_bag, output_bag)
    # cp_add_namespace(input_bag, output_bag, 'robot1')

    split_rosbag_start_time(input_bag, output_bag, 10, 47, 'robot1')
    # split_rosbag_start_time(input_bag, output_bag, 60, 46, 'robot2')
 