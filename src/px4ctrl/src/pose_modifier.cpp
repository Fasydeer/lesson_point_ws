/*2024-1-22
作用：改变动捕数据y和z方向，本package中的程序是原本在以X为骨轴的动捕场地下进行的，
AIRS的动捕场地是以Y为骨轴的，故本node用于转换坐标的Y和Z方向
*/
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

void Mocap_Callback(const geometry_msgs::PoseStamped::ConstPtr& msg, const ros::Publisher& pub) {
    geometry_msgs::PoseStamped new_msg = *msg;

    // 交换 Y 和 Z 轴
    std::swap(new_msg.pose.position.y, new_msg.pose.position.z);
    new_msg.pose.position.y = -1*new_msg.pose.position.y;

    new_msg.header.stamp = ros::Time::now();

    // 发布修改后的消息
    pub.publish(new_msg);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "pose_modifier");
    ros::NodeHandle nh;

    // 为每个机器人创建发布者
    ros::Publisher pub[5];
    char topic_name[50];

    for (int i = 0; i < 5; ++i) {
        sprintf(topic_name, "/drone%03d/mavros/vision_pose/pose", i+1);
        pub[i] = nh.advertise<geometry_msgs::PoseStamped>(topic_name, 10);
    }

    // 为每个机器人创建订阅者
    ros::Subscriber sub[5];
    for (int i = 0; i < 5; ++i) {
        sprintf(topic_name, "/vrpn_client_node/UAV_%03d/pose", i+1);
        sub[i] = nh.subscribe<geometry_msgs::PoseStamped>(topic_name, 10, boost::bind(Mocap_Callback, _1, pub[i]));
    }

    // 使用异步自旋器允许多线程回调处理
    ros::AsyncSpinner spinner(4); // 使用4个线程
    spinner.start();
    ros::waitForShutdown();

    return 0;
}

//这是不使用多线程的
/*
// 通用回调函数，用于处理接收到的消息并发布到新的主题
void Mocap_Callback(const geometry_msgs::PoseStamped::ConstPtr& msg, const ros::Publisher& pub) {
    geometry_msgs::PoseStamped new_msg = *msg;

    // 交换 Y 和 Z 轴
    std::swap(new_msg.pose.position.y, new_msg.pose.position.z);

    // 发布修改后的消息
    pub.publish(new_msg);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "pose_modifier");
    ros::NodeHandle nh;

    // 为每个机器人创建一个订阅者和发布者
    ros::Publisher pub1 = nh.advertise<geometry_msgs::PoseStamped>("/drone001/mavros/vision_pose/pose", 10);
    ros::Publisher pub2 = nh.advertise<geometry_msgs::PoseStamped>("/drone002/mavros/vision_pose/pose", 10);
    ros::Publisher pub3 = nh.advertise<geometry_msgs::PoseStamped>("/drone003/mavros/vision_pose/pose", 10);
    ros::Publisher pub4 = nh.advertise<geometry_msgs::PoseStamped>("/drone004/mavros/vision_pose/pose", 10);
    ros::Publisher pub5 = nh.advertise<geometry_msgs::PoseStamped>("/drone005/mavros/vision_pose/pose", 10);

    ros::Subscriber sub1 = nh.subscribe<geometry_msgs::PoseStamped>("UAV_001_rigid", 10, boost::bind(Mocap_Callback, _1, pub1),
                                                                    ros::TransportHints().unreliable().maxDatagramSize(1000));
    ros::Subscriber sub2 = nh.subscribe<geometry_msgs::PoseStamped>("UAV_002_rigid", 10, boost::bind(Mocap_Callback, _1, pub2),
                                                                    ros::TransportHints().unreliable().maxDatagramSize(1000));
    ros::Subscriber sub3 = nh.subscribe<geometry_msgs::PoseStamped>("UAV_003_rigid", 10, boost::bind(Mocap_Callback, _1, pub3),
                                                                    ros::TransportHints().unreliable().maxDatagramSize(1000));
    ros::Subscriber sub4 = nh.subscribe<geometry_msgs::PoseStamped>("UAV_004_rigid", 10, boost::bind(Mocap_Callback, _1, pub4),
                                                                    ros::TransportHints().unreliable().maxDatagramSize(1000));
    ros::Subscriber sub5 = nh.subscribe<geometry_msgs::PoseStamped>("UAV_005_rigid", 10, boost::bind(Mocap_Callback, _1, pub5),
                                                                    ros::TransportHints().unreliable().maxDatagramSize(1000));

    ros::spin();
    return 0;
}
*/