#include <ros/ros.h>
#include <ros/assert.h>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <vector>
#include <string>
struct Waypoint {
double x, y, z;};

class Deal_test_cmd
{
    public:
    Deal_test_cmd(const std::vector<Waypoint>& waypoints_001, const std::vector<Waypoint>& waypoints_002, 
                    const std::vector<Waypoint>& waypoints_003, const std::vector<Waypoint>& waypoints_004,
                    const std::vector<Waypoint>& waypoints_005, double threshold) {
        // 在这里初始化成员变量
        this->waypoints_005 = waypoints_005;
        this->threshold = threshold;
    }    

    ros::Publisher cmd_pub_005;

    ros::Subscriber local_sub_005;
    
    std::vector<Waypoint> waypoints_005;

    double threshold;

    ros::Timer cmd_loop;
    ros::Timer land_loop;

    geometry_msgs::PoseStamped cmd_pose_005;

    geometry_msgs::PoseStamped local_pose_005;

    size_t current_waypoint_index_005 = 0;

    int traj_end_005=0;

    void publishNextWaypoint_005();
    
    double calculateDistance(const geometry_msgs::PoseStamped& pose, const Waypoint& waypoint);

    void local_cb_005(geometry_msgs::PoseStampedConstPtr pMsg){local_pose_005 = *pMsg;}
    
    void cmd_loop_cb(const ros::TimerEvent&);

};


void Deal_test_cmd::publishNextWaypoint_005(){
    if(current_waypoint_index_005 < waypoints_005.size()-1){
        if(calculateDistance(local_pose_005,waypoints_005[current_waypoint_index_005])< threshold){
            cmd_pose_005.pose.position.x = waypoints_005[current_waypoint_index_005+1].x;
            cmd_pose_005.pose.position.y = waypoints_005[current_waypoint_index_005+1].y;
            cmd_pose_005.pose.position.z = waypoints_005[current_waypoint_index_005+1].z;
            cmd_pose_005.header.stamp = ros::Time::now();
            cmd_pub_005.publish(cmd_pose_005);
        current_waypoint_index_005++;        
        }
        else{
            cmd_pose_005.pose.position.x = waypoints_005[current_waypoint_index_005].x;
            cmd_pose_005.pose.position.y = waypoints_005[current_waypoint_index_005].y;
            cmd_pose_005.pose.position.z = waypoints_005[current_waypoint_index_005].z;
            cmd_pose_005.header.stamp = ros::Time::now();
            cmd_pub_005.publish(cmd_pose_005);        
        }
    }
    else if(current_waypoint_index_005 == waypoints_005.size()-1){
        if(calculateDistance(local_pose_005,waypoints_005[current_waypoint_index_005])< threshold && !traj_end_005){
            cmd_pose_005.pose.position.x = waypoints_005[current_waypoint_index_005].x;
            cmd_pose_005.pose.position.y = waypoints_005[current_waypoint_index_005].y;
            cmd_pose_005.pose.position.z = waypoints_005[current_waypoint_index_005].z;
            cmd_pose_005.header.stamp = ros::Time::now();
            cmd_pub_005.publish(cmd_pose_005);
            traj_end_005=1;  
            
        }
        else if(traj_end_005){
            // 所有航点已经发送完毕，可以进行降落等操作
            ROS_INFO("All waypoints sent. Initiating landing procedure.");
            cmd_pose_005.pose.position.x = waypoints_005[current_waypoint_index_005].x;
            cmd_pose_005.pose.position.y = waypoints_005[current_waypoint_index_005].y;
            cmd_pose_005.pose.position.z = -0.2;
            cmd_pub_005.publish(cmd_pose_005); 
        }
        else{
            cmd_pose_005.pose.position.x = waypoints_005[current_waypoint_index_005].x;
            cmd_pose_005.pose.position.y = waypoints_005[current_waypoint_index_005].y;
            cmd_pose_005.pose.position.z = waypoints_005[current_waypoint_index_005].z;
            cmd_pose_005.header.stamp = ros::Time::now();
            cmd_pub_005.publish(cmd_pose_005);       
        }
       
    }
}

void Deal_test_cmd::cmd_loop_cb(const ros::TimerEvent&)
{ 
    publishNextWaypoint_005();
    
    std::cout << "cmd_pose5_x=" << cmd_pose_005.pose.position.x << std::endl;
    std::cout << "cmd_pose5_z=" << cmd_pose_005.pose.position.z << std::endl;
    std::cout << "local_pose_005=" << local_pose_005.pose.position.x << std::endl;
    std::cout << "index_005=" << current_waypoint_index_005 << std::endl;
    std::cout << "distance=" << calculateDistance(local_pose_005,waypoints_005[current_waypoint_index_003]) << std::endl;
    std::cout << "threshold=" << threshold << std::endl;
    std::cout << "traj_end_005=" << traj_end_005 << std::endl;
    std::cout << "waypoints_005.size()=" << waypoints_005.size() << std::endl;
    
}


double Deal_test_cmd::calculateDistance(const geometry_msgs::PoseStamped& pose, const Waypoint& waypoint) {
    double dx = pose.pose.position.x - waypoint.x;
    double dy = pose.pose.position.y - waypoint.y;
    double dz = pose.pose.position.z - waypoint.z;
    return sqrt(dx * dx + dy * dy + dz * dz);
}


std::vector<Waypoint> getWaypoints(const std::string& param_name, ros::NodeHandle& nh) {
    std::vector<Waypoint> waypoints;
    XmlRpc::XmlRpcValue waypoints_list;

    if (nh.getParam(param_name, waypoints_list)) {
        // 假设每个列表是一个数组，每个元素是一个包含 x, y, z 的结构体
        for (int i = 0; i < waypoints_list.size(); i++) {
            Waypoint waypoint;
            waypoint.x = static_cast<double>(waypoints_list[i]["x"]);
            waypoint.y = static_cast<double>(waypoints_list[i]["y"]);
            waypoint.z = static_cast<double>(waypoints_list[i]["z"]);
            waypoints.emplace_back(waypoint);
        }
    } else {
        ROS_ERROR("Unable to load waypoints for %s", param_name.c_str());
    }
    return waypoints;
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "cmd_pub_airs");
    ros::NodeHandle nh("~");
    auto waypoints_005 = getWaypoints("drone_005_waypoints", nh);
    double threshold;
    nh.getParam("threshold", threshold);

    // for (const auto& waypoint : waypoints_001) {
    //     ROS_INFO("Waypoint_001: x = %f, y = %f, z = %f", waypoint.x, waypoint.y, waypoint.z);
    // }

    Deal_test_cmd test_cmd(waypoints_005,threshold);
    test_cmd.cmd_loop = nh.createTimer(ros::Duration(0.1), &Deal_test_cmd::cmd_loop_cb,&test_cmd);


    test_cmd.cmd_pub_005 = nh.advertise<geometry_msgs::PoseStamped>("/drone5/position_cmd",100);
    test_cmd.local_sub_005 =  nh.subscribe<geometry_msgs::PoseStamped>("/drone5/mavros/local_position/pose",
                                         10,
                                         boost::bind(&Deal_test_cmd::local_cb_005, &test_cmd, _1),
                                         ros::VoidConstPtr(),
                                         ros::TransportHints().unreliable().maxDatagramSize(1000));

    ros::spin();
    return 0;
}


