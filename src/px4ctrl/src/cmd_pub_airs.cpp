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
        this->waypoints_001 = waypoints_001;
        this->waypoints_002 = waypoints_002;
        this->waypoints_003 = waypoints_003;
        this->waypoints_004 = waypoints_004;
        this->waypoints_005 = waypoints_005;
        this->threshold = threshold;
    }    
    ros::Publisher cmd_pub_001;
    ros::Publisher cmd_pub_002;
    ros::Publisher cmd_pub_003;
    ros::Publisher cmd_pub_004;
    ros::Publisher cmd_pub_005;

    ros::Subscriber local_sub_001;
    ros::Subscriber local_sub_002;
    ros::Subscriber local_sub_003;
    ros::Subscriber local_sub_004;
    ros::Subscriber local_sub_005;
    
    std::vector<Waypoint> waypoints_001;
    std::vector<Waypoint> waypoints_002;
    std::vector<Waypoint> waypoints_003;
    std::vector<Waypoint> waypoints_004;
    std::vector<Waypoint> waypoints_005;

    double threshold;

    ros::Timer cmd_loop;
    ros::Timer land_loop;
    
    geometry_msgs::PoseStamped cmd_pose_001;
    geometry_msgs::PoseStamped cmd_pose_002;
    geometry_msgs::PoseStamped cmd_pose_003;
    geometry_msgs::PoseStamped cmd_pose_004;
    geometry_msgs::PoseStamped cmd_pose_005;
    
    geometry_msgs::PoseStamped local_pose_001;
    geometry_msgs::PoseStamped local_pose_002;
    geometry_msgs::PoseStamped local_pose_003;
    geometry_msgs::PoseStamped local_pose_004;
    geometry_msgs::PoseStamped local_pose_005;

    size_t current_waypoint_index_001 = 0;
    size_t current_waypoint_index_002 = 0;
    size_t current_waypoint_index_003 = 0;
    size_t current_waypoint_index_004 = 0;
    size_t current_waypoint_index_005 = 0;

    int traj_end_001=0;
    int traj_end_002=0;
    int traj_end_003=0;
    int traj_end_004=0;
    int traj_end_005=0;

    void publishNextWaypoint_001();
    void publishNextWaypoint_002();
    void publishNextWaypoint_003();
    void publishNextWaypoint_004();
    void publishNextWaypoint_005();
    
    double calculateDistance(const geometry_msgs::PoseStamped& pose, const Waypoint& waypoint);

    // void local_cb_001(geometry_msgs::PoseStampedConstPtr pMsg)
    // {
    //     local_pose_001.pose.position.x=pMsg->pose.position.x;
    //     local_pose_001.pose.position.y=pMsg->pose.position.y;
    //     local_pose_001.pose.position.z=pMsg->pose.position.z;
    // }
    void local_cb_001(geometry_msgs::PoseStampedConstPtr pMsg){local_pose_001 = *pMsg;}
    void local_cb_002(geometry_msgs::PoseStampedConstPtr pMsg){local_pose_002 = *pMsg;}
    // void local_cb_003(geometry_msgs::PoseStampedConstPtr pMsg){local_pose_003 = *pMsg;}
    void local_cb_003(geometry_msgs::PoseStampedConstPtr pMsg)
    {
        local_pose_003.pose.position.x=pMsg->pose.position.x;
        local_pose_003.pose.position.y=pMsg->pose.position.y;
        local_pose_003.pose.position.z=pMsg->pose.position.z;
    }    
    void local_cb_004(geometry_msgs::PoseStampedConstPtr pMsg){local_pose_004 = *pMsg;}
    void local_cb_005(geometry_msgs::PoseStampedConstPtr pMsg){local_pose_005 = *pMsg;}
    
    void cmd_loop_cb(const ros::TimerEvent&);

};

void Deal_test_cmd::publishNextWaypoint_001(){
    if(current_waypoint_index_001 < waypoints_001.size()-1){
        if(calculateDistance(local_pose_001,waypoints_001[current_waypoint_index_001])< threshold){
            cmd_pose_001.pose.position.x = waypoints_001[current_waypoint_index_001+1].x;
            cmd_pose_001.pose.position.y = waypoints_001[current_waypoint_index_001+1].y;
            cmd_pose_001.pose.position.z = waypoints_001[current_waypoint_index_001+1].z;
            cmd_pose_001.header.stamp = ros::Time::now();
            cmd_pub_001.publish(cmd_pose_001);
        current_waypoint_index_001++;        
        }
        else{
            cmd_pose_001.pose.position.x = waypoints_001[current_waypoint_index_001].x;
            cmd_pose_001.pose.position.y = waypoints_001[current_waypoint_index_001].y;
            cmd_pose_001.pose.position.z = waypoints_001[current_waypoint_index_001].z;
            cmd_pose_001.header.stamp = ros::Time::now();
            cmd_pub_001.publish(cmd_pose_001);        
        }
    }
    else if(current_waypoint_index_001 == waypoints_001.size()-1){
        if(calculateDistance(local_pose_001,waypoints_001[current_waypoint_index_001])< threshold && !traj_end_001){
            cmd_pose_001.pose.position.x = waypoints_001[current_waypoint_index_001].x;
            cmd_pose_001.pose.position.y = waypoints_001[current_waypoint_index_001].y;
            cmd_pose_001.pose.position.z = waypoints_001[current_waypoint_index_001].z;
            cmd_pose_001.header.stamp = ros::Time::now();
            cmd_pub_001.publish(cmd_pose_001);
            traj_end_001=1;  
            
        }
        else if(traj_end_001){
            // 所有航点已经发送完毕，可以进行降落等操作
            ROS_INFO("All waypoints sent. Initiating landing procedure.");
            cmd_pose_001.pose.position.x = waypoints_001[current_waypoint_index_001].x;
            cmd_pose_001.pose.position.y = waypoints_001[current_waypoint_index_001].y;
            cmd_pose_001.pose.position.z = -0.2;
            cmd_pub_001.publish(cmd_pose_001); 
        }
        else{
            cmd_pose_001.pose.position.x = waypoints_001[current_waypoint_index_001].x;
            cmd_pose_001.pose.position.y = waypoints_001[current_waypoint_index_001].y;
            cmd_pose_001.pose.position.z = waypoints_001[current_waypoint_index_001].z;
            cmd_pose_001.header.stamp = ros::Time::now();
            cmd_pub_001.publish(cmd_pose_001);       
        }
       
    }
}

void Deal_test_cmd::publishNextWaypoint_002(){
    if(current_waypoint_index_002 < waypoints_002.size()-1){
        if(calculateDistance(local_pose_002,waypoints_002[current_waypoint_index_002])< threshold){
            cmd_pose_002.pose.position.x = waypoints_002[current_waypoint_index_002+1].x;
            cmd_pose_002.pose.position.y = waypoints_002[current_waypoint_index_002+1].y;
            cmd_pose_002.pose.position.z = waypoints_002[current_waypoint_index_002+1].z;
            cmd_pose_002.header.stamp = ros::Time::now();
            cmd_pub_002.publish(cmd_pose_002);
        current_waypoint_index_002++;        
        }
        else{
            cmd_pose_002.pose.position.x = waypoints_002[current_waypoint_index_002].x;
            cmd_pose_002.pose.position.y = waypoints_002[current_waypoint_index_002].y;
            cmd_pose_002.pose.position.z = waypoints_002[current_waypoint_index_002].z;
            cmd_pose_002.header.stamp = ros::Time::now();
            cmd_pub_002.publish(cmd_pose_002);        
        }
    }
    else if(current_waypoint_index_002 == waypoints_002.size()-1){
        if(calculateDistance(local_pose_002,waypoints_002[current_waypoint_index_002])< threshold && !traj_end_002){
            cmd_pose_002.pose.position.x = waypoints_002[current_waypoint_index_002].x;
            cmd_pose_002.pose.position.y = waypoints_002[current_waypoint_index_002].y;
            cmd_pose_002.pose.position.z = waypoints_002[current_waypoint_index_002].z;
            cmd_pose_002.header.stamp = ros::Time::now();
            cmd_pub_002.publish(cmd_pose_002);
            traj_end_002=1;  
            
        }
        else if(traj_end_002){
            // 所有航点已经发送完毕，可以进行降落等操作
            ROS_INFO("All waypoints sent. Initiating landing procedure.");
            cmd_pose_002.pose.position.x = waypoints_002[current_waypoint_index_002].x;
            cmd_pose_002.pose.position.y = waypoints_002[current_waypoint_index_002].y;
            cmd_pose_002.pose.position.z = -0.2;
            cmd_pub_002.publish(cmd_pose_002); 
        }
        else{
            cmd_pose_002.pose.position.x = waypoints_002[current_waypoint_index_002].x;
            cmd_pose_002.pose.position.y = waypoints_002[current_waypoint_index_002].y;
            cmd_pose_002.pose.position.z = waypoints_002[current_waypoint_index_002].z;
            cmd_pose_002.header.stamp = ros::Time::now();
            cmd_pub_002.publish(cmd_pose_002);       
        }
       
    }
}

void Deal_test_cmd::publishNextWaypoint_003(){
    if(current_waypoint_index_003 < waypoints_003.size()-1){
        if(calculateDistance(local_pose_003,waypoints_003[current_waypoint_index_003])< threshold){
            cmd_pose_003.pose.position.x = waypoints_003[current_waypoint_index_003+1].x;
            cmd_pose_003.pose.position.y = waypoints_003[current_waypoint_index_003+1].y;
            cmd_pose_003.pose.position.z = waypoints_003[current_waypoint_index_003+1].z;
            cmd_pose_003.header.stamp = ros::Time::now();
            cmd_pub_003.publish(cmd_pose_003);
        current_waypoint_index_003++;        
        }
        else{
            cmd_pose_003.pose.position.x = waypoints_003[current_waypoint_index_003].x;
            cmd_pose_003.pose.position.y = waypoints_003[current_waypoint_index_003].y;
            cmd_pose_003.pose.position.z = waypoints_003[current_waypoint_index_003].z;
            cmd_pose_003.header.stamp = ros::Time::now();
            cmd_pub_003.publish(cmd_pose_003);        
        }
    }
    else if(current_waypoint_index_003 == waypoints_003.size()-1){
        if(calculateDistance(local_pose_003,waypoints_003[current_waypoint_index_003])< threshold && !traj_end_003){
            cmd_pose_003.pose.position.x = waypoints_003[current_waypoint_index_003].x;
            cmd_pose_003.pose.position.y = waypoints_003[current_waypoint_index_003].y;
            cmd_pose_003.pose.position.z = waypoints_003[current_waypoint_index_003].z;
            cmd_pose_003.header.stamp = ros::Time::now();
            cmd_pub_003.publish(cmd_pose_003);
            traj_end_003=1;  
            
        }
        else if(traj_end_003){
            // 所有航点已经发送完毕，可以进行降落等操作
            ROS_INFO("All waypoints sent. Initiating landing procedure.");
            cmd_pose_003.pose.position.x = waypoints_003[current_waypoint_index_003].x;
            cmd_pose_003.pose.position.y = waypoints_003[current_waypoint_index_003].y;
            cmd_pose_003.pose.position.z = -0.2;
            cmd_pub_003.publish(cmd_pose_003); 
        }
        else{
            cmd_pose_003.pose.position.x = waypoints_003[current_waypoint_index_003].x;
            cmd_pose_003.pose.position.y = waypoints_003[current_waypoint_index_003].y;
            cmd_pose_003.pose.position.z = waypoints_003[current_waypoint_index_003].z;
            cmd_pose_003.header.stamp = ros::Time::now();
            cmd_pub_003.publish(cmd_pose_003);       
        }
       
    }
}

void Deal_test_cmd::publishNextWaypoint_004(){
    if(current_waypoint_index_004 < waypoints_004.size()-1){
        if(calculateDistance(local_pose_004,waypoints_004[current_waypoint_index_004])< threshold){
            cmd_pose_004.pose.position.x = waypoints_004[current_waypoint_index_004+1].x;
            cmd_pose_004.pose.position.y = waypoints_004[current_waypoint_index_004+1].y;
            cmd_pose_004.pose.position.z = waypoints_004[current_waypoint_index_004+1].z;
            cmd_pose_004.header.stamp = ros::Time::now();
            cmd_pub_004.publish(cmd_pose_004);
        current_waypoint_index_004++;        
        }
        else{
            cmd_pose_004.pose.position.x = waypoints_004[current_waypoint_index_004].x;
            cmd_pose_004.pose.position.y = waypoints_004[current_waypoint_index_004].y;
            cmd_pose_004.pose.position.z = waypoints_004[current_waypoint_index_004].z;
            cmd_pose_004.header.stamp = ros::Time::now();
            cmd_pub_004.publish(cmd_pose_004);        
        }
    }
    else if(current_waypoint_index_004 == waypoints_004.size()-1){
        if(calculateDistance(local_pose_004,waypoints_004[current_waypoint_index_004])< threshold && !traj_end_004){
            cmd_pose_004.pose.position.x = waypoints_004[current_waypoint_index_004].x;
            cmd_pose_004.pose.position.y = waypoints_004[current_waypoint_index_004].y;
            cmd_pose_004.pose.position.z = waypoints_004[current_waypoint_index_004].z;
            cmd_pose_004.header.stamp = ros::Time::now();
            cmd_pub_004.publish(cmd_pose_004);
            traj_end_004=1;  
            
        }
        else if(traj_end_004){
            // 所有航点已经发送完毕，可以进行降落等操作
            ROS_INFO("All waypoints sent. Initiating landing procedure.");
            cmd_pose_004.pose.position.x = waypoints_004[current_waypoint_index_004].x;
            cmd_pose_004.pose.position.y = waypoints_004[current_waypoint_index_004].y;
            cmd_pose_004.pose.position.z = -0.2;
            cmd_pub_004.publish(cmd_pose_004); 
        }
        else{
            cmd_pose_004.pose.position.x = waypoints_004[current_waypoint_index_004].x;
            cmd_pose_004.pose.position.y = waypoints_004[current_waypoint_index_004].y;
            cmd_pose_004.pose.position.z = waypoints_004[current_waypoint_index_004].z;
            cmd_pose_004.header.stamp = ros::Time::now();
            cmd_pub_004.publish(cmd_pose_004);       
        }
       
    }
}

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
    publishNextWaypoint_001();
    publishNextWaypoint_002();
    publishNextWaypoint_003();
    publishNextWaypoint_004();
    publishNextWaypoint_005();
    
    std::cout << "cmd_pose_x=" << cmd_pose_003.pose.position.x << std::endl;
    std::cout << "cmd_pose_z=" << cmd_pose_003.pose.position.z << std::endl;
    std::cout << "local_pose_003=" << local_pose_003.pose.position.x << std::endl;
    std::cout << "index_003=" << current_waypoint_index_003 << std::endl;
    std::cout << "distance=" << calculateDistance(local_pose_003,waypoints_003[current_waypoint_index_003]) << std::endl;
    std::cout << "threshold=" << threshold << std::endl;
    std::cout << "traj_end_003=" << traj_end_003 << std::endl;
    std::cout << "waypoints_003.size()=" << waypoints_003.size() << std::endl;
    
    // std::cout << "cmd_pose_002=" << cmd_pose_002.pose.position.x << std::endl;
    // std::cout << "local_pose_002=" << local_pose_002.pose.position.x << std::endl;

    // std::cout << "cmd_pose_003=" << cmd_pose_003.pose.position.x << std::endl;
    // std::cout << "local_pose_003=" << local_pose_003.pose.position.x << std::endl;

    // std::cout << "cmd_pose_004=" << cmd_pose_004.pose.position.x << std::endl;
    // std::cout << "local_pose_004=" << local_pose_004.pose.position.x << std::endl;

    // std::cout << "cmd_pose_005=" << cmd_pose_005.pose.position.x << std::endl;
    // std::cout << "local_pose_005=" << local_pose_005.pose.position.x << std::endl;

    // std::cout << "flag=" << takeoff_flag << std::endl;
    // std::cout << "flagcount=" << flagcount << std::endl;
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

    auto waypoints_001 = getWaypoints("drone_001_waypoints", nh);
    auto waypoints_002 = getWaypoints("drone_002_waypoints", nh);
    auto waypoints_003 = getWaypoints("drone_003_waypoints", nh);
    auto waypoints_004 = getWaypoints("drone_004_waypoints", nh);
    auto waypoints_005 = getWaypoints("drone_005_waypoints", nh);
    double threshold;
    nh.getParam("threshold", threshold);

    // for (const auto& waypoint : waypoints_001) {
    //     ROS_INFO("Waypoint_001: x = %f, y = %f, z = %f", waypoint.x, waypoint.y, waypoint.z);
    // }

    Deal_test_cmd test_cmd(waypoints_001,waypoints_002,waypoints_003,waypoints_004,waypoints_005,threshold);
    test_cmd.cmd_loop = nh.createTimer(ros::Duration(0.1), &Deal_test_cmd::cmd_loop_cb,&test_cmd);

    test_cmd.cmd_pub_001 = nh.advertise<geometry_msgs::PoseStamped>("/drone4/position_cmd",100);
    test_cmd.local_sub_001 =  nh.subscribe<geometry_msgs::PoseStamped>("/drone4/mavros/local_position/pose",
                                         10,
                                         boost::bind(&Deal_test_cmd::local_cb_001, &test_cmd, _1),
                                         ros::VoidConstPtr(),
                                         ros::TransportHints().unreliable().maxDatagramSize(1000));
    
    test_cmd.cmd_pub_002 = nh.advertise<geometry_msgs::PoseStamped>("/drone5/position_cmd",100);
    test_cmd.local_sub_002 =  nh.subscribe<geometry_msgs::PoseStamped>("/drone5/mavros/local_position/pose",
                                         10,
                                         boost::bind(&Deal_test_cmd::local_cb_002, &test_cmd, _1),
                                         ros::VoidConstPtr(),
                                         ros::TransportHints().unreliable().maxDatagramSize(1000));
    
    test_cmd.cmd_pub_003 = nh.advertise<geometry_msgs::PoseStamped>("/drone6/position_cmd",100);
    test_cmd.local_sub_003 =  nh.subscribe<geometry_msgs::PoseStamped>("/drone6/mavros/local_position/pose",
                                         10,
                                         boost::bind(&Deal_test_cmd::local_cb_003, &test_cmd, _1),
                                         ros::VoidConstPtr(),
                                         ros::TransportHints().unreliable().maxDatagramSize(1000));
    
    test_cmd.cmd_pub_004 = nh.advertise<geometry_msgs::PoseStamped>("/drone004/position_cmd",100);
    test_cmd.local_sub_004 =  nh.subscribe<geometry_msgs::PoseStamped>("/drone004/mavros/local_position/pose",
                                         10,
                                         boost::bind(&Deal_test_cmd::local_cb_004, &test_cmd, _1),
                                         ros::VoidConstPtr(),
                                         ros::TransportHints().unreliable().maxDatagramSize(1000));

    test_cmd.cmd_pub_005 = nh.advertise<geometry_msgs::PoseStamped>("/drone005/position_cmd",100);
    test_cmd.local_sub_005 =  nh.subscribe<geometry_msgs::PoseStamped>("/drone005/mavros/local_position/pose",
                                         10,
                                         boost::bind(&Deal_test_cmd::local_cb_005, &test_cmd, _1),
                                         ros::VoidConstPtr(),
                                         ros::TransportHints().unreliable().maxDatagramSize(1000));

    ros::spin();
    return 0;
}


