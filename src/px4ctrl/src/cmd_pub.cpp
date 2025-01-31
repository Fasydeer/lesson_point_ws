#include <ros/ros.h>
#include <ros/assert.h>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PositionCommand.h>

class Deal_test_cmd
{
    public:
    Deal_test_cmd(){};
    ros::Publisher cmd_pub4;
    ros::Publisher cmd_pub5;
    ros::Publisher cmd_pub6;
    ros::Publisher cmd_pub7;

    ros::Subscriber local_sub4;
    ros::Subscriber local_sub5;
    ros::Subscriber local_sub6;
    ros::Subscriber local_sub7;
    
    ros::Timer cmd_loop;

    void local_cb4(geometry_msgs::PoseStampedConstPtr pMsg)
    {
        local_pose4.pose.position.x=pMsg->pose.position.x;
        local_pose4.pose.position.y=pMsg->pose.position.y;
        local_pose4.pose.position.z=pMsg->pose.position.z;
    }
    void local_cb5(geometry_msgs::PoseStampedConstPtr pMsg)
    {
        local_pose5.pose.position.x=pMsg->pose.position.x;
        local_pose5.pose.position.y=pMsg->pose.position.y;
        local_pose5.pose.position.z=pMsg->pose.position.z;
    }
    void local_cb6(geometry_msgs::PoseStampedConstPtr pMsg)
    {
        local_pose6.pose.position.x=pMsg->pose.position.x;
        local_pose6.pose.position.y=pMsg->pose.position.y;
        local_pose6.pose.position.z=pMsg->pose.position.z;
    }
    void local_cb7(geometry_msgs::PoseStampedConstPtr pMsg)
    {
        local_pose7.pose.position.x=pMsg->pose.position.x;
        local_pose7.pose.position.y=pMsg->pose.position.y;
        local_pose7.pose.position.z=pMsg->pose.position.z;
    }
    
    geometry_msgs::PoseStamped cmd_pose4;
    geometry_msgs::PoseStamped cmd_pose5;
    geometry_msgs::PoseStamped cmd_pose6;
    geometry_msgs::PoseStamped cmd_pose7;
    
    geometry_msgs::PoseStamped local_pose4;
    geometry_msgs::PoseStamped local_pose5;
    geometry_msgs::PoseStamped local_pose6;
    geometry_msgs::PoseStamped local_pose7;
    
    void cmd_loop_cb(const ros::TimerEvent&);
    bool reach_wpt();
    void pub_next_wpt4();
    void pub_next_wpt5();
    void pub_next_wpt6();
    void pub_next_wpt7();

    int takeoff_flag=0;
    int flagcount = 0;

};




bool Deal_test_cmd::reach_wpt()
{

    double dis4=
    abs(local_pose4.pose.position.x-cmd_pose4.pose.position.x)+
    abs(local_pose4.pose.position.y-cmd_pose4.pose.position.y);
    
    double dis5=
    abs(local_pose5.pose.position.x-cmd_pose5.pose.position.x)+
    abs(local_pose5.pose.position.y-cmd_pose5.pose.position.y);
    
    double dis6=
    abs(local_pose6.pose.position.x-cmd_pose6.pose.position.x)+
    abs(local_pose6.pose.position.y-cmd_pose6.pose.position.y);
    
    double dis7=
    abs(local_pose7.pose.position.x-cmd_pose7.pose.position.x)+
    abs(local_pose7.pose.position.y-cmd_pose7.pose.position.y);

    double reach_flag=0.3;

    if(dis4<=reach_flag) flagcount ++;
    if(dis5<=reach_flag) flagcount ++;
    if(dis6<=reach_flag) flagcount ++;
    if(dis7<=reach_flag) flagcount ++;

    
    if(flagcount >= 3)
    {
        takeoff_flag=1;
        return true;
    }
    else { flagcount = 0; return false; }
}

void Deal_test_cmd::pub_next_wpt4()
{
  
        cmd_pose4.pose.position.x=-3.0;
        cmd_pose4.pose.position.y= 2.0;
        cmd_pose4.pose.position.z= 1.5;
    
    
}

void Deal_test_cmd::pub_next_wpt5()
{

        cmd_pose5.pose.position.x=-3.0;
        cmd_pose5.pose.position.y= 0.5;
        cmd_pose5.pose.position.z= 1.5;

}

void Deal_test_cmd::pub_next_wpt7()
{
    
        cmd_pose7.pose.position.x=-3.0;
        cmd_pose7.pose.position.y= -1.0;
        cmd_pose7.pose.position.z= 1.5;

}

void Deal_test_cmd::pub_next_wpt6()
{

        cmd_pose6.pose.position.x=-1.2;
        cmd_pose6.pose.position.y= 1.5;
        cmd_pose6.pose.position.z= 1.5;
    
}




void Deal_test_cmd::cmd_loop_cb(const ros::TimerEvent&)
{ 
    if(takeoff_flag){
        if(cmd_pose4.pose.position.x<=7)
        {
           cmd_pose4.pose.position.x+=0.02;
           cmd_pose5.pose.position.x+=0.02;
           cmd_pose6.pose.position.x+=0.02; 
           cmd_pose7.pose.position.x+=0.02;
           
           cmd_pose4.pose.position.y-=0.001;
           cmd_pose5.pose.position.y-=0.001;
           cmd_pose6.pose.position.y-=0.001; 
           cmd_pose7.pose.position.y-=0.001;
        }

    }
    else
    {   reach_wpt();
        pub_next_wpt4();
        pub_next_wpt5();
        pub_next_wpt6();
        pub_next_wpt7();
   
    }

    cmd_pub4.publish(cmd_pose4);
    cmd_pub5.publish(cmd_pose5);
    cmd_pub6.publish(cmd_pose6);
    cmd_pub7.publish(cmd_pose7);
    std::cout << "cmd_pose4=" << cmd_pose4.pose.position.x << std::endl;
    std::cout << "local_pose4=" << local_pose4.pose.position.x << std::endl;
    
    std::cout << "cmd_pose5=" << cmd_pose5.pose.position.x << std::endl;
    std::cout << "local_pose5=" << local_pose5.pose.position.x << std::endl;

    std::cout << "cmd_pose7=" << cmd_pose7.pose.position.x << std::endl;
    std::cout << "local_pose7=" << local_pose7.pose.position.x << std::endl;

    std::cout << "flag=" << takeoff_flag << std::endl;
    std::cout << "flagcount=" << flagcount << std::endl;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "cmd_pub");
    ros::NodeHandle nh("~");
    Deal_test_cmd test_cmd;
    test_cmd.cmd_loop = nh.createTimer(ros::Duration(0.1), &Deal_test_cmd::cmd_loop_cb,&test_cmd);

    test_cmd.cmd_pub4 = nh.advertise<geometry_msgs::PoseStamped>("/drone4/position_cmd",100);
    test_cmd.local_sub4 =  nh.subscribe<geometry_msgs::PoseStamped>("/drone4/mavros/local_position/pose",
                                         10,
                                         boost::bind(&Deal_test_cmd::local_cb4, &test_cmd, _1),
                                         ros::VoidConstPtr(),
                                         ros::TransportHints().unreliable().maxDatagramSize(1000));
    
    test_cmd.cmd_pub5 = nh.advertise<geometry_msgs::PoseStamped>("/drone5/position_cmd",100);
    test_cmd.local_sub5 =  nh.subscribe<geometry_msgs::PoseStamped>("/drone5/mavros/local_position/pose",
                                         10,
                                         boost::bind(&Deal_test_cmd::local_cb5, &test_cmd, _1),
                                         ros::VoidConstPtr(),
                                         ros::TransportHints().unreliable().maxDatagramSize(1000));
    
    test_cmd.cmd_pub6 = nh.advertise<geometry_msgs::PoseStamped>("/drone6/position_cmd",100);
    test_cmd.local_sub6 =  nh.subscribe<geometry_msgs::PoseStamped>("/drone6/mavros/local_position/pose",
                                         10,
                                         boost::bind(&Deal_test_cmd::local_cb6, &test_cmd, _1),
                                         ros::VoidConstPtr(),
                                         ros::TransportHints().unreliable().maxDatagramSize(1000));
    
    test_cmd.cmd_pub7 = nh.advertise<geometry_msgs::PoseStamped>("/drone7/position_cmd",100);
    test_cmd.local_sub7 =  nh.subscribe<geometry_msgs::PoseStamped>("/drone7/mavros/local_position/pose",
                                         10,
                                         boost::bind(&Deal_test_cmd::local_cb7, &test_cmd, _1),
                                         ros::VoidConstPtr(),
                                         ros::TransportHints().unreliable().maxDatagramSize(1000));
    ros::spin();
    return 0;
}
//左后x:-5 y 3.2
//左前x:8.6 y 3.2
//右前x 7.2 y-2.7
//我 x-5.2 y-2.7
