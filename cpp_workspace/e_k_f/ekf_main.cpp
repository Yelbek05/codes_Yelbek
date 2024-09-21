#include "ros/ros.h"
#include "std_msgs/String.h"
#include "EKF.hpp"
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <sensor_msgs/NavSatFix.h>
#include <iostream>
#include <eigen3/Eigen/Core>

class EKFNode
{
public:
    EKFNode(const ros::NodeHandle& nh) : m_nh{nh}
    {
        m_sub = m_nh.subscribe<sensor_msgs::NavSatFix>("/gps/nova/fix", 1, &EKFNode::gpsCallback, this);
    
        // ros::Publisher topic_pub = nh.advertise<std_msgs::String>("tutorical",1000);
        // ros::Rate loop_rate(1);
    }

    void run()
    {}

private:

    void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
    {
        Eigen::Vector3d measurement(msg->latitude, msg->longitude, msg->altitude);
        m_kf.predict();
        m_kf.update(measurement);
        std::cout << "EKF done: " << m_kf.getState() << std::endl;
    }

    ros::NodeHandle m_nh;
    KalmanFilter m_kf; ///< 
    ros::Subscriber m_sub;

    //vector<> measurements;
    //vector<> states;
    //
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "square_node");
    ros::NodeHandle nh;

    EKFNode ekf_node(nh);
    ekf_node.run();

    ros::spin();

    return 0;
}


// void callback(msg)
// {
//     kf.predict();
//     kf.update(msg.imu.data);
// }

// #include "ros/ros.h"
// #include "std_msgs/String.h"
// #include "Eigen/Dense"

// int main(int argc,char **argv){
//     subscriber("/topic")
// }

// void callback(msg){
//     kf.predict();
//     kf.update(msg.imu.data);
// }