#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_node");

    ros::NodeHandle nh("~");

    float vel_linear_x ;
    float vel_angular_z ;

    ros::Publisher smooth_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("/smoother_cmd_vel", 1, true);

    ros::Rate rate(20);

    while (ros::ok()) {
        geometry_msgs::TwistStamped cmd_vel_stamp;
nh.param<float>("vel_linear_x",vel_linear_x,0.0);  
nh.param<float>("vel_angular_z",vel_angular_z,0.0); 

        cmd_vel_stamp.header.frame_id = "odom";
        cmd_vel_stamp.header.stamp = ros::Time::now();

//ROS_INFO("vel_linear_x=%f",vel_linear_x);
//ROS_INFO("vel_angular_z=%f",vel_angular_z);

        cmd_vel_stamp.twist.linear.x = vel_linear_x;
        cmd_vel_stamp.twist.angular.z = vel_angular_z;

        smooth_vel_pub.publish(cmd_vel_stamp);

        rate.sleep();
    }

    return 0;
}
