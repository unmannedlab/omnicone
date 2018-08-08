#include<ros/ros.h>
#include<std_msgs/Float64.h>
#include<std_msgs/Bool.h>
#include<list>
#include<pacmod_msgs/PacmodCmd.h>
ros::Publisher accel_pub; 
ros::Publisher brake_pub; 
float prev_speed = 0;
float brake;
ros::Time prev_time;
void speedCB(const std_msgs::Float64::ConstPtr msg){
	std_msgs::Float64 change;
        change.data = (msg->data - prev_speed)/(ros::Time::now() - prev_time).toSec();
        accel_pub.publish(change);
        prev_speed = msg->data;
        prev_time = ros::Time::now();
}
void EnableCB(const std_msgs::Bool::ConstPtr msg){
        pacmod_msgs::PacmodCmd cmd;
        if(msg->data){
           cmd.enable=true;
           cmd.f64_cmd=brake;
           brake_pub.publish(cmd);
           ROS_INFO("publishing brake = %f", brake);
        }
}
int main(int argc, char** argv){
	ros::init(argc, argv, "pub_accel");
        ros::NodeHandle nh("~");
        nh.param("brake", brake, float(0));
        prev_time = ros::Time::now();
        ros::Subscriber sub =  nh.subscribe("/pacmod/as_tx/vehicle_speed", 1, speedCB);
        ros::Subscriber sub2 =  nh.subscribe("/pacmod/as_tx/enable", 1, EnableCB);
        accel_pub = nh.advertise<std_msgs::Float64>("accel", 1);
        brake_pub = nh.advertise<pacmod_msgs::PacmodCmd>("/pacmod/as_rx/brake_cmd", 1);
        ros::spin();
}

