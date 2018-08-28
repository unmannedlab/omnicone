
#ifndef GPS_WAYPOINT_H_
#define GPS_WAYPOINT_H_

#include <list>
#include <iostream>
#include <fstream>

#include <boost/thread.hpp>          // Mutex
#include <boost/lexical_cast.hpp>

#include <ros/ros.h>
#include <ros/time.h>
#include <tf/transform_listener.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <novatel_msgs/INSPVAX.h>

#include <geodesy/wgs84.h>
#include <geodesy/utm.h>

#include <dynamic_reconfigure/server.h>
#include <wp_control/gpsWaypoint_paramsConfig.h>

#define PI 3.14159265358979323846264338

namespace wp_control
{
  class GpsWaypoint
  {
  public:
    GpsWaypoint();
    ~GpsWaypoint();
  private:
    ros::NodeHandle m_nh;
    ros::Subscriber m_odomSub;
    ros::Subscriber m_poseSub;
    ros::Subscriber m_joySub;
    ros::Publisher  m_twist_pub;
    ros::Publisher pub_wpts_;
    ros::Timer m_paramTimer;
    ros::Timer pub_timer_;



    std::string m_filename;
    std::ofstream outputFile;
    std::ofstream inputFile;

    std::list<geometry_msgs::Point> m_wpts;
    std::list<boost::array<double,6>> m_states;

//    sensor_msgs::Imu m_pose;
    std_msgs::Float64 m_orient;
    sensor_msgs::NavSatFix m_pose;
    geometry_msgs::Point m_point;
    geometry_msgs::Point m_prev_point;
    //nav_msgs::Odometry m_position;
    //nav_msgs::Odometry m_prevPos;
    double m_speed;
    double m_yaw;
    double m_wpRadius;
    double m_lookahead;
    double m_longDistP;
    double m_latDistP;
    double m_headingP;
    bool m_useThetaGPS;
    bool joy_start_command;
    double m_offsetX, m_offsetY;
    double m_prevTime;
    double m_initTime;
    double m_timeStamp;
    double m_timePeriod;
    double m_ramp_start_time;
    double m_ramp_target_speed;
    double m_old_ramp_target_speed;
    double m_ramp_rate;
    int skipto;
    //double m_prevx, m_prevy,m_x, m_y;
    dynamic_reconfigure::Server<gpsWaypoint_paramsConfig> m_dynServer;

    boost::mutex m_lock;

    //void Odomcb(nav_msgs::Odometry position);
    //void Posecb(sensor_msgs::Imu pose);
    void pubWaypoints(const ros::TimerEvent& event);
    void orientcb(novatel_msgs::INSPVAX msg);
    void posecb(sensor_msgs::NavSatFix pose);
    void recvJoy(const sensor_msgs::Joy::ConstPtr& msg);
    void paramCallback(const ros::TimerEvent& time);
    double GetDist(double x1, double y1, double x2, double y2);
    double AngleDiff(double a, double b);
    double Clamp(double num, double min, double max);
    void ConfigCallback(const gpsWaypoint_paramsConfig &config, uint32_t level);
    double calcSpeed();
    double setRamp(double set_speed, double rate, double start_time);
  };
};
#endif
