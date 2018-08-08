
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


#define PI 3.14159265358979323846264338
#define point_ geometry_msgs::Point
#define MINRAD 5.258

namespace wp_control
{
  template <typename T> double sgn(T val) {
    return (T(0) < val) - (val < T(0));
  };
  struct vec {
    double x;
    double y;
  };
  struct golfcart {
    double x;
    double y;
    double ang;
  };
  class PurePursuit
  {
  public:
    PurePursuit();
    ~PurePursuit();
  private:
    ros::NodeHandle m_nh;
    ros::Subscriber m_odomSub;
    ros::Subscriber m_poseSub;
    ros::Subscriber m_joySub;
    ros::Subscriber m_imuSub;
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
    double look_dist;
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
    double m_angular_rate;
    //double m_prevx, m_prevy,m_x, m_y;

    boost::mutex m_lock;

    //void Odomcb(nav_msgs::Odometry position);
    //void Posecb(sensor_msgs::Imu pose);
    void pubWaypoints(const ros::TimerEvent& event);
    void orientcb(novatel_msgs::INSPVAX msg);
    void posecb(sensor_msgs::NavSatFix pose);
    void recvJoy(const sensor_msgs::Joy::ConstPtr& msg);
    void recvImu(sensor_msgs::Imu msg);
    void paramCallback(const ros::TimerEvent& time);
    double GetDist(double x1, double y1, double x2, double y2);
    double AngleDiff(double a, double b);
    double Clamp(double num, double min, double max);
  };
  double dot(vec a, vec b){
    return a.x*b.x+a.y*b.y;
  };
  double dist(point_ A, point_ B){
    return sqrt(pow(B.x-A.x,2) + pow(B.y-A.y,2));
  };
  vec proj(vec a, vec b){
    vec c;
    double scale = dot(a,b)/(pow(a.x,2)+pow(a.y,2));
    if(scale < 0){
      scale = 0;
    }
    else if (scale > 1){
      scale = 1;
    }
    c.x = scale*a.x;
    c.y = scale*a.y;
    return c;
  };
  void transform(point_ &pt,golfcart gc){
    double xt = pt.x - gc.x;
    double yt = pt.y - gc.y;
    pt.x = xt*cos(-gc.ang) + yt*sin(-gc.ang);
    pt.y = yt*cos(-gc.ang) - xt*sin(-gc.ang);
    return;
  };
  void untransform(point_ &pt,golfcart gc){
    double xr = pt.x*cos(gc.ang) + pt.y*sin(gc.ang);
    double yr = pt.y*cos(gc.ang) - pt.x*sin(gc.ang);
    pt.x = xr + gc.x;
    pt.y = yr + gc.y;
    return;
  };
  bool onsegment(point_ A, point_ B, point_ pt){
    if((pt.x >= A.x && pt.x <= B.x) || (pt.x <= A.x && pt.x >= B.x)){
      if((pt.y >= A.y && pt.y <= B.y) || (pt.y <= A.y && pt.y >= B.y)){
        return true;
      }
    }
    return false;
  };
};
#endif
