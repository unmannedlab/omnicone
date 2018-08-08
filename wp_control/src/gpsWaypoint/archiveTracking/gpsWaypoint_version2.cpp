
//This is the version of the code that works! Note: It could be because the input data was logged at a faster rate
//This code considers (X,Y,t,v,psidot) as inputs
//All the data is changed to the vehicle frame
//The controller is basically a feed forward term plus an error term for both velocity and yawrate
//There were some minor issues with this controller

#include <boost/algorithm/string.hpp>
#include <vector>
#include "gpsWaypoint.h"

namespace wp_control
{

  GpsWaypoint::GpsWaypoint() :
    m_nh("~"), m_speed(0.0), m_useThetaGPS(true), m_prevTime(0.0)
  {
    m_nh.param("WaypointFile", m_filename, std::string("waypoints.txt"));
    m_nh.param("WaypointRadius", m_wpRadius, 5.0);
    m_nh.param("HeadingP", m_headingP, 0.0001);
    m_nh.param("LookaheadDist",m_lookahead,1.0);
    m_nh.param("VelocityP", m_distanceP, 0.01);
    
    //m_odomSub = m_nh.subscribe("Odom", 1, &GpsWaypoint::Odomcb, this);
    //m_poseSub = m_nh.subscribe("Pose", 1, &GpsWaypoint::Posecb, this);

    m_initTime = ros::Time::now().toSec();
    
    m_odomSub = m_nh.subscribe("/mti/filter/orientation", 1, &GpsWaypoint::orientcb, this);
    m_poseSub = m_nh.subscribe("/mti/filter/position", 1, &GpsWaypoint::posecb, this);

    m_twist_pub = m_nh.advertise<geometry_msgs::Twist>("/vehicle/cmd_vel", 1);
    m_gear_pub = m_nh.advertise<dbw_mkz_msgs::GearCmd>("gear_cmd", 1);

    std::ifstream wptFile;
    ROS_INFO("Opening file %s", m_filename.c_str());
    wptFile.open(m_filename.c_str());
    std::string curLine;
//    m_paramTimer = m_nh.createTimer(ros::Rate(1),
//                   &GpsWaypoint::paramCallback, this);
    while(getline(wptFile, curLine))
    {
      double x, y, velocity, yawrate;
      geometry_msgs::Point pt;
      boost::array<double,5> state;
      geographic_msgs::GeoPoint geo_point;
      sensor_msgs::NavSatFix latlon_point;
      std::vector<std::string> strs;
      boost::split(strs, curLine, boost::is_any_of(","));
      if (strs.size() == 5)
      latlon_point.latitude = boost::lexical_cast<double>(strs[0]);
      latlon_point.longitude = boost::lexical_cast<double>(strs[1]);
      m_timeStamp = boost::lexical_cast<double>(strs[2]);
      velocity = boost::lexical_cast<double>(strs[3]);
      yawrate = boost::lexical_cast<double>(strs[4]);
      //ROS_INFO("lat long time vel rate %f %f %f %f %f",latlon_point.latitude, latlon_point.longitude, m_timeStamp, velocity, yawrate);
      geo_point = geodesy::toMsg(latlon_point);
      geodesy::UTMPoint utm_point(geo_point);
      pt.x = utm_point.easting;
      pt.y = utm_point.northing;
      pt.z = m_timeStamp;
      state = {pt.x,pt.y,m_timeStamp,velocity,yawrate};
      //ROS_INFO("Loaded waypoint %f %f", pt.x, pt.y);
      m_wpts.push_back(pt);
      m_states.push_back(state);
    }

    m_timePeriod = m_states.back()[2];

    wptFile.close();
    //m_offsetX = m_wpts.front().x;
    //m_offsetY = m_wpts.front().y;

    //dynamic_reconfigure::Server<gpsWaypoint_paramsConfig>::CallbackType cb;

    //cb = boost::bind(&GpsWaypoint::ConfigCallback, this, _1, _2);
    //m_dynServer.setCallback(cb);
    //dbw_mkz_msgs::GearCmd gear_msg;
    //gear_msg.cmd.gear = dbw_mkz_msgs::Gear::DRIVE;
    //m_gear_pub.publish(gear_msg);
    ros::Duration d = ros::Duration(1.0);
    d.sleep();
  }

  GpsWaypoint::~GpsWaypoint()
  {}
  

  void GpsWaypoint::orientcb(custom_msgs::orientationEstimate orient)
  {
    double speed;
    double x, y,theta;
    double roll,pitch,yaw;

    m_lock.lock();
    m_orient = orient;
    roll= orient.roll * PI/180.0;
    pitch= orient.pitch * PI/180.0;
    yaw= orient.yaw * PI/180.0;
    
    //m_prevPos = m_position;
    //m_position = position;
    speed = m_speed;
    x = m_point.x;
    y = m_point.y;
    // Get our heading from the message
    //tf::Quaternion quat(position.pose.pose.orientation.x,
    //                  position.pose.pose.orientation.y,
    //                  position.pose.pose.orientation.z,
    //                  position.pose.pose.orientation.w);
    //tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    
    //double deltaX = x - m_prevPos.pose.pose.position.x;
    //double deltaY = y - m_prevPos.pose.pose.position.y;
    double deltaX = x - m_prev_point.x;
    double deltaY = y - m_prev_point.y;
    double thetaGPS = atan2(deltaY,deltaX);
    //theta = thetaGPS;
    theta = yaw;
    m_lock.unlock();

    //if (m_useThetaGPS)
    //  theta = thetaGPS;
    //else
    //theta = yaw;
    // Now we have all the latest data, do some maths
    // Are we at the next waypoint?
    
    //double xn = m_wpts.front().x;
    //double yn = m_wpts.front().y;
    //while (GetDist(x,y,xn,yn) < -m_wpRadius)
    //{
      // Get the next wp
      //m_wpts.push_back(m_wpts.front());
      //m_wpts.pop_front();
      //xn = m_wpts.front().x;
      //yn = m_wpts.front().y;
      //ROS_INFO("Hit a waypoint");
      //command.linear.x = 0.0;
    //}

    double time = ros::Time::now().toSec(); 
    double timeint = fmod((time-m_initTime),m_timePeriod);
    //std::list<geometry_msgs::Point> temp_wpts;
    std::list<boost::array<double,5>> temp_states;
    //temp_wpts = m_wpts;
    temp_states = m_states;

    bool noTarget = true;
    double targetYaw = 0;
    double xn = temp_states.front()[0];
    double yn = temp_states.front()[1];
    double tn = temp_states.front()[2];
    double targetVel = temp_states.front()[3];
    double targetYawrate = temp_states.front()[4];

    while (noTarget)
    {
      temp_states.push_back(temp_states.front());
      temp_states.pop_front();

      if (timeint-tn<0.5) { 
	 noTarget = false;
         targetYaw = atan2(temp_states.front()[1]-yn,temp_states.front()[0]-xn);

         //if (temp_states.front()[2] > 0.0){
           //targetVel = temp_states.front()[3]; 
           //sqrt( pow(temp_states.front()[0]-xn,2) + pow(temp_states.front()[1]-yn,2) )/abs(temp_states.front()[2]-tn);
         //}else{
         //  targetVel = 0;
         //}
         //ROS_INFO("Hit a waypoint");
      }else {
         xn = temp_states.front()[0];
         yn = temp_states.front()[1];
         tn = temp_states.front()[2];
         targetVel = temp_states.front()[3];
         targetYawrate = temp_states.front()[4];
      } 	
    }		
    
    // UTM is a ENU system
    // imugps node uses a local ENU system
    // so our heading is measured from the positive x axis, meaning 0 is pointing east
    // and this lets a normal x-y axis system drawn on paper represent our system with x pointing up.

    double bearing = atan2(yn-y, xn-x);
    double angFrame = -(PI/2-theta);
    double lateralError = cos(angFrame)*(xn-x) + sin(angFrame)*(yn-y);
    double longError = -sin(angFrame)*(xn-x) + cos(angFrame)*(yn-y);

    double positionError = sqrt( pow(xn-x,2) + pow(yn-y,2) );
    if (positionError < m_wpRadius){
      positionError = 0;
      bearing = targetYaw;
    }
    
    double headingError = AngleDiff(bearing, theta);

    geometry_msgs::Twist command;
    //command.linear.x = Clamp(targetVel + m_distanceP * positionError, 0 , 5.0);
    //command.angular.z = Clamp(m_headingP * headingError,-1.0,1.0);

    command.linear.x = Clamp(targetVel - m_distanceP * longError, 0 , 5.0);
    command.angular.z = Clamp(targetYawrate - m_headingP * lateralError,-1.0,1.0);

    //if(fabs(command.angular.z) < 0.1)
	//command.angular.z = 0.0;
    //command.angular.z = 0.3;
    //command.linear.x = 3.0;

    //command.header.stamp = ros::Time::now();
    //command.sender = "waypointFollower";
    m_twist_pub.publish(command);

    //double time = ros::Time::now().toSec();

    if (time > (m_prevTime + 1))
    {
      m_prevTime = time;
       std::cout << "   " << std::endl;
       std::cout << m_states.back()[2]   << std::endl;
       std::cout << " lateral error " << lateralError << " Long error " << longError << std::endl;
       std::cout << " target yaw rate " << targetYawrate << " cmd " << m_headingP*lateralError << std::endl;
       std::cout << " time lapsed " << (time-m_initTime) << " Time check " << timeint << std::endl;
       //std::cout << " position error " << positionError << std::endl;
       std::cout << " Target velocity " << targetVel << " additional velocity " << m_distanceP * longError << std::endl;
       std::cout << " cmd velocity " << command.linear.x << std::endl;
       //std::cout << " targetYaw " << targetYaw*180/PI << " Yaw " << theta*180/PI << std::endl;
       //std::cout << " Error " << headingError*180/PI << " Steering " << command.angular.z;
       //std::cout << " At theta " << bearing*180/PI << " bearing " << theta*180/PI;
       //std::cout << " x " << x << " y " << y << std::endl;
       //std::cout << " xn " << xn << " yn " << yn << " tn " << tn << std::endl;
       //std::cout << " dx " << deltaX<< " dy " << deltaY << std::endl;

    }

  }
void GpsWaypoint::posecb(custom_msgs::positionEstimate pose)
{
    geographic_msgs::GeoPoint geo_point;
    sensor_msgs::NavSatFix latlon;
    m_lock.lock();
    m_prev_point = m_point;
    //m_pose = pose;
    latlon.latitude=pose.latitude;
    latlon.longitude= pose.longitude;
    geo_point = geodesy::toMsg(latlon);
    geodesy::UTMPoint utm_point(geo_point);
    m_point.x = utm_point.easting;
    m_point.y =utm_point.northing;
    m_lock.unlock();
}
//void GpsWaypoint::paramCallback(const ros::TimerEvent& time)
//{
//  m_nh.param("HeadingP", m_headingP, 2.0);
//}

  double GpsWaypoint::GetDist(double x1, double y1, double x2, double y2)
  {
    return sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
  }

  double GpsWaypoint::AngleDiff(double a, double b)
  {
    double dif = fmod(a-b + PI, 2.0*PI);
    if (dif < 0)
      dif += 2.0*PI;
    return dif - PI;
  }

  double GpsWaypoint::Clamp(double num, double min, double max)
  {
    if (num < min)
      num = min;
    if (num > max)
      num = max;
    return num;
  }


  void GpsWaypoint::ConfigCallback(const gpsWaypoint_paramsConfig &config, uint32_t level)
  {
    m_headingP = config.pGain;
    m_useThetaGPS = config.gpsHeading;
    m_wpRadius = config.wpRadius;
    std::cout << "Got a config!!" << std::endl;
  }
};

int main (int argc, char** argv)
{
  ros::init(argc, argv, "GpsWaypoint");
  //ros::NodeHandle n;
  wp_control::GpsWaypoint wpt;
  ros::spin();
}
