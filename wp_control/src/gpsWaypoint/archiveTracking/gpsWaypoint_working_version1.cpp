
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
    m_nh.param("HeadingP", m_headingP, 0.05);
    m_nh.param("LookaheadDist",m_lookahead,1.0);
    m_nh.param("VelocityP", m_distanceP, 0.01);
    
    m_initTime = ros::Time::now().toSec();
    
    m_odomSub = m_nh.subscribe("/mti/filter/orientation", 1, &GpsWaypoint::orientcb, this);
    m_poseSub = m_nh.subscribe("/mti/filter/position", 1, &GpsWaypoint::posecb, this);

    m_twist_pub = m_nh.advertise<geometry_msgs::Twist>("/vehicle/cmd_vel", 1);
    m_gear_pub = m_nh.advertise<dbw_mkz_msgs::GearCmd>("gear_cmd", 1);

    std::ifstream wptFile;
    ROS_INFO("Opening file %s", m_filename.c_str());
    wptFile.open(m_filename.c_str());
    std::string curLine;
    
    while(getline(wptFile, curLine))
    {
      double x, y, velocity, yawrate, targetYaw;
      geometry_msgs::Point pt;
      boost::array<double,6> state;
      geographic_msgs::GeoPoint geo_point;
      sensor_msgs::NavSatFix latlon_point;
      std::vector<std::string> strs;
      boost::split(strs, curLine, boost::is_any_of(","));
      if (strs.size() == 6)
      latlon_point.latitude = boost::lexical_cast<double>(strs[0]);
      latlon_point.longitude = boost::lexical_cast<double>(strs[1]);
      m_timeStamp = boost::lexical_cast<double>(strs[2]);
      velocity = boost::lexical_cast<double>(strs[3]);
      yawrate = boost::lexical_cast<double>(strs[4]);
      targetYaw = boost::lexical_cast<double>(strs[5]) * PI/180;
      //ROS_INFO("lat long time vel rate %f %f %f %f %f",latlon_point.latitude, latlon_point.longitude, m_timeStamp, velocity, yawrate);
      geo_point = geodesy::toMsg(latlon_point);
      geodesy::UTMPoint utm_point(geo_point);
      pt.x = utm_point.easting;
      pt.y = utm_point.northing;
      pt.z = m_timeStamp;
      state = {pt.x,pt.y,m_timeStamp,velocity,yawrate,targetYaw};
      //ROS_INFO("Loaded waypoint %f %f", pt.x, pt.y);
      m_wpts.push_back(pt);
      m_states.push_back(state);
    }

    m_timePeriod = m_states.back()[2];
    wptFile.close();
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
    
    speed = m_speed;
    x = m_point.x;
    y = m_point.y;
    
    double deltaX = x - m_prev_point.x;
    double deltaY = y - m_prev_point.y;
    double thetaGPS = atan2(deltaY,deltaX);
    //theta = thetaGPS;
    theta = yaw;
    m_lock.unlock();

    double time = ros::Time::now().toSec(); 
    double timeint = fmod((time-m_initTime),m_timePeriod);
    std::list<boost::array<double,6>> temp_states;
    temp_states = m_states;

    bool noTarget = true;
    //double targetYaw = 0;
    double xn = temp_states.front()[0];
    double yn = temp_states.front()[1];
    double tn = temp_states.front()[2];
    double targetVel = temp_states.front()[3];
    double targetYawrate = temp_states.front()[4];
    double targetYaw = temp_states.front()[5];
    //double checkVel;
    
    while (noTarget)
    {
      temp_states.push_back(temp_states.front());
      temp_states.pop_front();

      if (timeint-tn<=0.02) { 
	 noTarget = false;
         //targetYaw = atan2(temp_states.front()[1]-yn,temp_states.front()[0]-xn);
         
	 //checkVel = sqrt( pow(temp_states.front()[0]-xn,2) + pow(temp_states.front()[1]-yn,2) )/(temp_states.front()[2]-tn);
         //ROS_INFO("Hit a waypoint");
      }else {
         xn = temp_states.front()[0];
         yn = temp_states.front()[1];
         tn = temp_states.front()[2];
         targetVel = temp_states.front()[3];
         targetYawrate = temp_states.front()[4];
         targetYaw = temp_states.front()[5];
      } 	
    }		
    
    // UTM is a ENU system
    // imugps node uses a local ENU system
    // so our heading is measured from the positive x axis, meaning 0 is pointing east
    // and this lets a normal x-y axis system drawn on paper represent our system with x pointing up.

    double k1 = m_distanceP; //2*m_distanceP*sqrt( pow(targetYawrate,2) + m_headingP*pow(targetVel,2));
    double k2 = m_headingP; //m_headingP*targetVel;
    double bearing = atan2(yn-y, xn-x);
    double angFrame = -(PI/2-theta);
    double lateralError = cos(angFrame)*(xn-x) + sin(angFrame)*(yn-y);
    double longError = -sin(angFrame)*(xn-x) + cos(angFrame)*(yn-y);
    double targetVelY = targetVel*sin(targetYaw + PI/2-theta);
    double headingError = AngleDiff(bearing, theta);

    geometry_msgs::Twist command;
    command.linear.x = Clamp(targetVelY + k1 * longError, 0 , 10.0);
    command.angular.z = Clamp(targetYawrate - k2 * lateralError,-1.0,1.0);

    m_twist_pub.publish(command);

    if (time > (m_prevTime + 1))
    {
      m_prevTime = time;
       std::cout << "   " << std::endl;
       std::cout << m_states.back()[2]   << std::endl;
       std::cout << " lateral error " << lateralError << " Long error " << longError << std::endl;
       std::cout << " target yaw rate " << targetYawrate << " cmd " << k2*lateralError << " target yaw "<< targetYaw*180/PI  << std::endl;
       std::cout << " time lapsed " << (time-m_initTime) << " Time check " << timeint << std::endl;
       //std::cout << " position error " << positionError << std::endl;
       std::cout << " Target velocity in Y " << targetVelY << " additional velocity " << k1*longError << std::endl;
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
