#include <boost/algorithm/string.hpp>
#include <boost/foreach.hpp>
#include <vector>
#include "gpsWaypoint.h"

namespace wp_control
{

  GpsWaypoint::GpsWaypoint() :
    m_nh("~"), m_speed(0.0), m_useThetaGPS(true), m_prevTime(0.0)
  {
    m_nh.param("WaypointFile", m_filename, std::string("waypoints.txt"));
    m_nh.param("WaypointRadius", m_wpRadius, 1.5);
    m_nh.param("HeadingP", m_headingP, 0.2);
    m_nh.param("Speed", m_speed, 1.0);
    m_nh.param("Skipto", skipto, 0);

    //m_odomSub = m_nh.subscribe("Odom", 1, &GpsWaypoint::Odomcb, this);
    //m_poseSub = m_nh.subscribe("Pose", 1, &GpsWaypoint::Posecb, this);

    m_odomSub = m_nh.subscribe("/novatel_data/inspvax", 1, &GpsWaypoint::orientcb, this);
    m_poseSub = m_nh.subscribe("/navsat/fix", 1, &GpsWaypoint::posecb, this);
    m_joySub = m_nh.subscribe("/joy", 1, &GpsWaypoint::recvJoy, this);


    m_twist_pub = m_nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    pub_wpts_ = m_nh.advertise<visualization_msgs::Marker>("/waypoints", 1);

    pub_timer_ = m_nh.createTimer(ros::Duration(0.1), &GpsWaypoint::pubWaypoints, this);

    m_ramp_start_time = ros::Time::now().toSec();
    m_ramp_target_speed = 0;
    joy_start_command = false;

    std::ifstream wptFile;
    ROS_INFO("Opening file %s", m_filename.c_str());
    wptFile.open(m_filename.c_str());
    std::string curLine;
//    m_paramTimer = m_nh.createTimer(ros::Rate(1),
//                   &GpsWaypoint::paramCallback, this);
    while(getline(wptFile, curLine))
    {
      double x, y;
      geometry_msgs::Point pt;
      geographic_msgs::GeoPoint geo_point;
      sensor_msgs::NavSatFix latlon_point;
      std::vector<std::string> strs;
      boost::split(strs, curLine, boost::is_any_of(","));
      if (strs.size() != 3){
        continue;
      }
      latlon_point.latitude = boost::lexical_cast<double>(strs[0]);
      latlon_point.longitude = boost::lexical_cast<double>(strs[1]);
      ROS_INFO("Loaded latlon waypoint %f %f", latlon_point.latitude, latlon_point.longitude);
      geo_point = geodesy::toMsg(latlon_point);
      geodesy::UTMPoint utm_point(geo_point);
      pt.x = utm_point.easting;
      pt.y = utm_point.northing;
      std::string abc = boost::lexical_cast<std::string>(strs[2]);
      ROS_INFO("Speed: %s", abc.c_str());
      pt.z = std::stod(abc);
     // pt.z = 4.4; 
    //  pt.z = boost::lexical_cast<double>(strs[2]);
      if(pt.z < 0){
        ROS_INFO("Loaded Stop Point %f %f", pt.x, pt.y);
      } else
        ROS_INFO("Loaded waypoint %f %f", pt.x, pt.y);
      m_wpts.push_back(pt);
    }
    wptFile.close();
    m_offsetX = m_wpts.front().x;
    m_offsetY = m_wpts.front().y;

    if(skipto>0){ //enables starting at midway point of WP file
      ROS_INFO("Starting at the %dth waypoint", skipto);
      for (int i = 0; i < skipto; i++) {
        m_wpts.push_back(m_wpts.front());
        m_wpts.pop_front();
      }
    }


    dynamic_reconfigure::Server<gpsWaypoint_paramsConfig>::CallbackType cb;

    cb = boost::bind(&GpsWaypoint::ConfigCallback, this, _1, _2);
    m_dynServer.setCallback(cb);

    ros::Duration d = ros::Duration(1.0);
    d.sleep();
  }

  GpsWaypoint::~GpsWaypoint()
  {}


  void GpsWaypoint::orientcb(novatel_msgs::INSPVAX msg)
  {
    double x, y,theta;
    double roll,pitch,yaw;

    m_lock.lock();
    roll= 0 * PI/180.0;
    pitch= 0 * PI/180.0;
    yaw= (msg.azimuth) * PI/180.0 + M_PI_2;

    //m_prevPos = m_position;
    //m_position = position;
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

    //if(yaw > M_PI)
	//yaw = yaw - 2*M_PI;
    theta = yaw;
    m_yaw = yaw;
    m_lock.unlock();

    //if (m_useThetaGPS)
    //  theta = thetaGPS;
    //else
    //theta = yaw;
    // Now we have all the latest data, do some maths
    // Are we at the next waypoint?
    double xn = m_wpts.front().x;
    double yn = m_wpts.front().y;
    double zn = m_wpts.front().z;
    while (GetDist(x,y,xn,yn) < m_wpRadius)
    {
      // Get the next wp
      m_wpts.push_back(m_wpts.front());
      m_wpts.pop_front();
      xn = m_wpts.front().x;
      yn = m_wpts.front().y;
      zn = m_wpts.front().z;
      ROS_INFO("Hit a waypoint");
      //command.linear.x = 0.0;
    }
    // UTM is a ENU system
    // imugps node uses a local ENU system
    // so our heading is measured from the positive x axis, meaning 0 is pointing east
    // and this lets a normal x-y axis system drawn on paper represent our system with x pointing up.

    double bearing = atan2(xn-x, yn-y);
    double error = AngleDiff(bearing, theta);

    geometry_msgs::Twist command;

    //if (zn < 0){
     // joy_start_command = false;
    //}
    //if(joy_start_command){
     // ROS_INFO("START");
      m_speed = m_wpts.front().z;
   // } else {
     // m_speed = 0;
    //}

    command.linear.x = m_speed;
    command.angular.z = Clamp(-m_headingP * error,-1.0,1.0);
    //if(fabs(command.angular.z) < 0.1)
	//command.angular.z = 0.0;
    //command.angular.z = 0.3;
    //command.linear.x = 3.0;

    //command.header.stamp = ros::Time::now();
    //command.sender = "waypointFollower";
    m_twist_pub.publish(command);
    double time = ros::Time::now().toSec();
    if (time > (m_prevTime + 0.1))
    {
      m_prevTime = time;
       std::cout << " Error " << error << " Steering " << command.angular.z;
       std::cout << " At theta " << theta*180/PI << " bearing " << bearing*180/PI;
       std::cout << " x " << x << " y " << y << std::endl;
       std::cout << " xn " << xn << " yn " << yn << std::endl;
       std::cout << " dx " << deltaX<< " dy " << deltaY << std::endl;

    }

  }
void GpsWaypoint::posecb(sensor_msgs::NavSatFix pose)
{
    geographic_msgs::GeoPoint geo_point;
    // sensor_msgs::NavSatFix latlon;
    m_lock.lock();
    m_prev_point = m_point;
    //m_pose = pose;
    // latlon = pose;
    // latlon.latitude=pose.latitude;
    // latlon.longitude= pose.longitude;
    geo_point = geodesy::toMsg(pose);
    geodesy::UTMPoint utm_point(geo_point);
    m_point.x = utm_point.easting;
    m_point.y = utm_point.northing;
    //publish new tf from waypoint frame to golfcart frame
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(m_point.x - m_offsetX, m_point.y - m_offsetY, 0.0) );
    tf::Quaternion q;
    q.setRPY(0, 0, -(m_yaw-M_PI_2));
    transform.setRotation(q);

    m_lock.unlock();

    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "waypoint", "golfcart"));
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

  double GpsWaypoint::calcSpeed()
  {
    double time = ros::Time::now().toSec();
    //calculate desired speed based on time since ramp was set and m_speed
    double finishtime = m_ramp_start_time+fabs((m_old_ramp_target_speed-m_ramp_target_speed)/m_ramp_rate);
    if(time >= finishtime)
      return m_ramp_target_speed;
    return (time - m_ramp_start_time)/(finishtime-m_ramp_start_time)*(m_ramp_target_speed-m_old_ramp_target_speed)+m_old_ramp_target_speed;
  }

  double GpsWaypoint::setRamp(double set_speed, double rate, double start_time)
  {
    m_old_ramp_target_speed = calcSpeed(); // in case speed ramp target is changed before speed is reached
    m_ramp_rate = rate;
    m_ramp_start_time = start_time;
    m_ramp_target_speed = set_speed;
  }

  void GpsWaypoint::recvJoy(const sensor_msgs::Joy::ConstPtr& msg)
  {
    if(msg->buttons[7] && m_ramp_target_speed == 0){
      joy_start_command = true;
    }
  }


  void GpsWaypoint::ConfigCallback(const gpsWaypoint_paramsConfig &config, uint32_t level)
  {
    m_headingP = config.pGain;
    m_useThetaGPS = config.gpsHeading;
    m_wpRadius = config.wpRadius;
    std::cout << "Got a config!!" << std::endl;
  }

  void GpsWaypoint::pubWaypoints(const ros::TimerEvent& event){
    visualization_msgs::Marker points;
    points.header.frame_id =  "waypoint";
    points.header.stamp = ros::Time::now();
    points.ns = "wp_control";
    points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = 1.0;
    points.id = 0;
    points.type = visualization_msgs::Marker::POINTS;
    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 1;
    points.scale.y = 1;
    // Points are blue
    points.color.b = 1.0f;
    points.color.a = 1.0;

    // Create the vertices for the points and lines
    bool skip_first = true;
    BOOST_FOREACH (const geometry_msgs::Point &pt, m_wpts){
      if(skip_first){
        skip_first = false;
        continue;
      }
      geometry_msgs::Point p;
      p.x = pt.x - m_offsetX;
      p.y = pt.y - m_offsetY;
      p.z = 0;
      points.points.push_back(p);
    }
    pub_wpts_.publish(points);
    points.points.clear();
    points.id = 1;
    points.color.b = 0.0f;
    points.color.r = 1.0f;
    points.color.a = 1.0;
    geometry_msgs::Point p;
    p.x = m_wpts.front().x - m_offsetX;
    p.y = m_wpts.front().y - m_offsetY;
    p.z = 0;
    points.points.push_back(p);
    pub_wpts_.publish(points);

  }
};

int main (int argc, char** argv)
{
  ros::init(argc, argv, "GpsWaypoint");
  //ros::NodeHandle n;
  wp_control::GpsWaypoint wpt;
  ros::spin();
}
