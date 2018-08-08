#include <boost/algorithm/string.hpp>
#include <boost/foreach.hpp>
#include <vector>
#include "purePursuit.h"

namespace wp_control
{

  PurePursuit::PurePursuit() :
    m_nh("~"), m_speed(0.0), m_useThetaGPS(true), m_prevTime(0.0)
  {
    m_nh.param("WaypointFile", m_filename, std::string("waypoints.txt"));
    m_nh.param("WaypointRadius", m_wpRadius, 1.5);
    m_nh.param("Lookahead", look_dist, 5.0);
    // m_nh.param("Speed", m_speed, 1.0);

    //m_odomSub = m_nh.subscribe("Odom", 1, &PurePursuit::Odomcb, this);
    //m_poseSub = m_nh.subscribe("Pose", 1, &PurePursuit::Posecb, this);

    m_odomSub = m_nh.subscribe("/novatel_data/inspvax", 1, &PurePursuit::orientcb, this);
    m_poseSub = m_nh.subscribe("/navsat/fix", 1, &PurePursuit::posecb, this);
    m_joySub = m_nh.subscribe("/joy", 1, &PurePursuit::recvJoy, this);
    m_imuSub = m_nh.subscribe("/imu/data", 1, &PurePursuit::recvImu, this);


    m_twist_pub = m_nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    pub_wpts_ = m_nh.advertise<visualization_msgs::Marker>("/waypoints", 1);

    pub_timer_ = m_nh.createTimer(ros::Duration(0.1), &PurePursuit::pubWaypoints, this);

    m_ramp_start_time = ros::Time::now().toSec();
    m_ramp_target_speed = 0;
    joy_start_command = false;

    std::ifstream wptFile;
    ROS_INFO("Opening file %s", m_filename.c_str());
    wptFile.open(m_filename.c_str());
    std::string curLine;
//    m_paramTimer = m_nh.createTimer(ros::Rate(1),
//                   &PurePursuit::paramCallback, this);
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
      // pt.z = boost::lexical_cast<double>(strs[2]);
      if(pt.z < 0){
        ROS_INFO("Loaded Stop Point %f %f", pt.x, pt.y);
      } else
        ROS_INFO("Loaded waypoint %f %f", pt.x, pt.y);
      m_wpts.push_back(pt);
    }
    wptFile.close();
    m_offsetX = m_wpts.front().x;
    m_offsetY = m_wpts.front().y;

    ros::Duration d = ros::Duration(1.0);
    d.sleep();
  }

  PurePursuit::~PurePursuit()
  {}
  void PurePursuit::recvImu(sensor_msgs::Imu msg){
      m_angular_rate = -msg.angular_velocity.z;
  }

  void PurePursuit::orientcb(novatel_msgs::INSPVAX msg)
  {
    double x, y,theta;
    double roll,pitch,yaw;
    m_lock.lock();
    double act_speed = sqrt(pow(msg.north_velocity,2) + pow(msg.east_velocity,2));
    roll= 0 * PI/180.0;
    pitch= 0 * PI/180.0;
    yaw= ((msg.azimuth) * PI/180.0 - M_PI_2 - M_PI);
    x = m_point.x;
    y = m_point.y;
    double deltaX = x - m_prev_point.x;
    double deltaY = y - m_prev_point.y;
    double thetaGPS = atan2(deltaY,deltaX);
    theta = yaw;
    m_yaw = yaw;
    m_lock.unlock();

    point_ goal;
    bool found_cand;
    vec ab;
    vec ac;
    vec bc;
    point_ A;
    point_ B;
    point_ closest_point;
    golfcart gc;
    gc.x = x;
    gc.y = y;
    gc.ang = theta;

    //adjust golfcart position to where it will be in 1 second
    double timedelay = 0.0;
    double rad;
    double ang = 0;
    point_ newpos;
    if(m_angular_rate == 0){
      newpos.x = 0;
      newpos.y = act_speed*2*timedelay;
    } else {
      rad = act_speed/m_angular_rate;
      ang = m_angular_rate*2*timedelay;
      newpos.x = rad - rad*cos(ang);
      newpos.y = rad*sin(ang);
    }
    // newpos.x = 0;
    // newpos.y = 0;
    untransform(newpos,gc);
    gc.x = newpos.x;
    gc.y = newpos.y;
    gc.ang = ang + gc.ang;
    double min = 10000;
    for (std::list<point_>::const_iterator it = m_wpts.begin(), end = m_wpts.end(); it != end; ++it) {
      A = *it;
      if(dist(A,m_point) < min){
        min = dist(A,m_point);
        look_dist = A.z + 5 < 10? 10: A.z + 5;// < 10? 10: A.z+5;
      }


    }
    //calculate goal point, checking each line segment
    for (std::list<point_>::const_iterator it = m_wpts.begin(), end = m_wpts.end(); it != end; ++it) {
        A = *it;
        B = *(std::next(it,1));
        //project vector to golfcart onto path to find closest point on path
        ab.x = B.x - A.x;
        ab.y = B.y - A.y;
        ac.x = x - A.x;
        ac.y = y - A.y;
        bc = proj(ab,ac);
        //calc closest point
        closest_point.x = A.x + bc.x;
        closest_point.y = A.y + bc.y;
        //transform to golfcart coordinates
        transform(A,gc);
        transform(B,gc);
        transform(closest_point,gc);
        if(GetDist(0,0,closest_point.x,closest_point.y) <= look_dist){
          double dx = B.x-A.x;
          double dy = B.y-A.y;
          double dr = sqrt(pow(dx,2)+pow(dy,2));
          double D = A.x*B.y-B.x*A.y;
          double xops [] = {(D*dy+sgn(dy)*dx*sqrt(pow(look_dist,2)*pow(dr,2)-pow(D,2)))/(pow(dr,2)), (D*dy-sgn(dy)*dx*sqrt(pow(look_dist,2)*pow(dr,2)-pow(D,2)))/(pow(dr,2))};
          double yops [] = {(-D*dx+fabs(dy)*sqrt(pow(look_dist,2)*pow(dr,2)-pow(D,2)))/(pow(dr,2)), (-D*dx-fabs(dy)*sqrt(pow(look_dist,2)*pow(dr,2)-pow(D,2)))/(pow(dr,2))};
          if(pow(look_dist,2)*pow(dr,2)-pow(D,2) > 0){
            //check which is closer to B, and make sure it is between A & B
            point_ pt1;
            pt1.x = xops[0];
            pt1.y = yops[0];
            point_ pt2;
            pt2.x = xops[1];
            pt2.y = yops[1];
            double d1 = dist(pt1,B);
            double d2 = dist(pt2,B);
            if(d2>d1 && onsegment(A,B,pt1)){
                goal.x = xops[0];
                goal.y = yops[0];
            } else if(onsegment(A,B,pt2)){
                goal.x = xops[1];
                goal.y = yops[1];
            }
          } else {
            goal.x = xops[0];
            goal.y = yops[0];
          }
          double interp = dist(goal,A)/dist(A,B);
          m_speed = (1-interp)*A.z + interp*B.z;
        }
    }
    //calculate needed angular rate from speed and
    double radius = (pow(goal.y,2)+pow(goal.x,2))/(2*goal.x);
    if (fabs(radius) < MINRAD) {
      radius = sgn(radius)*MINRAD;
    }
    // double speed = sqrt(pow(msg.north_velocity,2)+pow(msg.east_velocity,2)); //alternatively use Z command from wpts

    // double xn = m_wpts.front().x;
    // double yn = m_wpts.front().y;
    // double zn = m_wpts.front().z;
    // while (GetDist(x,y,xn,yn) < m_wpRadius)
    // {
    //   // Get the next wp
    //   m_wpts.push_back(m_wpts.front());
    //   m_wpts.pop_front();
    //   xn = m_wpts.front().x;
    //   yn = m_wpts.front().y;
    //   zn = m_wpts.front().z;
    //   ROS_INFO("Hit a waypoint");
    // }
    // // UTM is a ENU system
    // // imugps node uses a local ENU system
    // // so our heading is measured from the positive x axis, meaning 0 is pointing east
    // // and this lets a normal x-y axis system drawn on paper represent our system with x pointing up.
    //
    // double bearing = atan2(xn-x, yn-y);
    // double error = AngleDiff(bearing, theta);

    geometry_msgs::Twist command;

    if (m_speed < 0){
      joy_start_command = false;
    }
    // if(joy_start_command){
      command.linear.x = m_speed;
      // look_dist = m_speed < 10? 10: m_speed;
    // } else {
    //   command.linear.x = 0;
    // }
    double w = m_speed/radius;
    // command.linear.x = m_speed;
    command.angular.z = -w;

    m_twist_pub.publish(command);

    double time = ros::Time::now().toSec();
    if (time > (m_prevTime + 0.3))
    {
      untransform(goal,gc);
      m_prevTime = time;
       std::cout << " Steering " << command.angular.z  << " actual " << m_angular_rate << std::endl;
       std::cout << " Speed " << act_speed ;
       std::cout << " WP Radius " << m_wpRadius << " Lookahead " << look_dist << std::endl;
       std::cout << " goal_x " << goal.x << " goal_y " << goal.y << std::endl;
       std::cout << " gc_x " << gc.x << " gc_y " << gc.y <<" gc_ang " << gc.ang << std::endl;
    }
  }
void PurePursuit::posecb(sensor_msgs::NavSatFix pose)
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
    q.setRPY(0, 0, -(m_yaw)+M_PI_2);
    transform.setRotation(q);

    m_lock.unlock();

    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "waypoint", "golfcart"));
}
//void PurePursuit::paramCallback(const ros::TimerEvent& time)
//{
//  m_nh.param("HeadingP", m_headingP, 2.0);
//}

  double PurePursuit::GetDist(double x1, double y1, double x2, double y2)
  {
    return sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
  }

  double PurePursuit::AngleDiff(double a, double b)
  {
    double dif = fmod(a-b + PI, 2.0*PI);
    if (dif < 0)
      dif += 2.0*PI;
    return dif - PI;
  }

  double PurePursuit::Clamp(double num, double min, double max)
  {
    if (num < min)
      num = min;
    if (num > max)
      num = max;
    return num;
  }

  void PurePursuit::recvJoy(const sensor_msgs::Joy::ConstPtr& msg)
  {
    if(msg->buttons[7]){
      joy_start_command = true;
    }
  }

  void PurePursuit::pubWaypoints(const ros::TimerEvent& event){
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
    BOOST_FOREACH (const geometry_msgs::Point &pt, m_wpts){
      point_ p;
      p.x = pt.x - m_offsetX;
      p.y = pt.y - m_offsetY;
      p.z = 0;
      points.points.push_back(p);
    }
      pub_wpts_.publish(points);
  }
};

int main (int argc, char** argv)
{
  ros::init(argc, argv, "PurePursuit");
  //ros::NodeHandle n;
  wp_control::PurePursuit wpt;
  ros::spin();
}
