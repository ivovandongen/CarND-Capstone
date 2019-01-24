#include "MPC.h"
#include "dbw_mkz_msgs/ThrottleCmd.h"
#include "dbw_mkz_msgs/SteeringCmd.h"
#include "dbw_mkz_msgs/BrakeCmd.h"
#include "std_msgs/Bool.h"
using namespace std;

constexpr int LOOP_RATE = 30; //processing frequency, was: 30
constexpr double MAX_THROTTLE = 0.4;

class DBWNode
{
  private: 
    ros::NodeHandle nh;
  
    ros::Subscriber waypoint_sub;
    ros::Subscriber pose_sub;
    ros::Subscriber velocity_sub;
    ros::Subscriber dbw_enabled_sub;
      
    ros::Publisher steer_pub;
    ros::Publisher throttle_pub;
    ros::Publisher brake_pub;
    
    waypoint_follower::MPC mpc_controller;
    bool dbw_enabled;

    double vehicle_mass; 
    double fuel_capacity;
    double brake_deadband; 
    double decel_limit; 
    double accel_limit; 
    double wheel_radius;
    double wheel_base; 
    double steer_ratio;
    double max_lat_accel; 
    double max_steer_angle;
  
  public:


    DBWNode()
    {
      this->dbw_enabled = false;
      // reading parameters
      nh.param<double>("vehicle_mass", this->vehicle_mass, 1736.35);
      nh.param<double>("fuel_capacity", this->fuel_capacity,  13.5);
      nh.param<double>("brake_deadband", this->brake_deadband,  .1);
      nh.param<double>("decel_limit", this->decel_limit, -5.0);
      nh.param<double>("accel_limit", this->accel_limit, 1.0);
      nh.param<double>("wheel_radius", this->wheel_radius, 0.2413);
      nh.param<double>("wheel_base", this->wheel_base, 2.8498);
      nh.param<double>("steer_ratio", this->steer_ratio, 14.8);
      nh.param<double>("max_lat_accel", this->max_lat_accel, 3.0);
      nh.param<double>("max_steer_angle", this->max_steer_angle, 8.);
      double max_steer = this->max_steer_angle / this->steer_ratio;
      mpc_controller = waypoint_follower::MPC(this->accel_limit*MAX_THROTTLE, this->decel_limit, this->wheel_base, max_steer);
      ROS_INFO("set publisher...");
      // publish topic
      steer_pub = nh.advertise<dbw_mkz_msgs::SteeringCmd>("/vehicle/steering_cmd", 1);
      throttle_pub = nh.advertise<dbw_mkz_msgs::ThrottleCmd>("/vehicle/throttle_cmd", 1);
      brake_pub = nh.advertise<dbw_mkz_msgs::BrakeCmd>("/vehicle/brake_cmd", 1);
      
      ROS_INFO("set subscriber...");
      // subscribe topic
      waypoint_sub =
          nh.subscribe("final_waypoints", 1, &waypoint_follower::MPC::callbackFromWayPoints, &this->mpc_controller);
      pose_sub =
          nh.subscribe("current_pose", 1, &waypoint_follower::MPC::callbackFromCurrentPose, &this->mpc_controller);
      velocity_sub =
          nh.subscribe("current_velocity", 1, &waypoint_follower::MPC::callbackFromCurrentVelocity, &this->mpc_controller);
      dbw_enabled_sub = 
          nh.subscribe("/vehicle/dbw_enabled", 1, &DBWNode::callbackFromDBWControl, this);
       
      ROS_INFO("mpc controller start");
    }
    
    void callbackFromDBWControl(std_msgs::Bool::Ptr msg)
    {
      this->dbw_enabled = (*msg).data;
    }
                       
    void publish_cmds()
    {
      double throttle = 0.0;
      double accel = 0.0;
      double steer = 0.0;
      double brake = 0.0;
      if (dbw_enabled){
      	vector<double> mpc_result = mpc_controller.Solve();
      	steer = mpc_result[0] * this->steer_ratio;
      	accel = mpc_result[1];
      	brake = 0;
              
        if (fabs(accel) <= 0.1)
        {
          throttle = .0;
          brake = 400;
        }
        else if (accel < .0)
        {
          throttle = .0;
          brake = abs(accel) * this->vehicle_mass * this->wheel_radius;
        }
        else 
        {
          throttle = accel / this->accel_limit;
        }
      }
      // ROS_ERROR_STREAM("Throttle " << throttle << "; steering " << steer << "; brake " << brake);
      dbw_mkz_msgs::ThrottleCmd tcmd = dbw_mkz_msgs::ThrottleCmd();
      tcmd.enable = true;
      tcmd.pedal_cmd_type = dbw_mkz_msgs::ThrottleCmd::CMD_PERCENT;
      tcmd.pedal_cmd = throttle;
      this->throttle_pub.publish(tcmd);

      dbw_mkz_msgs::SteeringCmd scmd = dbw_mkz_msgs::SteeringCmd();
      scmd.enable = true;
      scmd.steering_wheel_angle_cmd = steer;
      this->steer_pub.publish(scmd);

      dbw_mkz_msgs::BrakeCmd bcmd = dbw_mkz_msgs::BrakeCmd();
      bcmd.enable = true;
      bcmd.pedal_cmd_type = dbw_mkz_msgs::BrakeCmd::CMD_TORQUE;
      bcmd.pedal_cmd = brake;
      this->brake_pub.publish(bcmd);

      // dbw_enabled = false;
    }
};


int main(int argc, char **argv)
{
  // set up ros
  ros::init(argc, argv, "dbw_node");

  DBWNode dbw_node = DBWNode();

  ros::Rate loop_rate(LOOP_RATE);
  while (ros::ok())
  {
    ros::spinOnce();
    dbw_node.publish_cmds();
    loop_rate.sleep();
  }
  return 0;
}

