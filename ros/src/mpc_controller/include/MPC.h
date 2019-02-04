hifndef MPC_H
#define MPC_H

#include "ros/ros.h"
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include "styx_msgs/Lane.h"
#include "libwaypoint_follower.h"
#include "std_msgs/Bool.h"

namespace waypoint_follower{
class MPC {
  private:
    
    // double accel_;
    // double steer_;
    double max_accel_;
    double max_decel_;
    double wheel_base_;
    double max_steer_angle_;
    double max_lat_accel_;
    int ref_no_; // number of reference waypoints 
    double dt_; // time interval between waypoints
    int state_no_; // state elements number 
    int act_no_; // actuator elements number
    bool waypoint_set_;
    bool pose_set_;
    bool velocity_set_;
    bool dbw_enabled_;
    bool updated_;

    // obtained from messages
    geometry_msgs::PoseStamped current_pose_;
    geometry_msgs::TwistStamped current_velocity_;
    WayPoints current_waypoints_;
    double speed_iir_;
    double yaw_iir_;
  
    std::vector<double> steer_cmd_;
    std::vector<double> accel_cmd_;;
    int steer_ind_;
    int accel_ind_;
    
  public:
      MPC()
    : ref_no_(10)
    , dt_(0.10)
    , state_no_(6)
    , act_no_(2)
    , waypoint_set_(false)
    , pose_set_(false)
    , velocity_set_(false) 
    , dbw_enabled_(false)
    , updated_(false)
    {
      this->max_accel_ = 1.0;
      this->max_decel_ = -5.0;
      this->wheel_base_ = 3.0;
      this->max_steer_angle_ = 0.54;
      this->max_lat_accel_ = 3.0;
      this->steer_ind_ = -1; 
      this->accel_ind_ = -1;
      // -1 refers to NO control
    }
  
    MPC(double max_accel, double max_decel, double wheel_base, double max_steer_angle, double max_lat_accel)
    : ref_no_(10)
    , dt_(0.10)
    , state_no_(6)
    , act_no_(2)
    , waypoint_set_(false)
    , pose_set_(false) 
    , velocity_set_(false) 
    , dbw_enabled_(false)
    , updated_(false)
    {
      this->max_accel_ = max_accel;
      this->max_decel_ = max_decel;
      this->wheel_base_ = wheel_base;
      this->max_steer_angle_ = max_steer_angle;
      this->max_lat_accel_ = max_lat_accel;
      this->steer_ind_ = -1; 
      this->accel_ind_ = -1;
      // -1 refers to NO control
    }

    ~MPC()
    {
    }

    // reading solution
    double GetSteerCmd(){
      if (this->steer_ind_ < 0)
        return 0.0;
      return this->steer_cmd_[0];
      /*
      if (this->steer_ind_ >= this->steer_cmd_.size())
      {
        ROS_WARN("Steering index is out of bound of steering command vector");
	return 0.0;
      }
      return this->steer_cmd_[steer_ind_++];
      */
    }

    double GetAccelCmd(){
      if (this->accel_ind_ < 0)
        return 0.0;
      return this->accel_cmd_[0];
      /*
      if (this->accel_ind_ >= this->accel_cmd_.size())
      {
        ROS_WARN("Acceleration index is out of bound of acceleration command vector");
	return 0.0;
      }
      return this->accel_cmd_[accel_ind_++];
      */
    }
     

    // helper functions
    int getNextWaypoint();
    int getReferenceWaypoint(int nextWaypoint);
  
    // callbacks
    void callbackFromCurrentPose(const geometry_msgs::PoseStampedConstPtr &msg);
    void callbackFromCurrentVelocity(const geometry_msgs::TwistStampedConstPtr &msg);
    void callbackFromWayPoints(const styx_msgs::LaneConstPtr &msg);
    void callbackFromDBWControl(const std_msgs::Bool::Ptr msg);

    // Solve the model given an initial state and polynomial coefficients.
    void Solve();
};
}
#endif /* MPC_H */
