#ifndef MPC_H
#define MPC_H

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
    
    double accel_;
    double steer_;
    double max_accel_;
    double max_decel_;
    double wheel_base_;
    double max_steer_angle_;

    int ref_no_; // number of reference waypoints 
    double dt_; // time interval between waypoints

    int state_no_; // state elements number 
    int act_no_; // actuator elements number
  
    bool waypoint_set_;
    bool pose_set_;
    bool velocity_set_;
    bool dbw_enabled_;
    
    // obtained from messages
    geometry_msgs::PoseStamped current_pose_;
    geometry_msgs::TwistStamped current_velocity_;
    WayPoints current_waypoints_;
  
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
    {

      this->steer_ = 0.0;
      this->accel_ = 0.0;
      this->max_accel_ = 1.0;
      this->max_decel_ = -5.0;
      this->wheel_base_ = 3.0;
      this->max_steer_angle_ = 0.54;
    }
  
    MPC(double max_accel, double max_decel, double wheel_base, double max_steer_angle)
    : ref_no_(10)
    , dt_(0.10)
    , state_no_(6)
    , act_no_(2)
    , waypoint_set_(false)
    , pose_set_(false) 
    , velocity_set_(false) 
    , dbw_enabled_(false)
    {
      this->steer_ = 0.0;
      this->accel_ = 0.0;
      this->max_accel_ = max_accel;
      this->max_decel_ = max_decel;
      this->wheel_base_ = wheel_base;
      this->max_steer_angle_ = max_steer_angle;
    }

    ~MPC()
    {
    }
    // reading solution
    double Steer(){
      return this->steer_;
    }
    double Accel(){
      return this->accel_;
    }

    // helper functions
    int getReferenceWaypoint();
  
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
