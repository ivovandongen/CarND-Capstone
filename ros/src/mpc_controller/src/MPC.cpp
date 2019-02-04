#include "MPC.h"
#include "ros/ros.h"
#include <vector>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "styx_msgs/Lane.h"
#include "tf/transform_datatypes.h"
#include "tf/LinearMath/Matrix3x3.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "mpc_controller/ControlCmd.h"
#include "mpc_controller/ControlCmdArray.h"

constexpr int LOOP_RATE = 20; // should be in sync with dt_
constexpr double MAX_THROTTLE = 0.4;

namespace waypoint_follower {
  
  using CppAD::AD;
  class FG_eval {
    private:  
      double max_accel;
      double max_decel;
      double Lf;
      double max_lat_accel;
      
      int N;
      double dt;
      int y_offset;
      int psi_offset;
      int v_offset;
      int cte_offset;
      int epsi_offset;
      int delta_offset;
      int a_offset;
      // Cost coefficients
      const double cost_weights[3] = {1.0, 300.0, 0.04};
      const double cost_weights_dot[3] = {0.3, 0.3, 50.0};
      const double cost_weights_dotdot[2] = {0.05, 0.05};
    public:
      // Fitted polynomial coefficients
      Eigen::VectorXd coeffs;
      WayPoints* waypoints;

      FG_eval(Eigen::VectorXd coeffs, WayPoints *waypoints, int N, double dt, double max_accel, double max_decel, double wheel_base, double max_lat_accel) { 
        this->coeffs = coeffs;
	this->waypoints = waypoints;
        
        this->N = N;
        this->dt = dt;
        this->Lf = wheel_base / 2;
        this->max_accel = max_accel;
        this->max_decel = max_decel;
	this->max_lat_accel = max_lat_accel;

        y_offset = N;
        psi_offset = N*2;
        v_offset = N*3;
        cte_offset = N*4;
        epsi_offset = N*5;
        delta_offset = N*6;
        a_offset = N*7 - 1;
      }

      typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
      void operator()(ADvector& fg, const ADvector& vars) {
        // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
        
        fg[0] = 0;
	int curWaypoint = 0;
        // estimating cost 
        for (int t = 0; t < N; t++){
          fg[0] += cost_weights[0] * CppAD::pow(vars[cte_offset + t], 2); 
          fg[0] += cost_weights[1] * CppAD::pow(vars[epsi_offset + t], 2);
	  // define max_speed by closest waypoint
	  AD<double> dx = vars[t] - waypoints->getWaypointPosition(curWaypoint).x;
	  AD<double> dy = vars[t + y_offset] - waypoints->getWaypointPosition(curWaypoint).y;
	  AD<double> min_dist = (dx*dx + dy*dy); 
	  for (int i = curWaypoint + 1; i < waypoints->getSize(); i++)
	  {
            dx = vars[t] - waypoints->getWaypointPosition(curWaypoint).x;
	    dy = vars[t + y_offset] - waypoints->getWaypointPosition(curWaypoint).y;
	    AD<double> dist = (dx*dx + dy*dy);
	    if (dist > min_dist)
	    {
              break;
	    }
	    min_dist = dist;
            curWaypoint = i;
	  }
	  AD<double> target_speed = waypoints->getWaypointVelocityMPS(curWaypoint);
          fg[0] += cost_weights[2] * CppAD::pow(vars[v_offset + t] - target_speed, 2);
	  // punish speed violations
	  // if (vars[v_offset + t] > target_speed)
          //  fg[0] += 100;
	  // punish unnecessary stops
	  // if (vars[v_offset + t] < 0.05 && target_speed > 0.05)
	  //   fg[0] += 100;
        }
        // acceleration term (based on actuators)
        for (int t = 0; t < N - 1; t++){
          fg[0] += cost_weights_dot[0] * CppAD::pow(vars[a_offset + t], 2);
	  AD<double> lat_accel = CppAD::pow(vars[v_offset + t], 2) * vars[delta_offset + t] / Lf;
          fg[0] += cost_weights_dot[1] * CppAD::pow(lat_accel, 2); // centripetal 
	  fg[0] += cost_weights_dot[2] * CppAD::pow(vars[delta_offset + t], 2);
	  // if (lat_accel > max_lat_accel)
	  //  fg[0] += 100;
        }
        // jerk term 
        for (int t = 0; t < N - 2; t++){
          fg[0] += cost_weights_dotdot[0] * CppAD::pow((vars[a_offset + t + 1] - vars[a_offset + t])/dt, 2) / dt;
          fg[0] += cost_weights_dotdot[1] * CppAD::pow((vars[delta_offset + t + 1] - vars[delta_offset + t]) / dt, 2);
        }
        fg[1] = vars[0];
        fg[1 + y_offset] = vars[y_offset];
        fg[1 + psi_offset] = vars[psi_offset];
        fg[1 + v_offset] = vars[v_offset];
        fg[1 + cte_offset] = vars[cte_offset];
        fg[1 + epsi_offset] = vars[epsi_offset];
        for (int t = 1; t < N; t++){
          // state at time t-1
          AD<double> x0 = vars[t - 1];
          AD<double> y0 = vars[y_offset + t - 1];
          AD<double> psi0 = vars[psi_offset + t - 1];
          AD<double> v0 = vars[v_offset + t - 1];
          AD<double> cte0 = vars[cte_offset + t - 1];
          AD<double> epsi0 = vars[epsi_offset + t - 1];
          // state at time t
          AD<double> x1 = vars[t];
          AD<double> y1 = vars[y_offset + t];
          AD<double> psi1 = vars[psi_offset + t];
          AD<double> v1 = vars[v_offset + t];
          AD<double> cte1 = vars[cte_offset + t];
          AD<double> epsi1 = vars[epsi_offset + t];
          // actuators state at time t-11, i.e. 500ms delay!
          AD<double> delta = vars[delta_offset + std::max(t - 6, 0)];
          AD<double> a = vars[a_offset + std::max(t - 6, 0)];

          // polyfit predictions for error estimation
          AD<double> f0 = this->coeffs[0] + this->coeffs[1] * x0 + this->coeffs[2] * CppAD::pow(x0,2);
          AD<double> psi_des0 = CppAD::atan(this->coeffs[1] + 2*this->coeffs[2]*x0);
          // Constraints
          fg[1 + t] = x1 - (x0 + v0*CppAD::cos(psi0)*dt);
          fg[1 + y_offset + t] = y1 - (y0 + v0*CppAD::sin(psi0)*dt);
          fg[1 + psi_offset + t] = psi1 - (psi0 + v0*delta/Lf*dt);
          fg[1 + v_offset + t] = v1 - (v0 + a*dt);
          fg[1 + cte_offset + t] = cte1 - ((f0 - y0) - v0*CppAD::sin(epsi0)*dt);
          fg[1 + epsi_offset + t] = epsi1 - ((psi_des0 - psi0) - v0 * delta / Lf * dt);
        }
      }
  };

  // Fit a polynomial.
  // Adapted from https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
  Eigen::VectorXd Polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order) {

    Eigen::MatrixXd A(xvals.size(), order + 1);

    for (int i = 0; i < xvals.size(); i++) {
      A(i, 0) = 1.0;
    }

    for (int j = 0; j < xvals.size(); j++) {
      for (int i = 0; i < order; i++) {
        A(j, i + 1) = A(j, i) * xvals(j);
      }
    }

    auto Q = A.householderQr();
    auto result = Q.solve(yvals);
    return result;
  }
  
  // Evaluate a polynomial.
  double Polyeval(Eigen::VectorXd coeffs, double x) {
    double result = 0.0;
    for (int i = 0; i < coeffs.size(); i++) {
      result += coeffs[i] * pow(x, i);
    }
    return result;
  }

  double IIR(double cur_value, double new_value, double decay)
  {
    return new_value * (1 - decay) + cur_value * decay; 
  }

  int MPC::getNextWaypoint()
  {
    int nextWaypoint = 0;
    double min_dist = getPlaneDistance(current_waypoints_.getWaypointPosition(0), current_pose_.pose.position);
    for (int i = 1; i < current_waypoints_.getSize(); i++)
    {
      double dist = getPlaneDistance(current_waypoints_.getWaypointPosition(i), current_pose_.pose.position);
      if (dist > min_dist)
      {
        return nextWaypoint;  
      }
      min_dist = dist;
      nextWaypoint = i;
    }
    return nextWaypoint;
  }

  int MPC::getReferenceWaypoint(int nextWaypoint)
  {
    int path_size = static_cast<int>(current_waypoints_.getSize());
    double lookahead_distance = std::max(40.0, std::min(speed_iir_ * 4.0, 100.0));
    // if waypoints are not given, do nothing.
    if (path_size == 0)
      return -1;
    
    // look for the reference waypoint.
    for (int i = nextWaypoint; i < path_size; i++)
    {
      if (i == path_size - 1)
      {
        ROS_INFO("search waypoint is the last");
        return i;
      }
      // if there exists an effective waypoint 
      if (getPlaneDistance(current_waypoints_.getWaypointPosition(i), current_pose_.pose.position) > lookahead_distance)
      {
        return i;
      }
    }

    // if this program reaches here , it means we lost the waypoint
    return -1;
  }

  void MPC::callbackFromCurrentPose(const geometry_msgs::PoseStampedConstPtr &msg)
  {
    if (pose_set_)
    {
      double prev_x = current_pose_.pose.position.x;
      double prev_y = current_pose_.pose.position.y;
      double prev_qz = current_pose_.pose.orientation.z;
      double prev_qw = current_pose_.pose.orientation.w;
      current_pose_.header = msg->header;
      current_pose_.pose = msg->pose;
      // translate orientation quaternion to Euler
      double pitch, roll, yaw;
      tf::Quaternion quat;
      tf::quaternionMsgToTF(current_pose_.pose.orientation, quat);
      tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
      yaw_iir_ = IIR(yaw_iir_, yaw, 0.3);

      if (fabs(prev_x - current_pose_.pose.position.x) > 0.01 || fabs(prev_y - current_pose_.pose.position.y) > 0.01 || fabs(prev_qz - current_pose_.pose.orientation.z) >0.01 || fabs(prev_qw - current_pose_.pose.orientation.w) > 0.01)
        updated_ = true;
        // this->Solve();
    }
    else
    {
      current_pose_.header = msg->header;
      current_pose_.pose = msg->pose;
      // translate orientation quaternion to Euler
      double pitch, roll, yaw;
      tf::Quaternion quat;
      tf::quaternionMsgToTF(current_pose_.pose.orientation, quat);
      tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
      yaw_iir_ = yaw;
      pose_set_ = true;
      updated_ = true;
      // this->Solve();
    }
  }

  void MPC::callbackFromCurrentVelocity(const geometry_msgs::TwistStampedConstPtr &msg)
  {
    if (velocity_set_)
    {
      double prev_vx = current_velocity_.twist.linear.x;
      double prev_vy = current_velocity_.twist.linear.y;
      current_velocity_ = *msg;
      double new_vx = current_velocity_.twist.linear.x;
      double new_vy = current_velocity_.twist.linear.y;
      speed_iir_ = IIR(speed_iir_, std::sqrt(new_vx*new_vx+new_vy*new_vy), 0.3);
      if (std::fabs(prev_vx - new_vx) > 0.01 || std::fabs(prev_vy - new_vy) > 0.01)
        updated_ = true;
        // this->Solve();
    }
    else
    {
      current_velocity_ = *msg;
      double new_vx = current_velocity_.twist.linear.x;
      double new_vy = current_velocity_.twist.linear.y;
      speed_iir_ = std::sqrt(new_vx*new_vx + new_vy*new_vy);
      velocity_set_ = true;
      updated_ = true;
      // this->Solve();
    }
  }
  
  void MPC::callbackFromDBWControl(const std_msgs::Bool::Ptr msg)
  {
    bool prev_status = this->dbw_enabled_;
    this->dbw_enabled_ = (*msg).data;
    if (!prev_status && this->dbw_enabled_)
      updated_ = true;
      // this->Solve();
  }

  void MPC::callbackFromWayPoints(const styx_msgs::LaneConstPtr &msg)
  {
    if (waypoint_set_){
      double prev_x = current_waypoints_.getWaypointPosition(0).x;
      double prev_y = current_waypoints_.getWaypointPosition(0).y;
      double prev_v = current_waypoints_.getWaypointVelocityMPS(0);
      current_waypoints_.setPath(*msg);
      if (fabs(prev_x - current_waypoints_.getWaypointPosition(0).x) > 0.01 || fabs(prev_y - current_waypoints_.getWaypointPosition(0).y) > 0.01 
         || fabs(prev_v - current_waypoints_.getWaypointVelocityMPS(0)) > 0.01)
        updated_ = true;
        // this->Solve();
    }
    else 
    {
      current_waypoints_.setPath(*msg);
      waypoint_set_ = true;
      updated_ = true;
      // this->Solve();
    }
  }

  void MPC::Solve() {
    if(!updated_)
    {
      return;
    }
    if(!dbw_enabled_)
    {
      this->steer_ind_ = -1;
      this->accel_ind_ = -1;
      return;
    }
    if(!pose_set_ || !waypoint_set_ || !velocity_set_){
      if(!pose_set_) {
        ROS_WARN("position is missing");
      }
      if(!waypoint_set_) {
        ROS_WARN("waypoint is missing");
      }
      if(!velocity_set_) {
        ROS_WARN("velocity is missing");
      }
      this->steer_ind_ = -1;
      this->accel_ind_ = -1;
      return;
    }

    int next_waypoint = getNextWaypoint();
    int ref_waypoint = getReferenceWaypoint(next_waypoint);

    if (ref_waypoint == -1)
    {
      ROS_WARN("lost next waypoint");
      this->steer_ind_ = -1;
      this->accel_ind_ = -1;
      return;
    }
    ROS_ERROR_STREAM("NEW MPC ITERATION");

    int v_offset = ref_no_*3;
    int delta_offset = ref_no_*6;
    int a_offset = ref_no_*7 - 1;

    // translate waypoints to ego vehicle coordinate system
    Eigen::VectorXd waypoints_x(ref_waypoint - next_waypoint);
    Eigen::VectorXd waypoints_y(ref_waypoint - next_waypoint);
    for (int i = next_waypoint; i < ref_waypoint; i++){
      geometry_msgs::Point p = current_waypoints_.getWaypointPosition(i);
      double dx = p.x - current_pose_.pose.position.x;
      double dy = p.y - current_pose_.pose.position.y;
      waypoints_x[i - next_waypoint] = dx * std::cos(-yaw_iir_) - dy * std::sin(-yaw_iir_);
      waypoints_y[i - next_waypoint] = dx * std::sin(-yaw_iir_) + dy * std::cos(-yaw_iir_);
    }  

    // estimating polynomial fit, ego vehicles' cte, etc.
    Eigen::VectorXd coeffs = Polyfit(waypoints_x, waypoints_y, 2);
    double cte = coeffs[0];
    double epsi = atan(coeffs[1]);
    
    // current state
    Eigen::VectorXd state(6);
    state << 0, 0, 0, speed_iir_, cte, epsi;
    
    bool ok = true;
    int i;
    // Number of model variables (includes both states and inputs).
    int n_vars = (state_no_ + act_no_) * ref_no_ - act_no_; 
    // The number of constraints
    int n_constraints = state_no_ * ref_no_;

    // Initial value of the independent variables.
    // 0 besides initial state.
    typedef CPPAD_TESTVECTOR(double) Dvector;
    Dvector vars(n_vars);
    for (i = 0; i < n_vars; i++) {
      vars[i] = 0;
    }
    for (i = 0; i < state_no_; i++){
      vars[i*ref_no_] = state[i];
    }

    // Lower and upper limits for the variables
    Dvector vars_lowerbound(n_vars);
    Dvector vars_upperbound(n_vars);
    for (i = 0; i < n_vars; i++){
      vars_lowerbound[i] = -1E19;
      vars_upperbound[i] = 1E19;
    }
    // no reverse driving in the capstone project
    for (i = v_offset; i < v_offset + ref_no_; i++)
    {
      vars_lowerbound[i] = 0;
    }

    for (i = delta_offset; i < a_offset; i++){
      vars_lowerbound[i] = -max_steer_angle_;
      vars_upperbound[i] = max_steer_angle_;
    }
    for (i = a_offset; i < n_vars; i++){
      vars_lowerbound[i] = max_decel_;
      vars_upperbound[i] = max_accel_;
    }

    // Lower and upper limits for the constraints
    // 0 besides initial state.
    Dvector constraints_lowerbound(n_constraints);
    Dvector constraints_upperbound(n_constraints);
    for (i = 0; i < n_constraints; i++) {
      constraints_lowerbound[i] = 0;
      constraints_upperbound[i] = 0;
    }
    for (i = 0; i < state_no_; i++){
      constraints_lowerbound[i*ref_no_] = state[i];
      constraints_upperbound[i*ref_no_] = state[i];
    }

    // object that computes objective and constraints
    FG_eval fg_eval(coeffs, &current_waypoints_, ref_no_, dt_, max_decel_, max_accel_, wheel_base_, max_lat_accel_);

    // options for IPOPT solver
    std::string options;
    // Uncomment this if you'd like more print information
    options += "Integer print_level  0\n";
    // NOTE: Setting sparse to true allows the solver to take advantage
    // of sparse routines, this makes the computation MUCH FASTER. If you
    // can uncomment 1 of these and see if it makes a difference or not but
    // if you uncomment both the computation time should go up in orders of
    // magnitude.
    options += "Sparse  true        forward\n";
    options += "Sparse  true        reverse\n";
    // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
    // Change this as you see fit.
    options += "Numeric max_cpu_time          0.10\n";
    options += "Integer max_iter 	3000\n";
    
    // place to return solution
    CppAD::ipopt::solve_result<Dvector> solution;

    // solve the problem
    CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);
    // Check some of tho solution values
    ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;
    this->steer_cmd_.assign(ref_no_ - 1, 0.0);
    this->accel_cmd_.assign(ref_no_ - 1, 0.0);
    for (i = 0; i < ref_no_ - 1; i++)
    {
      this->steer_cmd_[i] = solution.x[i + delta_offset];
      this->accel_cmd_[i] = solution.x[i + a_offset];
      this->steer_ind_ = 0;
      this->accel_ind_ = 0;
    } 
    updated_ = false;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mpc_controller");
  ros::NodeHandle nh;
  // Reading important parameters
  double decel_limit, accel_limit, wheel_base, steer_ratio, max_steer_angle, max_lat_accel;
  nh.param<double>("decel_limit", decel_limit, -5.0);
  nh.param<double>("accel_limit", accel_limit, 1.0);
  nh.param<double>("wheel_base", wheel_base, 2.8498);
  nh.param<double>("steer_ratio", steer_ratio, 14.8);
  nh.param<double>("max_steer_angle", max_steer_angle, 8.);
  nh.param<double>("max_lat_accel", max_lat_accel, 3.);
  waypoint_follower::MPC mpc_controller = waypoint_follower::MPC(accel_limit * MAX_THROTTLE, decel_limit, wheel_base, max_steer_angle/steer_ratio, max_lat_accel);

  ROS_INFO("set publisher...");
  // publish topic
  ros::Publisher steer_angle_pub = nh.advertise<std_msgs::Float64>("/mpc_controller/steering_angle", 1);
  ros::Publisher acceleration_pub = nh.advertise<std_msgs::Float64>("/mpc_controller/acceleration", 1);

  ROS_INFO("set subscriber...");
  // subscribe topic
  ros::Subscriber waypoint_sub =
          nh.subscribe("final_waypoints", 1, &waypoint_follower::MPC::callbackFromWayPoints, &mpc_controller);
  ros::Subscriber pose_sub =
          nh.subscribe("current_pose", 1, &waypoint_follower::MPC::callbackFromCurrentPose, &mpc_controller);
  ros::Subscriber velocity_sub =
          nh.subscribe("current_velocity", 1, &waypoint_follower::MPC::callbackFromCurrentVelocity, &mpc_controller);
  ros::Subscriber dbw_enabled_sub = 
          nh.subscribe("/vehicle/dbw_enabled", 1, &waypoint_follower::MPC::callbackFromDBWControl, &mpc_controller);
  // ros::AsyncSpinner spinner(1);
  // spinner.start();
  
  
  ros::Rate loop_rate(LOOP_RATE);  
  while (ros::ok())
  {
    ros::spinOnce();
    mpc_controller.Solve();
    ROS_ERROR_STREAM("PUBLISHING MPC RESULTS");
    std_msgs::Float64 steer_msg = std_msgs::Float64();
    steer_msg.data = mpc_controller.GetSteerCmd();
    steer_angle_pub.publish(steer_msg);

    std_msgs::Float64 accel_msg = std_msgs::Float64();
    accel_msg.data = mpc_controller.GetAccelCmd();
    acceleration_pub.publish(accel_msg);
    loop_rate.sleep();
  }
  // spinner.stop();
	
  return 0;
}
