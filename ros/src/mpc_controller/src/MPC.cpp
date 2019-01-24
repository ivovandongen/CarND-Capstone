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

namespace waypoint_follower {
  
  using CppAD::AD;
  class FG_eval {
    private:  
      double max_accel;
      double max_decel;
      double Lf;
      double max_speed;
      
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
      // const double cost_weights[6] = {1.7, 200, 0.01, 0.001, 0.005, 0.0008};
      const double cost_weights[6] = {1, 150, 0.004, 0.001, 0.005, 0.000};
    public:
      // Fitted polynomial coefficients 
      Eigen::VectorXd coeffs;
      Eigen::VectorXd coeffs_vel;
      FG_eval(Eigen::VectorXd coeffs, int N, double dt, double max_accel, double max_decel, double wheel_base, double max_speed) { 
        this->coeffs = coeffs;
        
        this->N = N;
        this->dt = dt;
        this->Lf = wheel_base / 2;
        this->max_accel = max_accel;
        this->max_decel = max_decel;
        this->max_speed = max_speed;
        
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
        // estimating cost 
        for (int t = 0; t < N; t++){
          fg[0] += cost_weights[0] * CppAD::pow(vars[cte_offset + t], 2); 
          fg[0] += cost_weights[1] * CppAD::pow(vars[epsi_offset + t], 2);
          fg[0] += cost_weights[2] * CppAD::pow(vars[v_offset + t] - max_speed, 2);
        }
        // acceleration term (based on actuators)
        for (int t = 0; t < N - 1; t++){
          fg[0] += cost_weights[3] * CppAD::pow(vars[a_offset + t], 2);
          fg[0] += cost_weights[4] * CppAD::pow(CppAD::pow(vars[v_offset + t], 2) * vars[delta_offset + t] / Lf, 2); // centripetal 
        }
        // jerk term
        for (int t = 0; t < N - 2; t++){
          AD<double> tang_jerk = (vars[a_offset + t + 1] - vars[a_offset + t]) / dt;
          AD<double> cp_jerk = (vars[delta_offset + t + 1] - vars[delta_offset + t]) * CppAD::pow(vars[v_offset + t],2) / Lf / dt;
          fg[0] += cost_weights[5] * (CppAD::pow(tang_jerk, 2) + CppAD::pow(cp_jerk, 2));
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
          // actuators state at time 
          AD<double> delta = vars[delta_offset + t - 1];
          AD<double> a = vars[a_offset + t - 1];
	  if (t > 1)
	  {
	    delta = vars[delta_offset + t - 2];
	    a = vars[a_offset + t - 2];
	  }

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

  int MPC::getReferenceWaypoint()
  {
    int path_size = static_cast<int>(current_waypoints_.getSize());
    double v = std::sqrt(current_velocity_.twist.linear.x * current_velocity_.twist.linear.x + current_velocity_.twist.linear.y * current_velocity_.twist.linear.y);
    double lookahead_distance = std::max(40.0, std::min(v * 4.0, 100.0));
    // if waypoints are not given, do nothing.
    if (path_size == 0)
      return -1;
    
    // look for the next waypoint.
    for (int i = 0; i < path_size; i++)
    {
      // if there exists an effective waypoint 
      if (getPlaneDistance(current_waypoints_.getWaypointPosition(i), current_pose_.pose.position) > lookahead_distance &&  current_waypoints_.isFront(i, current_pose_.pose))
      {
        //ROS_ERROR_STREAM("wp = " << i << " dist = " << getPlaneDistance(current_waypoints_.getWaypointPosition(i), current_pose_.pose.position) );
        return i;
      }
    }
    // if search waypoint is the last
    if (current_waypoints_.isFront(path_size - 1, current_pose_.pose))
    {
      ROS_INFO("search waypoint is the last");
      return path_size - 1;
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
      current_pose_.header = msg->header;
      current_pose_.pose = msg->pose;
      if (fabs(prev_x - current_pose_.pose.position.x) > 0.1 || fabs(prev_y - current_pose_.pose.position.y) > 0.1)
        updated_ = true;
    }
    else
    {
      current_pose_.header = msg->header;
      current_pose_.pose = msg->pose;
      updated_ = true;
    }
    pose_set_ = true; 
  }

  void MPC::callbackFromCurrentVelocity(const geometry_msgs::TwistStampedConstPtr &msg)
  {
    if (velocity_set_)
    {
      double prev_vx = current_velocity_.twist.linear.x;
      double prev_vy = current_velocity_.twist.linear.y;
      current_velocity_ = *msg;
      if (std::fabs(prev_vx - current_velocity_.twist.linear.x) > 0.1 || std::fabs(prev_vy - current_velocity_.twist.linear.y) > 0.1)
        updated_ = true;
    }
    else
    {
      current_velocity_ = *msg;
      updated_ = true;
    }

    velocity_set_ = true;
  }

  void MPC::callbackFromWayPoints(const styx_msgs::LaneConstPtr &msg)
  {
    if (waypoint_set_){
      double prev_x = current_waypoints_.getWaypointPosition(0).x;
      double prev_y = current_waypoints_.getWaypointPosition(0).y;
      double prev_v = current_waypoints_.getWaypointVelocityMPS(0);
      current_waypoints_.setPath(*msg);
      if (fabs(prev_x - current_waypoints_.getWaypointPosition(0).x) > 0.1 || fabs(prev_y - current_waypoints_.getWaypointPosition(0).y) > 0.1 
         || fabs(prev_v - current_waypoints_.getWaypointVelocityMPS(0)) > 0.1)
        updated_ = true;
    }
    else 
    {
      current_waypoints_.setPath(*msg);
      updated_ = true;
    }
    
    waypoint_set_ = true;
    // ROS_INFO_STREAM("waypoint subscribed");
  }

  std::vector<double> MPC::Solve() {
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
      return {0, 0};
    }
    int ref_waypoint = getReferenceWaypoint();
    if (ref_waypoint == -1)
    {
      ROS_WARN("lost next waypoint");
      return {0, 0};
    }
    if (!updated_) // only if neither of pose, waypoints nor velocity changed / updated
    {
      return {steer_, accel_};
    }
    int y_offset = ref_no_;
    int v_offset = ref_no_*3;
    int delta_offset = ref_no_*6;
    int a_offset = ref_no_*7 - 1;

    // translate orientation quaternion to Euler
    double pitch, roll, yaw;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(current_pose_.pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    // translate waypoints to ego vehicle coordinate system
    Eigen::VectorXd waypoints_x(ref_waypoint);
    Eigen::VectorXd waypoints_y(ref_waypoint);
    for (int i = 0; i < ref_waypoint; i++){
      geometry_msgs::Point p = current_waypoints_.getWaypointPosition(i);
      double dx = p.x - current_pose_.pose.position.x;
      double dy = p.y - current_pose_.pose.position.y;
      waypoints_x[i] = dx * std::cos(-yaw) - dy * std::sin(-yaw);
      waypoints_y[i] = dx * std::sin(-yaw) + dy * std::cos(-yaw);
    }  

    // estimating polynomial fit, ego vehicles' cte, etc.
    Eigen::VectorXd coeffs = Polyfit(waypoints_x, waypoints_y, 2);
    double cte = Polyeval(coeffs, 0);
    double epsi = atan(coeffs[1]);
    double max_speed = current_waypoints_.getWaypointVelocityMPS(0);
    
    // current state
    double v = std::sqrt(current_velocity_.twist.linear.x * current_velocity_.twist.linear.x + current_velocity_.twist.linear.y * current_velocity_.twist.linear.y);
    Eigen::VectorXd state(6);
    state << 0, 0, 0, v, cte, epsi;
    
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
    FG_eval fg_eval(coeffs, ref_no_, dt_, max_decel_, max_accel_, wheel_base_, max_speed);

    // options for IPOPT solver
    std::string options;
    // Uncomment this if you'd like more print information
    options += "Integer print_level  0\n";
    // NOTE: Setting sparse to true allows the solver to take advantage
    // of sparse routines, this makes the computation MUCH FASTER. If you
    // can uncomment 1 of these and see if it makes a difference or not but
    // if you uncomment both the computation time should go up in orders of
    // magnitude.
    // options += "Sparse  true        forward\n";
    // options += "Sparse  true        reverse\n";
    // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
    // Change this as you see fit.
    options += "Numeric max_cpu_time          0.5\n";
    options += "Integer max_iter 	3000\n";
    
    // place to return solution
    CppAD::ipopt::solve_result<Dvector> solution;

    // solve the problem
    CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);
    // Check some of the solution values
    ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

    accel_ = solution.x[a_offset];
    steer_ = solution.x[delta_offset];
    updated_ = false;
    return {steer_, accel_};
}
}
