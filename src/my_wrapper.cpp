#include <teb_local_planner/my_wrapper.h>
// MBF return codes
#include <mbf_msgs/ExePathResult.h>

#include <teb_local_planner/g2o_types/vertex_pose.h>

#include <teb_local_planner/pose_se2.h>

#include <geometry_msgs/PoseStamped.h>

//#include <teb_local_planner/timed_elastic_band.h>


namespace teb_local_planner
{
  
MyWrapper::MyWrapper() : costmap_(), initialized_(false)
{
}


MyWrapper::~MyWrapper()
{
}

void MyWrapper::initialize(const LocalCostmapROS &local_costmap, const geometry_msgs::PoseStamped &odom_pose, const geometry_msgs::TwistStamped& odom_velocity,
  const geometry_msgs::PoseStamped &amcl_pose, const geometry_msgs::TransformStamped &tf_odom_map_v, const geometry_msgs::TransformStamped &tf_map_odom_v,
  const std::vector<geometry_msgs::Point>& robotFootprint)
{
	// create robot footprint/contour model for optimization
	RobotFootprintModelPtr robot_model = getRobotFootprintFromParamServer();
	
	// local costmap
	costmap_ = local_costmap;

    // local costmap model
	costmap_model_ = boost::make_shared<base_local_planner::CostmapModel>(costmap_);	
	
	// reserve some memory for obstacles
	obstacles_.reserve(500);

    // robot's odometry (local) and map (global) location
	amcl_pose_ = amcl_pose;
	odom_pose_ = odom_pose;

	// robot's odometry (local) velocity
	odom_velocity_ = odom_velocity;

	// transformations between /odom and /map frame
	tf_odom_map_ = tf_odom_map_v;
	tf_map_odom_ = tf_map_odom_v;

    // robot footprint
	footprint_spec_ = robotFootprint;
			
	// create the planner instance - here only TebOptimalPlanner is considered and used
	planner_ = PlannerInterfacePtr(new TebOptimalPlanner(config_, &obstacles_, robot_model));
	//ROS_INFO("Parallel planning in distinctive topologies disabled.\n");

	//ROS_INFO("No costmap conversion plugin specified. All occupied costmap cells are treaten as point obstacles.\n");

	initialized_ = true;		
	//ROS_DEBUG("teb_local_planner initialized.\n");
}


bool MyWrapper::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
{
  // check if plugin is initialized
  if(!initialized_)
  {
    ROS_ERROR("teb_local_planner has not been initialized, please call initialize() before using this planner\n");
    return false;
  }

  // store the global plan
  global_plan_.clear();
  global_plan_ = orig_global_plan;

  // reset goal_reached_ flag
  goal_reached_ = false;
  
  return true;
}



uint32_t MyWrapper::computeVelocityCommandsImage(geometry_msgs::TwistStamped &cmd_vel, std::string &message,
std::vector<geometry_msgs::PoseStamped>& local_plan, std::vector<geometry_msgs::PoseStamped>& transformed_plan)
{
  // check if local_planner initialized
  if(!initialized_)
  {
    ROS_ERROR("teb_local_planner has not been initialized, please call initialize() before using this planner\n");
    message = "teb_local_planner has not been initialized";
    return mbf_msgs::ExePathResult::NOT_INITIALIZED;
  }
  
  // command velocity - output - set all to zero
  cmd_vel.twist.linear.x = cmd_vel.twist.linear.y = cmd_vel.twist.angular.z = 0;
  
  // goal_reached
  goal_reached_ = false;
   
  // prune global plan 
  bool prune = pruneGlobalPlan(global_plan_, config_.trajectory.global_plan_prune_distance);
  //std::cout << "prune: " << prune << std::endl;

  // Transform global plan to the frame of interest (w.r.t. the local costmap) - from "map" frame to "odom" frame
  int goal_idx; // index of the goal
  if (!transformGlobalPlan(global_plan_, costmap_, config_.trajectory.max_global_plan_lookahead_dist, transformed_plan, &goal_idx))
  {
    ROS_WARN("Could not transform the global plan to the frame of the controller\n");
    message = "Could not transform the global plan to the frame of the controller";
    return mbf_msgs::ExePathResult::INTERNAL_ERROR;
  }
  //std::cout << "goal_idx: " << goal_idx << std::endl;

  // Get current goal point (last point of the transformed plan) in the "odom" frame
  PoseSE2 odom_robot_goal;
  odom_robot_goal.x() = transformed_plan.back().pose.position.x;
  odom_robot_goal.y() = transformed_plan.back().pose.position.y;

  // Overwrite goal orientation if needed
  if (config_.trajectory.global_plan_overwrite_orientation)
  {
    odom_robot_goal.theta() = estimateLocalGoalOrientation(global_plan_, transformed_plan.back(), goal_idx, tf_map_odom_);
    // overwrite/update goal orientation of the transformed plan with the actual goal (enable using the plan as initialization)
    tf2::Quaternion q;
    q.setRPY(0, 0, odom_robot_goal.theta());
    tf2::convert(q, transformed_plan.back().pose.orientation);
  }  
  else
  {
    odom_robot_goal.theta() = tf2::getYaw(transformed_plan.back().pose.orientation);
  }

  // this could be interesting to change
  // overwrite/update start of the transformed plan with the actual robot position (allows using the plan as initial trajectory)
  if (transformed_plan.size()==1) // plan only contains the goal
  {
    transformed_plan.insert(transformed_plan.begin(), geometry_msgs::PoseStamped()); // insert start (not yet initialized)
  }
  transformed_plan.front() = odom_pose_; // update start

  // clear currently existing obstacles
  obstacles_.clear();  

  // update obstacles with costmap
  updateObstacleContainerWithCostmap(odom_pose_);

  // print how many points obstacles there are in the local costmap
  //std::cout << "obstacles_.size() = " << obstacles_.size() << std::endl;

  // structure that will be sent to the planning function - it requires Twist message
  geometry_msgs::Twist odom_robot_vel;
  odom_robot_vel.linear = odom_velocity_.twist.linear;
  odom_robot_vel.angular = odom_velocity_.twist.angular;
  
  // call the planning function
  bool success = planner_->plan(transformed_plan, &odom_robot_vel, config_.goal_tolerance.free_goal_vel);
  //std::cout << "success: " << success << std::endl;
  
  // if planning not successful, call clearPlanner and return NO_VALID_CMD
  if (!success)
  {
    ROS_WARN("teb_local_planner was not able to obtain a local plan for the current setting.\n");
    return mbf_msgs::ExePathResult::NO_VALID_CMD;
  }
  
  // Check for divergence
  if (planner_->hasDiverged())
  {
    cmd_vel.twist.linear.x = cmd_vel.twist.linear.y = cmd_vel.twist.angular.z = 0;

    // Reset everything to start again with the initialization of new trajectories.
    ROS_WARN_THROTTLE(1.0, "TebLocalPlannerROS: the trajectory has diverged. Resetting planner...");

	message = "teb_local_planner was not able to obtain a local plan";
    return mbf_msgs::ExePathResult::NO_VALID_CMD;
  }

  // Check feasibility (but within the first few states only)
  if(config_.robot.is_footprint_dynamic)
  {
	// Update footprint of the robot and minimum and maximum distance from the center of the robot to its footprint vertices.
    costmap_2d::calculateMinAndMaxDistances(footprint_spec_, robot_inscribed_radius_, robot_circumscribed_radius_);
  }

  bool feasible = planner_->isTrajectoryFeasibleImage(costmap_, footprint_spec_, robot_inscribed_radius_, robot_circumscribed_radius_, config_.trajectory.feasibility_check_no_poses);
  if (!feasible)
  {	  
    cmd_vel.twist.linear.x = cmd_vel.twist.linear.y = cmd_vel.twist.angular.z = 0;

    // now we reset everything to start again with the initialization of new trajectories.
    ROS_WARN("TebLocalPlannerROS: trajectory is not feasible. Resetting planner...");
    
    message = "teb_local_planner trajectory is not feasible";
    return mbf_msgs::ExePathResult::NO_VALID_CMD;
  }

  // Get the velocity command for this sampling interval
  if (!planner_->getVelocityCommand(cmd_vel.twist.linear.x, cmd_vel.twist.linear.y, cmd_vel.twist.angular.z, config_.trajectory.control_look_ahead_poses))
  {
    ROS_WARN("TebLocalPlannerROS: velocity command invalid. Resetting planner...\n");
    return mbf_msgs::ExePathResult::NO_VALID_CMD;
  }
  
  // Saturate velocity, if the optimization results violates the constraints (could be possible due to soft constraints).
  saturateVelocity(cmd_vel.twist.linear.x, cmd_vel.twist.linear.y, cmd_vel.twist.angular.z,
                   config_.robot.max_vel_x, config_.robot.max_vel_y, config_.robot.max_vel_theta);

  const TimedElasticBand& teb_ =  planner_->getTeb();
  std::vector<VertexPose*> plan_vec = teb_.poses();
  
  for(int i = 0; i < plan_vec.size(); i++)
  {
	  PoseSE2 pose_tmp = plan_vec[i]->pose();
	  geometry_msgs::PoseStamped poseStamped;
	  poseStamped.pose.position.x = pose_tmp.x();
	  poseStamped.pose.position.y = pose_tmp.y();
	  Eigen::Vector2d orient_vec = pose_tmp.orientationUnitVec();
	  //poseStamped.pose.orientation = pose_tmp.orientationUnitVec();
	  //std::cout << "Theta: " << pose_tmp.theta() << std::endl;
	  local_plan.push_back(poseStamped);
  }

  return mbf_msgs::ExePathResult::SUCCESS;
}


double MyWrapper::estimateLocalGoalOrientation(const std::vector<geometry_msgs::PoseStamped>& global_plan, const geometry_msgs::PoseStamped& local_goal,
              int current_goal_idx, const geometry_msgs::TransformStamped& tf_plan_to_global, int moving_average_length) const
{
  int n = (int)global_plan.size();
  
  // check if we are near the global goal already
  if (current_goal_idx > n-moving_average_length-2)
  {
    if (current_goal_idx >= n-1) // we've exactly reached the goal
    {
      return tf2::getYaw(local_goal.pose.orientation);
    }
    else
    {
      tf2::Quaternion global_orientation;
      tf2::convert(global_plan.back().pose.orientation, global_orientation);
      tf2::Quaternion rotation;
      tf2::convert(tf_plan_to_global.transform.rotation, rotation);
      // TODO(roesmann): avoid conversion to tf2::Quaternion
      return tf2::getYaw(rotation *  global_orientation);
    }     
  }
  
  // reduce number of poses taken into account if the desired number of poses is not available
  moving_average_length = std::min(moving_average_length, n-current_goal_idx-1 ); // maybe redundant, since we have checked the vicinity of the goal before
  
  std::vector<double> candidates;
  geometry_msgs::PoseStamped tf_pose_k = local_goal;
  geometry_msgs::PoseStamped tf_pose_kp1;
  
  int range_end = current_goal_idx + moving_average_length;
  for (int i = current_goal_idx; i < range_end; ++i)
  {
    // Transform pose of the global plan to the planning frame
    tf2::doTransform(global_plan.at(i+1), tf_pose_kp1, tf_plan_to_global);

    // calculate yaw angle  
    candidates.push_back( std::atan2(tf_pose_kp1.pose.position.y - tf_pose_k.pose.position.y,
        tf_pose_kp1.pose.position.x - tf_pose_k.pose.position.x ) );
    
    if (i<range_end-1) 
      tf_pose_k = tf_pose_kp1;
  }
  return average_angles(candidates);
}


RobotFootprintModelPtr MyWrapper::getRobotFootprintFromParamServer()
{
	// hardcoding - circular model
	double radius = 0.275;
    return boost::make_shared<CircularRobotFootprint>(radius);
}


void MyWrapper::updateObstacleContainerWithCostmap(const geometry_msgs::PoseStamped& pose)
{  
  // Add costmap obstacles if desired
  if (config_.obstacles.include_costmap_obstacles)
  {	  
    Eigen::Vector2d robot_orient(Eigen::Vector2d(std::cos(robot_orientation_yaw), std::sin(robot_orientation_yaw))); 
	Eigen::Vector2d robot_pose(pose.pose.position.x, pose.pose.position.y);
	//Eigen::Vector2d robot_pose(amcl_pose_.position.x, amcl_pose_.position.y);
	
	//std::cout << costmap_.getSizeInCellsX() << std::endl;
	//std::cout << costmap_.getSizeInCellsY() << std::endl;
    
    for (unsigned int i=0; i<costmap_.getSizeInCellsX()-1; ++i)
    {
      for (unsigned int j=0; j<costmap_.getSizeInCellsY()-1; ++j)
      {
		if (costmap_.getCost(i,j) == 99) //costmap_2d::LETHAL_OBSTACLE
        {
          Eigen::Vector2d obs;
          costmap_.mapToWorld(i,j,obs.coeffRef(0), obs.coeffRef(1));
		  
          // check if obstacle is interesting (e.g. not far behind the robot)
          Eigen::Vector2d obs_dir = obs-robot_pose;
		  		  		  
		  //std::cout << "obs: " << obs << std::endl;
		  //std::cout << "robot_orient: " << robot_orient << std::endl;
		  //std::cout << "robot_pose: " << robot_pose << std::endl;
		  //std::cout << "obs_dir.norm(): " << obs_dir.norm() << "  ";
		  //std::cout << "config: " << config_.obstacles.costmap_obstacles_behind_robot_dist << std::endl;

          if ( obs_dir.dot(robot_orient) < 0 && obs_dir.norm() > config_.obstacles.costmap_obstacles_behind_robot_dist  )
            continue;
            
          obstacles_.push_back(ObstaclePtr(new PointObstacle(obs)));
        }
      }
    }
  }
}

void MyWrapper::saturateVelocity(double& vx, double& vy, double& omega, double max_vel_x, double max_vel_y, double max_vel_theta) const
{
  double ratio_x = 1, ratio_omega = 1, ratio_y = 1;
  
  // Limit translational velocity for forward driving
  if (vx > max_vel_x)
    ratio_x = max_vel_x / vx;
  
  // limit strafing velocity
  if (vy > max_vel_y || vy < -max_vel_y)
    ratio_y = std::abs(vy / max_vel_y);
  
  // Limit angular velocity
  if (omega > max_vel_theta || omega < -max_vel_theta)
    ratio_omega = std::abs(max_vel_theta / omega);

  if (config_.robot.use_proportional_saturation)
  {
    double ratio = std::min(std::min(ratio_x, ratio_y), ratio_omega);
    vx *= ratio;
    vy *= ratio;
    omega *= ratio;
  }
  else
  {
    vx *= ratio_x;
    vy *= ratio_y;
    omega *= ratio_omega;
  }
}


bool MyWrapper::pruneGlobalPlan(std::vector<geometry_msgs::PoseStamped>& global_plan, double dist_behind_robot)
{
	if (global_plan.empty())
		return true;
   
	double dist_thresh_sq = dist_behind_robot*dist_behind_robot;

	// iterate plan until a pose close the robot is found
	std::vector<geometry_msgs::PoseStamped>::iterator it = global_plan.begin();
	std::vector<geometry_msgs::PoseStamped>::iterator erase_end = it;
	while (it != global_plan.end())
	{
	  double dx = amcl_pose_.pose.position.x - it->pose.position.x;
	  double dy = amcl_pose_.pose.position.y - it->pose.position.y;
	  double dist_sq = dx * dx + dy * dy;
	  if (dist_sq < dist_thresh_sq)
	  {
		 //std::cout << "usao" << std::endl;
		 erase_end = it;
		 break;
	  }
	  ++it;
	}
	
	if (erase_end == global_plan.end())
	{
	  //std::cout << "usao1" << std::endl;	
	  return false;
	}

	//if(erase_end == global_plan.begin())
		//std::cout << "Pocetak" << std::endl;

	if (erase_end != global_plan.begin())
	{
	  //std::cout << "usao2" << std::endl;	
	  global_plan.erase(global_plan.begin(), erase_end);
	}

	return true;
}


bool MyWrapper::transformGlobalPlan(const std::vector<geometry_msgs::PoseStamped>& global_plan,
                  const LocalCostmapROS& costmap, double max_plan_length,
                  std::vector<geometry_msgs::PoseStamped>& transformed_plan, int* current_goal_idx) const
{
  // this method is a slightly modified version of base_local_planner/goal_functions.h

  const geometry_msgs::PoseStamped& plan_pose = global_plan[0];

  transformed_plan.clear();

  if (global_plan.empty())
    {
      ROS_ERROR("Received plan with zero length\n");
      *current_goal_idx = 0;
      return false;
    }

    //we'll discard points on the plan that are outside the local costmap
    double dist_threshold = std::max(costmap.getSizeInCellsX() * costmap.getResolution() / 2.0,
                                     costmap.getSizeInCellsY() * costmap.getResolution() / 2.0);
    dist_threshold *= 0.85; // just consider 85% of the costmap size to better incorporate point obstacle that are
                           // located on the border of the local costmap
    

    int i = 0;
    double sq_dist_threshold = dist_threshold * dist_threshold;
    double sq_dist = 1e10;
    
    //we need to loop to a point on the plan that is within a certain distance of the robot
    bool robot_reached = false;
    for(int j=0; j < (int)global_plan.size(); ++j)
    {
      double x_diff = amcl_pose_.pose.position.x - global_plan[j].pose.position.x;
      double y_diff = amcl_pose_.pose.position.y - global_plan[j].pose.position.y;
      double new_sq_dist = x_diff * x_diff + y_diff * y_diff;
      if (new_sq_dist > sq_dist_threshold)
        break;  // force stop if we have reached the costmap border

      if (robot_reached && new_sq_dist > sq_dist)
        break;

      if (new_sq_dist < sq_dist) // find closest distance
      {
        sq_dist = new_sq_dist;
        i = j;
        if (sq_dist < 0.05)      // 2.5 cm to the robot; take the immediate local minima; if it's not the global
          robot_reached = true;  // minima, probably means that there's a loop in the path, and so we prefer this
      }
    }
    
    geometry_msgs::PoseStamped newer_pose;
    
    double plan_length = 0; // check cumulative Euclidean distance along the plan
    
    //now we'll transform until points are outside of our distance threshold
    while(i < (int)global_plan.size() && sq_dist <= sq_dist_threshold && (max_plan_length<=0 || plan_length <= max_plan_length))
    {
      const geometry_msgs::PoseStamped& pose = global_plan[i];
      // ovdje ide transformacija
	  tf2::doTransform(pose, newer_pose, tf_map_odom_);

      transformed_plan.push_back(newer_pose);

      double x_diff = amcl_pose_.pose.position.x - global_plan[i].pose.position.x;
      double y_diff = amcl_pose_.pose.position.y - global_plan[i].pose.position.y;
      sq_dist = x_diff * x_diff + y_diff * y_diff;
      
      // caclulate distance to previous pose
      if (i>0 && max_plan_length>0)
        plan_length += distance_points2d(global_plan[i-1].pose.position, global_plan[i].pose.position);

      ++i;
    }
        
    // if we are really close to the goal (<sq_dist_threshold) and the goal is not yet reached (e.g. orientation error >>0)
    // the resulting transformed plan can be empty. In that case we explicitly inject the global goal.
    if (transformed_plan.empty())
    {
	  // ovdje ide transformacija	
      tf2::doTransform(global_plan.back(), newer_pose, tf_map_odom_);

      transformed_plan.push_back(newer_pose);
      
      // Return the index of the current goal point (inside the distance threshold)
      if (current_goal_idx) *current_goal_idx = int(global_plan.size())-1;
    }
    else
    {
      // Return the index of the current goal point (inside the distance threshold)
      if (current_goal_idx) *current_goal_idx = i-1; // subtract 1, since i was increased once before leaving the loop
    }
 
  return true;
}



































uint32_t MyWrapper::computeVelocityCommands(const geometry_msgs::PoseStamped& odom_pose,
                                                     const geometry_msgs::TwistStamped& odom_velocity,
                                                     geometry_msgs::TwistStamped &cmd_vel,
                                                     std::string &message)
{
  // check if local_planner initialized
  if(!initialized_)
  {
    ROS_ERROR("teb_local_planner has not been initialized, please call initialize() before using this planner\n");
    message = "teb_local_planner has not been initialized";
    return mbf_msgs::ExePathResult::NOT_INITIALIZED;
  }
  
  cmd_vel.twist.linear.x = cmd_vel.twist.linear.y = cmd_vel.twist.angular.z = 0;
  goal_reached_ = false;
   
  //std::cout << global_plan_.size() << std::endl;
  bool prune = pruneGlobalPlan(global_plan_, config_.trajectory.global_plan_prune_distance);
  //std::cout << std::boolalpha;  
  //std::cout << "prune: " << prune << std::endl;
  //std::cout << global_plan_.size() << std::endl;

   // Transform global plan to the frame of interest (w.r.t. the local costmap)
  std::vector<geometry_msgs::PoseStamped> transformed_plan;
  int goal_idx;
  if (!transformGlobalPlan(global_plan_, costmap_, config_.trajectory.max_global_plan_lookahead_dist, transformed_plan, &goal_idx))
  {
    ROS_WARN("Could not transform the global plan to the frame of the controller\n");
    message = "Could not transform the global plan to the frame of the controller";
    return mbf_msgs::ExePathResult::INTERNAL_ERROR;
  }
  
/*  
    // Get current goal point (last point of the transformed plan)
  robot_goal_.x() = transformed_plan.back().pose.position.x;
  robot_goal_.y() = transformed_plan.back().pose.position.y;
  // Overwrite goal orientation if needed
  if (cfg_.trajectory.global_plan_overwrite_orientation)
  {
    robot_goal_.theta() = estimateLocalGoalOrientation(global_plan_, transformed_plan.back(), goal_idx, tf_plan_to_global);
    // overwrite/update goal orientation of the transformed plan with the actual goal (enable using the plan as initialization)
    tf2::Quaternion q;
    q.setRPY(0, 0, robot_goal_.theta());
    tf2::convert(q, transformed_plan.back().pose.orientation);
  }  
  else
  {
    robot_goal_.theta() = tf2::getYaw(transformed_plan.back().pose.orientation);
  }

  // overwrite/update start of the transformed plan with the actual robot position (allows using the plan as initial trajectory)
  if (transformed_plan.size()==1) // plan only contains the goal
  {
    transformed_plan.insert(transformed_plan.begin(), geometry_msgs::PoseStamped()); // insert start (not yet initialized)
  }
  transformed_plan.front() = robot_pose; // update start
*/  

  // clear currently existing obstacles
  obstacles_.clear();
  
  // update obstacles with costmap
  updateObstacleContainerWithCostmap(odom_pose_);

  // print how many points obstacles there are in the local costmap
  std::cout << "obstacles_.size() = " << obstacles_.size() << std::endl;

  // structure that will be sent to the planning function - it requires Twist message
  geometry_msgs::Twist odom_robot_vel;
  odom_robot_vel.linear = odom_velocity_.twist.linear;
  odom_robot_vel.angular = odom_velocity_.twist.angular;
  
  // call the planning function
  bool success = planner_->plan(transformed_plan, &odom_robot_vel, config_.goal_tolerance.free_goal_vel);
  
  // if planning not successful, call clearPlanner and return NO_VALID_CMD
  if (!success)
  {
    ROS_WARN("teb_local_planner was not able to obtain a local plan for the current setting.\n");
    return mbf_msgs::ExePathResult::NO_VALID_CMD;
  }
    
  // Get the velocity command for this sampling interval
  if (!planner_->getVelocityCommand(cmd_vel.twist.linear.x, cmd_vel.twist.linear.y, cmd_vel.twist.angular.z, config_.trajectory.control_look_ahead_poses))
  {
    ROS_WARN("TebLocalPlannerROS: velocity command invalid. Resetting planner...\n");
    return mbf_msgs::ExePathResult::NO_VALID_CMD;
  }
  
  // Saturate velocity, if the optimization results violates the constraints (could be possible due to soft constraints).
  saturateVelocity(cmd_vel.twist.linear.x, cmd_vel.twist.linear.y, cmd_vel.twist.angular.z,
                   config_.robot.max_vel_x, config_.robot.max_vel_y, config_.robot.max_vel_theta);
 
  return mbf_msgs::ExePathResult::SUCCESS;
}


} // end namespace teb_local_planner
