#ifndef MY_WRAPPER_H_
#define MY_WRAPPER_H_

#include <ros/ros.h>

#include <teb_local_planner/costmap_wrapper.h>
#include <teb_local_planner/optimal_planner.h>
#include <teb_local_planner/homotopy_class_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
#include <string.h>

// transforms
#include <tf2/utils.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <boost/shared_ptr.hpp>

#include <nav_core/base_local_planner.h>

namespace teb_local_planner
{

class MyWrapper 
{

public:
  /**
    * @brief Default constructor
    */
  MyWrapper();

  /**
    * @brief  Destructor
    */
  ~MyWrapper();


   /**
   * @brief Get the current robot footprint/contour model
   * @return Robot footprint model used for optimization
   */
  static RobotFootprintModelPtr getRobotFootprintFromParamServer();
  

  uint32_t computeVelocityCommands(const geometry_msgs::PoseStamped& pose, const geometry_msgs::TwistStamped& velocity, geometry_msgs::TwistStamped &cmd_vel, std::string &message);

  /**
    * @brief Update internal obstacle vector based on occupied costmap cells
    * @remarks All occupied cells will be added as point obstacles.
    * @remarks All previous obstacles are cleared.
    * @sa updateObstacleContainerWithCostmapConverter
    * @todo Include temporal coherence among obstacle msgs (id vector)
    * @todo Include properties for dynamic obstacles (e.g. using constant velocity model)
    */
  void updateObstacleContainerWithCostmap(const geometry_msgs::PoseStamped& pose);		

  /*
   * @brief Saturate the translational and angular velocity to given limits.
   * 
   * The limit of the translational velocity for backwards driving can be changed independently.
   * Do not choose max_vel_x_backwards <= 0. If no backward driving is desired, change the optimization weight for
   * penalizing backwards driving instead.
   * @param[in,out] vx The translational velocity that should be saturated.
   * @param[in,out] vy Strafing velocity which can be nonzero for holonomic robots
   * @param[in,out] omega The angular velocity that should be saturated.
   * @param max_vel_x Maximum translational velocity for forward driving
   * @param max_vel_y Maximum strafing velocity (for holonomic robots)
   * @param max_vel_theta Maximum (absolute) angular velocity
   * @param max_vel_x_backwards Maximum translational velocity for backwards driving
   */
  void saturateVelocity(double& vx, double& vy, double& omega, double max_vel_x, double max_vel_y,
                        double max_vel_theta) const;  
						
 
  bool pruneGlobalPlan(std::vector<geometry_msgs::PoseStamped>& global_plan, double dist_behind_robot);

  bool transformGlobalPlan(const std::vector<geometry_msgs::PoseStamped>& global_plan,
                           const LocalCostmapROS& costmap,
                           double max_plan_length, std::vector<geometry_msgs::PoseStamped>& transformed_plan,
                           int* current_goal_idx = NULL) const;

  /**
    * @brief Estimate the orientation of a pose from the global_plan that is treated as a local goal for the local planner.
    * 
    * If the current (local) goal point is not the final one (global)
    * substitute the goal orientation by the angle of the direction vector between 
    * the local goal and the subsequent pose of the global plan. 
    * This is often helpful, if the global planner does not consider orientations. \n
    * A moving average filter is utilized to smooth the orientation.
    * @param global_plan The global plan
    * @param local_goal Current local goal
    * @param current_goal_idx Index of the current (local) goal pose in the global plan
    * @param[out] tf_plan_to_global Transformation between the global plan and the global planning frame
    * @param moving_average_length number of future poses of the global plan to be taken into account
    * @return orientation (yaw-angle) estimate
    */
  double estimateLocalGoalOrientation(const std::vector<geometry_msgs::PoseStamped>& global_plan, const geometry_msgs::PoseStamped& local_goal,
                                      int current_goal_idx, const geometry_msgs::TransformStamped& tf_plan_to_global, int moving_average_length=3) const;						   






  void initialize(const LocalCostmapROS &local_costmap, const geometry_msgs::PoseStamped &odom_pose, const geometry_msgs::TwistStamped& odom_velocity,
  const geometry_msgs::PoseStamped &amcl_pose, const geometry_msgs::TransformStamped &tf_odom_map_v, const geometry_msgs::TransformStamped &tf_map_odom_v, const std::vector<geometry_msgs::Point>& robotFootprint);

  bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

    /**
    * @return Result code as described on ExePath action result:
    *         SUCCESS         = 0
    *         1..9 are reserved as plugin specific non-error results
    *         FAILURE         = 100   Unspecified failure, only used for old, non-mfb_core based plugins
    *         CANCELED        = 101
    *         NO_VALID_CMD    = 102
    *         PAT_EXCEEDED    = 103
    *         COLLISION       = 104
    *         OSCILLATION     = 105
    *         ROBOT_STUCK     = 106
    *         MISSED_GOAL     = 107
    *         MISSED_PATH     = 108
    *         BLOCKED_PATH    = 109
    *         INVALID_PATH    = 110
    *         TF_ERROR        = 111
    *         NOT_INITIALIZED = 112
    *         INVALID_PLUGIN  = 113
    *         INTERNAL_ERROR  = 114
    *         121..149 are reserved as plugin specific errors
    */
  uint32_t computeVelocityCommandsImage(geometry_msgs::TwistStamped &cmd_vel, std::string &message, std::vector<geometry_msgs::PoseStamped>& local_plan, std::vector<geometry_msgs::PoseStamped>& transformed_plan);


  // parameters
  LocalCostmapROS costmap_;
  boost::shared_ptr<base_local_planner::CostmapModel> costmap_model_;

  geometry_msgs::PoseStamped odom_pose_;
  geometry_msgs::TwistStamped odom_velocity_;
  geometry_msgs::PoseStamped amcl_pose_;
  geometry_msgs::TransformStamped tf_odom_map_;
  geometry_msgs::TransformStamped tf_map_odom_;

  std::vector<geometry_msgs::Point> footprint_spec_; //!< Store the footprint of the robot
  double robot_inscribed_radius_; //!< The radius of the inscribed circle of the robot (collision possible)
  double robot_circumscribed_radius_; //!< The radius of the circumscribed circle of the robot

  bool initialized_;
  PlannerInterfacePtr planner_; //!< Instance of the underlying optimal planner class
  ObstContainer obstacles_; //!< Obstacle vector that should be considered during local trajectory optimization
  TebConfig config_; //!< Config class that stores and manages all related parameters
  std::vector<geometry_msgs::PoseStamped> global_plan_; //!< Store the current global plan
  bool goal_reached_; //!< store whether the goal is reached or not
  double robot_orientation_yaw;
};
  
}; // end namespace teb_local_planner

#endif // MY_WRAPPER_H_


