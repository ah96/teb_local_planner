#include <teb_local_planner/my_wrapper.h>
#include <teb_local_planner/TebLocalPlannerReconfigureConfig.h>
#include <teb_local_planner/costmap_wrapper.h>

#include <string>
#include <fstream>
#include <vector>
#include <utility> // std::pair
#include <stdexcept> // std::runtime_error
#include <sstream> // std::stringstream

#include <std_msgs/Int64MultiArray.h>
#include <std_msgs/Float32MultiArray.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>

#include <geometry_msgs/Transform.h>

#include <geometry_msgs/Point.h>

#include <typeinfo>

#include <std_srvs/Empty.h>
//#include <std_srvs/srv/empty.hpp>


using namespace teb_local_planner; // it is ok here to import everything for testing purposes


// ============= Global Variables ================
// Ok global variables are bad, but here we only have a simple testing node.
//bool perturbNodeImageFinished = false;

// Read csv file to vector
std::vector<std::vector<double>> parse2DCsvFile(std::string inputFileName) {
 
    std::vector<std::vector<double> > data;
    std::ifstream inputFile(inputFileName);
    int l = 0;
 
    while (inputFile) {
        l++;
        std::string s;
        if (!getline(inputFile, s)) break;
        if (s[0] != '#') {
            std::istringstream ss(s);
            std::vector<double> record;
 
            while (ss) {
                std::string line;
                if (!getline(ss, line, ','))
                    break;
                try {
                    record.push_back(stof(line));
                }
                catch (const std::invalid_argument e) {
                    std::cout << "NaN found in file " << inputFileName << " line " << l
                         << std::endl;
                    e.what();
                }
            }
 
            data.push_back(record);
        }
    }
 
    if (!inputFile.eof()) {
        std::cerr << "Could not read file " << inputFileName << "\n";
        std::__throw_invalid_argument("File not found.");
    }
 
    return data;
}

bool callbackFinished(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    return true;
    //return perturbNodeImageFinished;
}

// =============== Main function =================   Samostalno se pozivaju funkcije iz teb_local_plannera
int main( int argc, char** argv )
{
		ros::init(argc, argv, "perturb_node_image");
		ros::NodeHandle n("~");

        //ros::ServiceServer service = n.advertiseService("finished", callbackFinished);


		std::cout << "perturb_node_image started" << std::endl << std::endl;
		
		// Load local costmap info
		std::vector<std::vector<double>> local_costmap_info = parse2DCsvFile("/home/robolab/amar_ws/src/teb_local_planner/src/Data/costmap_info.csv");
				
		// Load amcl_pose
		std::vector<std::vector<double>> amcl = parse2DCsvFile("/home/robolab/amar_ws/src/teb_local_planner/src/Data/amcl_pose.csv");
		std::vector<geometry_msgs::PoseStamped> amcl_pose;
		for(int i = 0; i < amcl.size(); i++) 
		{
			geometry_msgs::PoseStamped pose;
			pose.pose.position.x = amcl[i][0];
			pose.pose.position.y = amcl[i][1];
			pose.pose.position.z = 0.0;
			pose.pose.orientation.x = 0.0;
			pose.pose.orientation.y = 0.0;
			pose.pose.orientation.z = amcl[i][2];
			pose.pose.orientation.w = amcl[i][3];
			amcl_pose.push_back(pose);
		}
		
		// Load tf_odom_map
		std::vector<std::vector<double>> tf_odom_map = parse2DCsvFile("/home/robolab/amar_ws/src/teb_local_planner/src/Data/tf_odom_map.csv");
		std::vector<geometry_msgs::TransformStamped> tf_odom_map_v;
		for(int i = 0; i < tf_odom_map.size(); i++) 
		{
			geometry_msgs::TransformStamped transform;
			transform.transform.translation.x = tf_odom_map[i][0];
			transform.transform.translation.y = tf_odom_map[i][1];
			transform.transform.translation.z = tf_odom_map[i][2];
			transform.transform.rotation.x = tf_odom_map[i][3];
			transform.transform.rotation.y = tf_odom_map[i][4];
			transform.transform.rotation.z = tf_odom_map[i][5];
			transform.transform.rotation.w = tf_odom_map[i][6];
			//std::cout << transform << std::endl;
			tf_odom_map_v.push_back(transform);
		}
		
		// Load tf_map_odom
		std::vector<std::vector<double>> tf_map_odom = parse2DCsvFile("/home/robolab/amar_ws/src/teb_local_planner/src/Data/tf_map_odom.csv");
		std::vector<geometry_msgs::TransformStamped> tf_map_odom_v;
		for(int i = 0; i < tf_odom_map.size(); i++) 
		{
			geometry_msgs::TransformStamped transform;
			transform.transform.translation.x = tf_map_odom[i][0];
			transform.transform.translation.y = tf_map_odom[i][1];
			transform.transform.translation.z = tf_odom_map[i][2];
			transform.transform.rotation.x = tf_odom_map[i][3];
			transform.transform.rotation.y = tf_odom_map[i][4];
			transform.transform.rotation.z = tf_map_odom[i][5];
			transform.transform.rotation.w = tf_map_odom[i][6];
			//std::cout << transform << std::endl;
			tf_map_odom_v.push_back(transform);
		}
		
		// Load odometry
		std::vector<std::vector<double>> odom = parse2DCsvFile("/home/robolab/amar_ws/src/teb_local_planner/src/Data/odom.csv");
		std::vector<geometry_msgs::PoseStamped> odom_pose;
		std::vector<geometry_msgs::TwistStamped> odom_twist;
		for(int i = 0; i < odom.size(); i++) 
		{
			geometry_msgs::PoseStamped pose;
			pose.header.seq = i;
			pose.pose.position.x = odom[i][0];
			pose.pose.position.y = odom[i][1];
			pose.pose.position.z = 0.0;
			pose.pose.orientation.x = 0.0;
			pose.pose.orientation.y = 0.0;
			pose.pose.orientation.z = odom[i][2];
			pose.pose.orientation.w = odom[i][3];
			odom_pose.push_back(pose);
			
			geometry_msgs::TwistStamped tw;
			tw.twist.linear.x = odom[i][4];
			tw.twist.linear.y = 0.0;
			tw.twist.linear.z = 0.0;
			tw.twist.angular.x = 0.0;
			tw.twist.angular.y = 0.0;
			tw.twist.angular.z = odom[i][5];
			odom_twist.push_back(tw);
		}	
		
		// Load plan from global planner
		std::vector<std::vector<double>> global_plan = parse2DCsvFile("/home/robolab/amar_ws/src/teb_local_planner/src/Data/plan.csv");
		std::vector<geometry_msgs::PoseStamped> global_plan_t;
		for(int i = 0; i < global_plan.size(); i++) 
		{
			geometry_msgs::PoseStamped pose;
			pose.header.seq = i;
			pose.pose.position.x = global_plan[i][0];
			pose.pose.position.y = global_plan[i][1];
			pose.pose.position.z = 0.0;
			pose.pose.orientation.x = 0.0;
			pose.pose.orientation.y = 0.0;
			pose.pose.orientation.z = global_plan[i][2];
			pose.pose.orientation.w = global_plan[i][3];
			global_plan_t.push_back(pose);
		}
		
		// Load local plan
		std::vector<std::vector<double>> local_plan = parse2DCsvFile("/home/robolab/amar_ws/src/teb_local_planner/src/Data/local_plan.csv");
		std::vector<geometry_msgs::PoseStamped> local_plan_t;
		for(int i = 0; i < local_plan.size(); i++) 
		{
			geometry_msgs::PoseStamped pose;
			pose.header.seq = i;
			pose.pose.position.x = local_plan[i][0];
			pose.pose.position.y = local_plan[i][1];
			pose.pose.position.z = 0.0;
			pose.pose.orientation.x = 0.0;
			pose.pose.orientation.y = 0.0;
			pose.pose.orientation.z = local_plan[i][2];
			pose.pose.orientation.w = local_plan[i][3];
			local_plan_t.push_back(pose);
		}
		
		// Load footprint
		std::vector<std::vector<double>> footprint = parse2DCsvFile("/home/robolab/amar_ws/src/teb_local_planner/src/Data/footprint.csv");
		std::vector<geometry_msgs::Point> footprint_t;
		for(int i = 0; i < footprint.size(); i++) 
		{
			geometry_msgs::Point point;
			point.x = local_plan[i][0];
			point.y = local_plan[i][1];
			point.z = 0.0;
			footprint_t.push_back(point);
		}
		
		// Load local costmap perturbations
		std::vector<std::vector<double>> local_costmap_data = parse2DCsvFile("/home/robolab/amar_ws/src/teb_local_planner/src/Data/costmap_data.csv");
		unsigned char arr[25600];
		unsigned char* local_costmap_data_ptr = NULL;

		// teb_local_planner declaration
		MyWrapper wrapper;
		uint32_t rez;
		geometry_msgs::TwistStamped cmd_vel;
		// local costmap declaration and partial initialization
		LocalCostmapROS local_costmap_t(local_costmap_info[0][1], local_costmap_info[0][2], local_costmap_info[0][0], local_costmap_info[0][3], local_costmap_info[0][4], odom_pose[0]);
			
			    	
		// Command velocities output .csv file
		std::ofstream myfile1;
		myfile1.open("/home/robolab/amar_ws/src/teb_local_planner/src/Data/cmd_vel.csv");
		// Write columns' names to output files
		std::string s1 = "cmd_vel_lin_x";
		std::string s2 = "cmd_vel_lin_y";
		std::string s3 = "cmd_vel_ang_z";
		myfile1 <<  s1 + "," + s2 + "," + s3 + "\n";
		myfile1.close();
		
		
		// Local plans output .csv file
		std::ofstream myfile2;
		myfile2.open("/home/robolab/amar_ws/src/teb_local_planner/src/Data/local_plans.csv");		
		// Write columns' names to output files
		s1 = "local_plan_position_x";
		s2 = "local_plan_position_y";
		s3 = "local_plan_orientation_z";
		std::string s4 = "local_plan_orientation_w";
		std::string s5 = "ID";
		myfile2 <<  s1 + "," + s2 + "," + s3 + "," + s4 + "," + s5 + "\n";
		myfile2.close();


		// Tranformed plan output .csv file
		std::ofstream myfile3;
		myfile3.open("/home/robolab/amar_ws/src/teb_local_planner/src/Data/transformed_plan.csv");
		// Write columns' names to output files
		s1 = "transformed_plan_position_x";
		s2 = "transformed_plan_position_y";
		s3 = "transformed_plan_orientation_z";
		s4 = "transformed_plan_orientation_w";
		myfile3 <<  s1 + "," + s2 + "," + s3 + "," + s4 + "\n";
		myfile3.close();

		
		
		std::cout << "Vrijeme za inicijalizaciju" << std::endl << std::endl;
				
		// Calculate command velocities for every sampled perturbation
		for(int i = 0; i < int(local_costmap_data.size() / 160); i++)
		{
			// Create/Get local costmap
			for(int j = 0; j < local_costmap_data[0].size(); j++)
			{
				for(int k = 0; k < local_costmap_data[0].size(); k++)
				{
					arr[j*160+k] = local_costmap_data[i*local_costmap_data[0].size() + j][k]; // correct this 160
					//std::cout << int(arr[j*160+k]) << std::endl;
					//std::cout << j << std::endl;
				}
			}			
			local_costmap_data_ptr = arr;
			// full initialization of the local costmap - set local costmap data to the local costmap instance
			local_costmap_t.setCostmap(local_costmap_data_ptr);

            // teb_local_planner initialization
			wrapper.initialize(local_costmap_t, odom_pose[0], odom_twist[0], amcl_pose[0], tf_odom_map_v[0], tf_map_odom_v[0], footprint_t);

		    // send global plan to the teb_local_planner instance
			wrapper.setPlan(global_plan_t); // send (global) plan to the teb local planner

			// declare local variables used
			std::string message = "None";
			std::vector<geometry_msgs::PoseStamped> local_plan;
			std::vector<geometry_msgs::PoseStamped> transformed_plan;

			// print current iteration
			std::cout << "i = " << i << std::endl;
			rez = wrapper.computeVelocityCommandsImage(cmd_vel, message, local_plan, transformed_plan); // compute command velocities

			// print result of the algorithm (whether it was successfull or no) and its message
			std::cout << "Da li je racunanje komandnih brzina bilo ispravno (0-SUCCESS) = " << rez << std::endl;
			std::cout << "Message: " << message << std::endl;

			// put local costmap pointer to the null pointer for the safety
			local_costmap_data_ptr = NULL;

            // print command velocities
			std::cout <<  std::to_string(cmd_vel.twist.linear.x) + "," + std::to_string(cmd_vel.twist.linear.y) + "," + std::to_string(cmd_vel.twist.angular.z) + "\n" << std::endl;

			// write command velocities to the .csv file
			myfile1.open("/home/robolab/amar_ws/src/teb_local_planner/src/Data/cmd_vel.csv", std::ios::app);
			myfile1 <<  std::to_string(cmd_vel.twist.linear.x) + "," + std::to_string(cmd_vel.twist.linear.y) + "," + std::to_string(cmd_vel.twist.angular.z) + "\n";
			myfile1.close();

			// write local plans to the .csv file
			myfile2.open("/home/robolab/amar_ws/src/teb_local_planner/src/Data/local_plans.csv", std::ios::app);
			for(int j = 0; j < local_plan.size(); j++)
				myfile2 <<  std::to_string(local_plan[j].pose.position.x) + "," + std::to_string(local_plan[j].pose.position.y) + "," + std::to_string(local_plan[j].pose.orientation.z) 
			+ "," + std::to_string(local_plan[j].pose.orientation.w) + "," + std::to_string(i) + "\n";			
			myfile2.close();

            // write transformed (part of the) (global) plan (from the global planner) to the .csv file
			if(i == 0)
			{
                myfile3.open("/home/robolab/amar_ws/src/teb_local_planner/src/Data/transformed_plan.csv", std::ios::app);
                for(int j = 0; j < transformed_plan.size(); j++)
                    myfile3 <<  std::to_string(transformed_plan[j].pose.position.x) + "," + std::to_string(transformed_plan[j].pose.position.y) + "," +
                    std::to_string(transformed_plan[j].pose.orientation.z) + "," + std::to_string(transformed_plan[j].pose.orientation.w) + "\n";
                myfile3.close();
			}
		}

	    ros::ServiceServer service = n.advertiseService("finished", callbackFinished);

		std::cout << "perturb_node_image finished" << std::endl << std::endl;

		//perturbNodeImageFinished = true;

		//ros::shutdown();
		
		ros::spin();
	
		return 0;
}







/*
for (auto l : amcl) 
{
	for (auto x : l)
		std::cout << x << " ";
	std::cout << std::endl;
}		  
*/
































