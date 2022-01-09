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

#include <typeinfo>

#include <std_srvs/Empty.h>



using namespace teb_local_planner; // it is ok here to import everything for testing purposes


// ============= Global Variables ================
// Ok global variables are bad, but here we only have a simple testing node.

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
		ros::init(argc, argv, "perturb_node_tabular_costmap");
		ros::NodeHandle n("~");
				
		std::cout << "perturb_node_tabular_costmap started" << std::endl << std::endl;
		
		// Ucitavanje informacija o lokalnoj costmapi
		std::vector<std::vector<double>> local_costmap_info = parse2DCsvFile("./src/teb_local_planner/src/Data/costmap_info.csv");
				
		// Load amcl_pose
		std::vector<std::vector<double>> amcl = parse2DCsvFile("./src/teb_local_planner/src/Data/amcl_pose.csv");
		std::vector<geometry_msgs::PoseStamped> amcl_pose;
		for(int i = 0; i < amcl.size(); i++) 
		{
		    //std::cout << amcl[i][0] << " " << amcl[i][1] << std::endl;
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
		std::vector<std::vector<double>> tf_odom_map = parse2DCsvFile("./src/teb_local_planner/src/Data/tf_odom_map.csv");
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
		std::vector<std::vector<double>> tf_map_odom = parse2DCsvFile("./src/teb_local_planner/src/Data/tf_map_odom.csv");
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
		std::vector<std::vector<double>> odom = parse2DCsvFile("./src/teb_local_planner/src/Data/odom.csv");
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
		std::vector<std::vector<double>> global_plan = parse2DCsvFile("./src/teb_local_planner/src/Data/plan.csv");
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

		
		// Ucitavanje lokalnog plana
		std::vector<std::vector<double>> local_plan = parse2DCsvFile("./src/teb_local_planner/src/Data/local_plan.csv");
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
		std::vector<std::vector<double>> footprint = parse2DCsvFile("./src/teb_local_planner/src/Data/footprint.csv");
		std::vector<geometry_msgs::Point> footprint_t;
		for(int i = 0; i < footprint.size(); i++) 
		{
			geometry_msgs::Point point;
			point.x = footprint[i][0];
			point.y = footprint[i][1];
			point.z = 0.0;
			//std::cout << point.x << " " << point.y << " " << point.z << std::endl;
			footprint_t.push_back(point);
		}
		
		// Ucitavanje lokalne costmape (perturb lokalne costmape)
		std::vector<std::vector<double>> local_costmap_data = parse2DCsvFile("./src/teb_local_planner/src/Data/costmap_data.csv");
		unsigned char arr[25600];
		unsigned char* local_costmap_data_ptr = NULL;
		// Inicijalizacija instance teb_local_plannera
		MyWrapper wrapper;
		uint32_t rez = 33;
		geometry_msgs::TwistStamped cmd_vel;
		//LocalCostmapROS local_costmap_t(local_costmap_info[0][1], local_costmap_info[0][2], local_costmap_info[0][0], local_costmap_info[0][3], local_costmap_info[0][4]);
		LocalCostmapROS local_costmap_t(local_costmap_info[0][1], local_costmap_info[0][2], local_costmap_info[0][0], local_costmap_info[0][3], local_costmap_info[0][4], local_costmap_data_ptr);
		
			
			    	
		// Kreiranje izlaznog csv fajla za komandne brzine
		std::ofstream myfile;
		myfile.open("./src/teb_local_planner/src/Data/cmd_vel.csv");		
		// Upisivanje naslova kolona u izlazni fajl
		std::string s1 = "cmd_vel_lin_x";
		std::string s2 = "cmd_vel_lin_y";
		std::string s3 = "cmd_vel_ang_z";
		myfile <<  s1 + "," + s2 + "," + s3 + "\n";
		myfile.close();
		
		std::cout << "Vrijeme za inicijalizaciju" << std::endl << std::endl;

		int num_iter = int(local_costmap_data.size() / 60);
		std::cout << "num of iterations: " << num_iter << std::endl;
				
		// For petlja za racunanje komandnih brzina za svaku sempliranu instancu
		for(int i = 0; i < num_iter; i++)
		{
			// Kreiranje lokalne costmape
			for(int j = 0; j < local_costmap_data[0].size(); j++)
			{
				for(int k = 0; k < local_costmap_data[0].size(); k++)
				{
					arr[j*160+k] = local_costmap_data[i*local_costmap_data[0].size() + j][k];
					//std::cout << int(arr[j*160+k]) << std::endl;
					//std::cout << j << std::endl;
				}
			}			
			local_costmap_data_ptr = arr;
			local_costmap_t.setCostmap(local_costmap_data_ptr);
				
			//wrapper.initialize(local_costmap_t, amcl_pose[0],  tf_odom_map_v[0],  tf_map_odom_v[0]);
			wrapper.initialize(local_costmap_t, odom_pose[0], odom_twist[0], amcl_pose[0], tf_odom_map_v[0], tf_map_odom_v[0], footprint_t);
		
			wrapper.setPlan(global_plan_t); // send (global) plan to the teb local planner
			
			std::string message;
			std::cout << "i = " << i << std::endl;
			rez = wrapper.computeVelocityCommands(odom_pose[0], odom_twist[0], cmd_vel, message); // compute command velocities
			std::cout << "Da li je racunanje komandnih brzina bilo ispravno (0-SUCCESS) = " << rez << std::endl;
			
			local_costmap_data_ptr = NULL;

			std::cout <<  std::to_string(cmd_vel.twist.linear.x) + "," + std::to_string(cmd_vel.twist.linear.y) + "," + std::to_string(cmd_vel.twist.angular.z) + "\n" << std::endl;
			
			myfile.open("./src/teb_local_planner/src/Data/cmd_vel.csv", std::ios_base::app);
			myfile <<  std::to_string(cmd_vel.twist.linear.x) + "," + std::to_string(cmd_vel.twist.linear.y) + "," + std::to_string(cmd_vel.twist.angular.z) + "\n";
			std::cout << "Upisao!" << std::endl;
			myfile.close();
			std::cout << "Zatvorio file" << std::endl;

			if(i == num_iter)
			{
				break;
			}			
		}

		std::cout << "Izasao iz petlje" << std::endl;

		std::cout << "Zatvorio file" << std::endl;
		
		std::cout << "before advertising finished" << std::endl << std::endl;

	    ros::ServiceServer service = n.advertiseService("finished", callbackFinished);

		std::cout << "perturb_node_tabular_costmap finished" << std::endl << std::endl;

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
































