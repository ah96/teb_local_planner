#include <teb_local_planner/costmap_wrapper.h>
#include <base_local_planner/line_iterator.h>


namespace teb_local_planner
{
	teb_local_planner::LocalCostmapROS::LocalCostmapROS()
	{
	}

	teb_local_planner::LocalCostmapROS::LocalCostmapROS(unsigned int cells_size_x, unsigned int cells_size_y, double resolution, double origin_x, double origin_y, unsigned char* costmap)
	{
		size_x_ = cells_size_x;
		size_y_ = cells_size_y;
		resolution_ = resolution; 
		origin_x_ = origin_x;
		origin_y_ = origin_y;
		costmap_ = costmap;
	}
	
	teb_local_planner::LocalCostmapROS::LocalCostmapROS(unsigned int cells_size_x, unsigned int cells_size_y, double resolution, double origin_x, double origin_y, const geometry_msgs::PoseStamped& odom_pose)
	{
		size_x_ = cells_size_x;
		size_y_ = cells_size_y;
		resolution_ = resolution; 
		origin_x_ = origin_x;
		origin_y_ = origin_y;
		costmap_ = NULL;
		odom_pose_ = odom_pose;
	}

	teb_local_planner::LocalCostmapROS::~LocalCostmapROS()
	{
		costmap_ = NULL;
	}
	
	void teb_local_planner::LocalCostmapROS::setCostmap(unsigned char* costmap)
	{
		costmap_ = costmap;
	}

    inline double teb_local_planner::LocalCostmapROS::distance(double x0, double y0, double x1, double y1)
    {
        return hypot(x1 - x0, y1 - y0);
    }

    double teb_local_planner::LocalCostmapROS::distanceToLine(double pX, double pY, double x0, double y0, double x1, double y1)
    {
        double A = pX - x0;
        double B = pY - y0;
        double C = x1 - x0;
        double D = y1 - y0;

        double dot = A * C + B * D;
        double len_sq = C * C + D * D;
        double param = dot / len_sq;

        double xx, yy;

        if (param < 0)
        {
            xx = x0;
            yy = y0;
        }
        else if (param > 1)
        {
            xx = x1;
            yy = y1;
        }
        else
        {
            xx = x0 + param * C;
            yy = y0 + param * D;
        }

        return distance(pX, pY, xx, yy);
    }

	void teb_local_planner::LocalCostmapROS::calculateMinAndMaxDistances(const std::vector<geometry_msgs::Point>& footprint, double& min_dist, double& max_dist)
    {
        min_dist = std::numeric_limits<double>::max();
        max_dist = 0.0;

        if (footprint.size() <= 2)
        {
            return;
        }

        for (unsigned int i = 0; i < footprint.size() - 1; ++i)
        {
            // check the distance from the robot center point to the first vertex
            double vertex_dist = distance(odom_pose_.pose.position.x, odom_pose_.pose.position.y, footprint[i].x, footprint[i].y);
            double edge_dist = distanceToLine(odom_pose_.pose.position.x, odom_pose_.pose.position.y, footprint[i].x, footprint[i].y, footprint[i + 1].x, footprint[i + 1].y);
            min_dist = std::min(min_dist, std::min(vertex_dist, edge_dist));
            max_dist = std::max(max_dist, std::max(vertex_dist, edge_dist));
        }

        // we also need to do the last vertex and the first vertex
        double vertex_dist = distance(odom_pose_.pose.position.x, odom_pose_.pose.position.y, footprint.back().x, footprint.back().y);
        double edge_dist = distanceToLine(odom_pose_.pose.position.x, odom_pose_.pose.position.y, footprint.back().x, footprint.back().y, footprint.front().x, footprint.front().y);
        min_dist = std::min(min_dist, std::min(vertex_dist, edge_dist));
        max_dist = std::max(max_dist, std::max(vertex_dist, edge_dist));
    }

    bool teb_local_planner::LocalCostmapROS::worldToMap(double wx, double wy, unsigned int& mx, unsigned int& my) const
    {
        ///*
        std::cout << "origin_x_: " << origin_x_ << std::endl;
        std::cout << "origin_y_: " << origin_y_ << std::endl;
        std::cout << "size_x_: " << size_x_ << std::endl;
        std::cout << "size_y_: " << size_y_ << std::endl;
        std::cout << "resolution_: " << resolution_ << std::endl;
        //*/
        if (wx < origin_x_ || wy < origin_y_)
        {
            std::cout << "wx = " << wx << ", wy = " << wy << ", origin_x_ = " << origin_x_ << ", origin_y_ = " << origin_y_ << std::endl; 
            std::cout << "worldToMap 1" << std::endl;
            return false;
        }
            
        mx = (int)((wx - origin_x_) / resolution_);
        my = (int)((wy - origin_y_) / resolution_);

        if (mx < size_x_ && my < size_y_)
            return true;

        std::cout << "worldToMap 2" << std::endl;
        return false;
    }

    double teb_local_planner::LocalCostmapROS::footprintCostHelper(const geometry_msgs::Point& position, const std::vector<geometry_msgs::Point>& footprint, double inscribed_radius, double circumscribed_radius)
    {
        //used to put things into grid coordinates
        unsigned int cell_x, cell_y;

        //get the cell coord of the center point of the robot
        if(!worldToMap(position.x, position.y, cell_x, cell_y))
        {
            std::cout << "footprintCostHelper False 1" << std::endl;
            return -1.0;
        }

        //if number of points in the footprint is less than 3, we'll just assume a circular robot
        if(footprint.size() < 3)
        {
            unsigned char cost = getCost(cell_x, cell_y);
            //if(cost == LETHAL_OBSTACLE || cost == INSCRIBED_INFLATED_OBSTACLE)
            //if(cost == LETHAL_OBSTACLE || cost == INSCRIBED_INFLATED_OBSTACLE || cost == NO_INFORMATION)
            if(cost == 99)
            {
                std::cout << "footprintCostHelper False 2" << std::endl;
                return -1.0;
            }
            return cost;
        }

        //now we really have to lay down the footprint in the costmap grid
        unsigned int x0, x1, y0, y1;
        double line_cost = 0.0;
        double footprint_cost = 0.0;

        //std::cout << "footprint.size(): " << footprint.size() << std::endl;

        //we need to rasterize each line in the footprint
        for(unsigned int i = 0; i < footprint.size() - 1; ++i)
        {
            /*
            std::cout << "i in footprintCostHelper: " << i << std::endl;
            std::cout << "footprint[i].x: " << footprint[i].x << std::endl;
            std::cout << "footprint[i].y: " << footprint[i].y << std::endl;
            std::cout << "footprint[i+1].x: " << footprint[i+1].x << std::endl;
            std::cout << "footprint[i+1].y: " << footprint[i+1].y << std::endl; // << std::endl;
            */
            //get the cell coord of the first point
            if(!worldToMap(footprint[i].x, footprint[i].y, x0, y0))
            {
                std::cout << "footprintCostHelper False 3" << std::endl;
                return -1.0;
            }

            //get the cell coord of the second point
            if(!worldToMap(footprint[i + 1].x, footprint[i + 1].y, x1, y1))
            {
                std::cout << "footprintCostHelper False 4" << std::endl;
                return -1.0;
            }

            line_cost = lineCost(x0, x1, y0, y1);
            footprint_cost = std::max(line_cost, footprint_cost);
            /*
            std::cout << "line cost: " << line_cost << std::endl;
            std::cout << "x0: " << x0 << std::endl;
            std::cout << "y0: " << y0 << std::endl;
            std::cout << "x1: " << x1 << std::endl;
            std::cout << "x1: " << y1 << std::endl << std::endl;
            */
            //if there is an obstacle that hits the line... we know that we can return false right away
            if(line_cost < 0)
            {
                std::cout << "footprintCostHelper False 5" << std::endl;
                return -1.0;
            }
        }

        //we also need to connect the first point in the footprint to the last point
        //get the cell coord of the last point
        if(!worldToMap(footprint.back().x, footprint.back().y, x0, y0))
        {
                std::cout << "footprintCostHelper False 6" << std::endl;
                return -1.0;
        }

        //get the cell coord of the first point
        if(!worldToMap(footprint.front().x, footprint.front().y, x1, y1))
        {
                std::cout << "footprintCostHelper False 7" << std::endl;
                return -1.0;
        }

        line_cost = lineCost(x0, x1, y0, y1);
        footprint_cost = std::max(line_cost, footprint_cost);

        if(line_cost < 0)
        {
                std::cout << "footprintCostHelper False 8" << std::endl;
                return -1.0;
        }

        //if all line costs are legal... then we can return that the footprint is legal
        return footprint_cost;
    }

    //calculate the cost of a ray-traced line
    double teb_local_planner::LocalCostmapROS::lineCost(int x0, int x1, int y0, int y1)
    {
        double line_cost = 0.0;
        double point_cost = -1.0;

        for( base_local_planner::LineIterator line( x0, y0, x1, y1 ); line.isValid(); line.advance() )
        {
            point_cost = pointCost( line.getX(), line.getY() ); //Score the current point

            if(point_cost < 0)
                return -1;

            if(line_cost < point_cost)
                line_cost = point_cost;
        }

        return line_cost;
    }

    double teb_local_planner::LocalCostmapROS::pointCost(int x, int y)
    {
        unsigned char cost = getCost(x, y);
        //if the cell is in an obstacle the path is invalid
        //if(cost == LETHAL_OBSTACLE){
        //if(cost == LETHAL_OBSTACLE || cost == NO_INFORMATION)
        if(cost == 99)
        {
            return -1;
        }

        return cost;
    }

	double teb_local_planner::LocalCostmapROS::footprintCost(double x, double y, double theta, const std::vector<geometry_msgs::Point>& footprint_spec, double inscribed_radius = 0.0, double circumscribed_radius=0.0)
	{
        double cos_th = cos(theta);
        double sin_th = sin(theta);
        std::vector<geometry_msgs::Point> oriented_footprint;
        for(unsigned int i = 0; i < footprint_spec.size(); ++i)
        {
            geometry_msgs::Point new_pt;
            new_pt.x = x + (footprint_spec[i].x - odom_pose_.pose.position.x) * sin_th + (footprint_spec[i].y - odom_pose_.pose.position.y) * cos_th; //footprint_spec[i].x + (x - odom_pose_.pose.position.x);  //x + (footprint_spec[i].x - odom_pose_.pose.position.x) * sin_th + (footprint_spec[i].y - odom_pose_.pose.position.y) * cos_th;
            new_pt.y = y - (footprint_spec[i].x - odom_pose_.pose.position.x) * cos_th + (footprint_spec[i].y - odom_pose_.pose.position.y) * sin_th; //footprint_spec[i].y + (y - odom_pose_.pose.position.y); //y - (footprint_spec[i].x - odom_pose_.pose.position.x) * cos_th + (footprint_spec[i].y - odom_pose_.pose.position.y) * sin_th;
            oriented_footprint.push_back(new_pt);
         }

        geometry_msgs::Point robot_position;
        robot_position.x = x;
        robot_position.y = y;

        if(inscribed_radius==0.0)
        {
            calculateMinAndMaxDistances(footprint_spec, inscribed_radius, circumscribed_radius);
        }

        return footprintCostHelper(robot_position, oriented_footprint, inscribed_radius, circumscribed_radius);
        //return footprintCostHelper(robot_position, footprint_spec, inscribed_radius, circumscribed_radius);
    }

    inline unsigned int teb_local_planner::LocalCostmapROS::getIndex(unsigned int mx, unsigned int my) const
    {
        return my * size_x_ + mx;
    }

    unsigned char teb_local_planner::LocalCostmapROS::getCost(unsigned int mx, unsigned int my) const
    {
        return costmap_[getIndex(mx, my)];
    }


} // namespace teb_local_planner
