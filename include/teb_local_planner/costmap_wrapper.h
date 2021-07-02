#include <costmap_2d/costmap_2d.h>

#include <vector>
#include <math.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>

#ifndef COSTMAP_WRAPPER_H_
#define COSTMAP_WRAPPER_H_


namespace teb_local_planner
{

class LocalCostmapROS :  public costmap_2d::Costmap2D
{
	public:
	LocalCostmapROS();
	LocalCostmapROS(unsigned int cells_size_x, unsigned int cells_size_y, double resolution, double origin_x, double origin_y, unsigned char* costmap);
	LocalCostmapROS(unsigned int cells_size_x, unsigned int cells_size_y, double resolution, double origin_x, double origin_y, const geometry_msgs::PoseStamped& odom_pose);
	~LocalCostmapROS();
	void setCostmap(unsigned char* costmap);

    inline double distance(double x0, double y0, double x1, double y1);
    double distanceToLine(double pX, double pY, double x0, double y0, double x1, double y1);
    void calculateMinAndMaxDistances(const std::vector<geometry_msgs::Point>& footprint, double& min_dist, double& max_dist);
    bool worldToMap(double wx, double wy, unsigned int& mx, unsigned int& my) const;
    double footprintCostHelper(const geometry_msgs::Point& position, const std::vector<geometry_msgs::Point>& footprint, double inscribed_radius, double circumscribed_radius);
	double footprintCost(double x, double y, double theta, const std::vector<geometry_msgs::Point>& footprint_spec, double inscribed_radius, double circumscribed_radius);

	inline unsigned int getIndex(unsigned int mx, unsigned int my) const;
	unsigned char getCost(unsigned int mx, unsigned int my) const;

    double lineCost(int x0, int x1, int y0, int y1);
	double pointCost(int x, int y);

	geometry_msgs::PoseStamped odom_pose_;
};


} // namespace teb_local_planner

#endif // COSTMAP_WRAPPER_H_