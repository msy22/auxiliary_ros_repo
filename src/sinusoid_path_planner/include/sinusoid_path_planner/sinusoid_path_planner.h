/** include the libraries you need in your planner here */
/** for global path planner interface */
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <navfn/navfn_ros.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>

using std::string;

#ifndef SINUSOID_PATH_PLANNER
#define SINUSOID_PATH_PLANNER

namespace SinusoidPathPlanner
{

  class GlobalPlanner : public nav_core::BaseGlobalPlanner
  {
    public:
      GlobalPlanner();
      GlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

      /** overridden classes from interface nav_core::BaseGlobalPlanner **/
      void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
      bool makePlan(const geometry_msgs::PoseStamped& start,
                    const geometry_msgs::PoseStamped& goal,
                    std::vector<geometry_msgs::PoseStamped>& plan);

    protected:
      ros::Publisher plan_pub_;
      bool initialized_;

    private:
      navfn::NavfnROS* navfn_planner = NULL;
  };
}
#endif
