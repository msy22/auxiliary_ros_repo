#include <pluginlib/class_list_macros.h>
#include <sinusoid_path_planner/sinusoid_path_planner.h>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(SinusoidPathPlanner::GlobalPlanner, nav_core::BaseGlobalPlanner)

using namespace std;

//Default Constructor
namespace SinusoidPathPlanner
{

  GlobalPlanner::GlobalPlanner ()
    : initialized_(false)
  {

  }

  GlobalPlanner::GlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
    : initialized_(false)
  {
    initialize(name, costmap_ros);
  }

  void GlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
  {
    if (!initialized_)
    {
      ros::NodeHandle private_nh("sinusoid_path_planner");
      plan_pub_ = private_nh.advertise<nav_msgs::Path>("custom_plan", 1);

      initialized_ = true;
    }
    else
      ROS_WARN("This planner has already been initialized!");
  }


  bool GlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start,
                               const geometry_msgs::PoseStamped& goal,
                               std::vector<geometry_msgs::PoseStamped>& plan )
  {
    plan.push_back(start);
    {


    }
    plan.push_back(goal);
    return true;
  }
}
