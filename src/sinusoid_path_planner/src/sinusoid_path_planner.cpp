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
      // Set up pubs/subs for class
      ros::NodeHandle private_nh("sinusoid_path_planner");
      plan_pub_ = private_nh.advertise<nav_msgs::Path>("custom_plan", 1);
      ROS_INFO("Created global planner sinusoid_path_planner/GlobalPlanner");

      // Initialize Navfn planner
      navfn_planner = new navfn::NavfnROS();
      navfn_planner->initialize(name, costmap_ros);

      // Set up default parameters
      sinusoidal_path = false;

      initialized_ = true;
    }
    else
      ROS_WARN("This planner has already been initialized!");
  }



  void GlobalPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path,
                                  double r, double g, double b, double a)
  {
    if(!initialized_)
    {
      ROS_ERROR("This planner has not been initialized yet, but it is being "
                "used, please call initialize() before use");
      return;
    }

    //create a message for the plan
    nav_msgs::Path gui_path;
    gui_path.poses.resize(path.size());
    if(!path.empty())
    {
      gui_path.header.frame_id = path[0].header.frame_id;
      gui_path.header.stamp = path[0].header.stamp;
    }

    // Extract the plan in world co-ordinates, we assume the path is all in the same frame
    for(unsigned int i=0; i < path.size(); i++){
      gui_path.poses[i] = path[i];
    }

    plan_pub_.publish(gui_path);
  }



  bool GlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start,
                               const geometry_msgs::PoseStamped& goal,
                               std::vector<geometry_msgs::PoseStamped>& plan )
  {
    if (sinusoidal_path)
    {
      ROS_INFO("Publishing a sinusoidal path");
      // make a sinusoidal path
    }
    else
    {
      ROS_INFO("Publishing navfn path");
//      plan.push_back(start);
//      plan.push_back(goal);
      navfn_planner->makePlan(start, goal, plan);
    }
    publishPlan(plan, 0.0, 1.0, 0.0, 0.0);
    return true;
  }
}
