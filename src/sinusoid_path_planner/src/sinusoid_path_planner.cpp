#include <pluginlib/class_list_macros.h>
#include <sinusoid_path_planner/sinusoid_path_planner.h>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(SinusoidPathPlanner::GlobalPlanner, nav_core::BaseGlobalPlanner)

using namespace std;

//Default Constructor
namespace SinusoidPathPlanner
{

  GlobalPlanner::GlobalPlanner ()
  {

  }

  GlobalPlanner::GlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
  {
    initialize(name, costmap_ros);
  }

  void GlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
  {
    cout << "Initalizing planner..." << endl;
    navfn_planner = new navfn::NavfnROS();
    navfn_planner->initialize(name, costmap_ros);
    cout << "Planner successfully initialized!" << endl;
  }

  bool GlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start,
                               const geometry_msgs::PoseStamped& goal,
                               std::vector<geometry_msgs::PoseStamped>& plan )
  {
    cout << "Creating plan" << endl;
    navfn_planner->makePlan(start, goal, plan);
//    navfn_planner->makePlan(start, goal, plan);
//    plan.push_back(start);
//    for (int i=0; i<20; i++)
//    {
//      geometry_msgs::PoseStamped new_goal = goal;
//      tf::Quaternion goal_quat = tf::createQuaternionFromYaw(1.54);

//      new_goal.pose.position.x = -2.5+(0.05*i);
//      new_goal.pose.position.y = -3.5+(0.05*i);

//      new_goal.pose.orientation.x = goal_quat.x();
//      new_goal.pose.orientation.y = goal_quat.y();
//      new_goal.pose.orientation.z = goal_quat.z();
//      new_goal.pose.orientation.w = goal_quat.w();

//      plan.push_back(new_goal);
//    }
//    plan.push_back(goal);
    return true;
  }
}
