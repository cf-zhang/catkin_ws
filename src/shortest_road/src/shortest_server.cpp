#include <ros/ros.h>
#include <navfn/MakeNavPlan.h>
#include <geometry_msgs/PoseStamped.h>
bool makePlanService(navfn::MakeNavPlan::Request& req, navfn::MakeNavPlan::Response& resp) {
    std::vector<geometry_msgs::PoseStamped> path;
    geometry_msgs::PoseStamped ps;
    ps.pose.position.x = 1.0;
    ps.pose.position.y = 1.0;

    path.push_back(ps);
    ps.pose.position.x = 2.0;
    ps.pose.position.y = 2.0;

    path.push_back(ps);
    ps.pose.position.x = 3.0;
    ps.pose.position.y = 3.0;
    path.push_back(ps);
    resp.path = path;

    return true;
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "shortest_server");

    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("make_plan", makePlanService);

    ros::spin();
    return 0;    
}
