#include <ros/ros.h>
#include <ros/spinner.h>
#include <ros/callback_queue.h>
#include <actionlib/server/simple_action_server.h>
#include <roboy_control_msgs/PerformMovementAction.h>
#include <roboy_control_msgs/PerformMovementsAction.h>
//#include <actionlib_tutorials/FibonacciAction.h>
#include <roboy_plexus/myoControl.hpp>

using namespace std;

class PerformMovementAction {
protected:
    ros::NodeHandle nh_;
    boost::shared_ptr<MyoControl> myoControl;
    string action_name;
    actionlib::SimpleActionServer<roboy_control_msgs::PerformMovementAction> performMovement_as;
    roboy_control_msgs::PerformMovementFeedback feedback;
    roboy_control_msgs::PerformMovementResult result;

public:
    PerformMovementAction(boost::shared_ptr<MyoControl> myoControl, string name);
    void executeCB(const roboy_control_msgs::PerformMovementGoalConstPtr &goal);
};

class PerformMovementsAction {
protected:
    ros::NodeHandle nh_;
    boost::shared_ptr<MyoControl> myoControl;
    string action_name;
    actionlib::SimpleActionServer<roboy_control_msgs::PerformMovementsAction> performMovements_as;
    roboy_control_msgs::PerformMovementsFeedback feedback;
    roboy_control_msgs::PerformMovementsResult result;

public:
    PerformMovementsAction(boost::shared_ptr<MyoControl> myoControl, string name);
    void executeCB(const roboy_control_msgs::PerformMovementsGoalConstPtr &goal);
};

//class PerformBehaviorAction {
//protected:
//    ros::NodeHandle nh_;
//    boost::shared_ptr<MyoControl> myoControl;
//    string action_name;
//    actionlib::SimpleActionServer<roboy_control_msgs::PerformBehaviorAction> performMovements_as;
//    roboy_control_msgs::PerformBehaviorFeedback feedback;
//    roboy_control_msgs::PerformBehaviorResult result;
//
//public:
//    PerformBehaviorAction(boost::shared_ptr<MyoControl> myoControl, string name);
//    void executeCB(const roboy_control_msgs::PerformBehaviorGoalConstPtr &goal);
//};

