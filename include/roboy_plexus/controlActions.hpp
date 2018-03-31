#include <ros/ros.h>
#include <ros/spinner.h>
#include <ros/callback_queue.h>
#include <actionlib/server/simple_action_server.h>
#include <roboy_communication_control/PerformMovementAction.h>
#include <roboy_communication_control/PerformMovementsAction.h>
#include <actionlib_tutorials/FibonacciAction.h>
#include <roboy_plexus/myoControl.hpp>

using namespace std;

class PerformMovementAction {
protected:
    ros::NodeHandle nh_;
    boost::shared_ptr<MyoControl> myoControl;
    string action_name;
    actionlib::SimpleActionServer<roboy_communication_control::PerformMovementAction> performMovement_as;
    roboy_communication_control::PerformMovementFeedback feedback;
    roboy_communication_control::PerformMovementResult result;

public:
    PerformMovementAction(boost::shared_ptr<MyoControl> myoControl, string name);
    void executeCB(const roboy_communication_control::PerformMovementGoalConstPtr &goal);
};

class PerformMovementsAction {
protected:
    ros::NodeHandle nh_;
    boost::shared_ptr<MyoControl> myoControl;
    string action_name;
    actionlib::SimpleActionServer<roboy_communication_control::PerformMovementsAction> performMovements_as;
    roboy_communication_control::PerformMovementsFeedback feedback;
    roboy_communication_control::PerformMovementsResult result;

public:
    PerformMovementsAction(boost::shared_ptr<MyoControl> myoControl, string name);
    void executeCB(const roboy_communication_control::PerformMovementsGoalConstPtr &goal);
};

//class PerformBehaviorAction {
//protected:
//    ros::NodeHandle nh_;
//    boost::shared_ptr<MyoControl> myoControl;
//    string action_name;
//    actionlib::SimpleActionServer<roboy_communication_control::PerformBehaviorAction> performMovements_as;
//    roboy_communication_control::PerformBehaviorFeedback feedback;
//    roboy_communication_control::PerformBehaviorResult result;
//
//public:
//    PerformBehaviorAction(boost::shared_ptr<MyoControl> myoControl, string name);
//    void executeCB(const roboy_communication_control::PerformBehaviorGoalConstPtr &goal);
//};

