#include "roboy_plexus/controlActions.hpp"

PerformMovementAction::PerformMovementAction(boost::shared_ptr<MyoControl> myoControl_, string name) :
        myoControl(myoControl_),
        performMovement_as(nh_, action_name, boost::bind(&PerformMovementAction::executeCB, this, _1), false),
        action_name(name)
{
    performMovement_as.start();
    ROS_INFO_STREAM( action_name + ": started action server");
}


void PerformMovementAction::executeCB(const roboy_communication_control::PerformMovementGoalConstPtr &goal) {
    string file = myoControl->trajectories_folder + goal->action;
    const char *fileName = file.c_str();
    result.success = myoControl->playTrajectory(fileName);
    performMovement_as.setSucceeded(result);

};

PerformMovementsAction::PerformMovementsAction(boost::shared_ptr<MyoControl> myoControl_, string name) :
        myoControl(myoControl_),
        performMovements_as(nh_, action_name, boost::bind(&PerformMovementsAction::executeCB, this, _1), false),
        action_name(name)
{
    performMovements_as.start();
    ROS_INFO_STREAM( action_name + ": started action server");
}

void PerformMovementsAction::executeCB(const roboy_communication_control::PerformMovementsGoalConstPtr &goal) {

    bool success;
    for (string actionName: goal->actions) {
        if (actionName.find("pause") != std::string::npos) {
            string delimiter = "_";
            int pause = stoi(actionName.substr(0, actionName.find(delimiter)));
            ros::Duration(pause).sleep();
            success = true && success;
        }
        else if (actionName.find("relax") != std::string::npos) {
            myoControl->allToDisplacement(0);
            success = true && success;
        }
        else {
            actionName = myoControl->trajectories_folder + actionName;
            success = myoControl->playTrajectory(actionName.c_str()) && success;
        }
    }

    result.success = success;
    performMovements_as.setSucceeded(result);

};
