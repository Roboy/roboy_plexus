#include "control/controlActions.hpp"

PerformMovementAction::PerformMovementAction(IcebusControlPtr icebusControl, string name) :
        icebusControl(icebusControl),
        performMovement_as(nh_, action_name, boost::bind(&PerformMovementAction::executeCB, this, _1), false),
        action_name(name)
{
    performMovement_as.start();
    ROS_INFO_STREAM( action_name + ": started action server");
}


void PerformMovementAction::executeCB(const roboy_control_msgs::PerformMovementGoalConstPtr &goal) {
    string file = icebusControl->trajectories_folder + goal->action;
    const char *fileName = file.c_str();
    result.success = icebusControl->PlayTrajectory(fileName);
    performMovement_as.setSucceeded(result);

};

PerformMovementsAction::PerformMovementsAction(IcebusControlPtr icebusControl, string name) :
        icebusControl(icebusControl),
        performMovements_as(nh_, action_name, boost::bind(&PerformMovementsAction::executeCB, this, _1), false),
        action_name(name)
{
    performMovements_as.start();
    ROS_INFO_STREAM( action_name + ": started action server");
}

void PerformMovementsAction::executeCB(const roboy_control_msgs::PerformMovementsGoalConstPtr &goal) {

    bool success;
    for (string actionName: goal->actions) {
        if (actionName.find("pause") != std::string::npos) {
            string delimiter = "_";
            int pause = stoi(actionName.substr(0, actionName.find(delimiter)));
            ros::Duration(pause).sleep();
            success = true && success;
        }
        else if (actionName.find("relax") != std::string::npos) {
            icebusControl->SetAllToDisplacement(0);
            success = true && success;
        }
        else {
            actionName = icebusControl->trajectories_folder + actionName;
            success = icebusControl->PlayTrajectory(actionName.c_str()) && success;
        }
    }

    result.success = success;
    performMovements_as.setSucceeded(result);

};