#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <sensor_msgs/JointState.h>

// This is a general server that wraps the call to the FollowJointTrajectory actions
// All the services offered are general, except the one that sets a rectilinear trajectory in the Cartsian spac,
// that is particularized for the UR3 robot.

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> Client;

control_msgs::FollowJointTrajectoryGoal goal;
actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> *robotClient;


uint numjoints = 6;
std::vector<std::string> jointnames;
std::vector<double> currentjoints;

bool updatedstates = false;


//Callback function: Called once when the goal completes
void doneCb(const actionlib::SimpleClientGoalState& state,
            const control_msgs::FollowJointTrajectoryResultConstPtr& result)
{
  ROS_INFO("Finished in state [%s]", state.toString().c_str());
  ROS_INFO("Answer: error_code is %d", result->error_code);
  //error_string is not showing the message...
  //see http://docs.ros.org/api/control_msgs/html/action/FollowJointTrajectory.html for the meaning of the error codes.
  //ROS_INFO_STREAM("Answer: error_string is "<< result->error_string);
}

//Callback function: Called once when the goal becomes active
void activeCb()
{
  ROS_INFO("Goal just went active");
}

//Callback function: Called every time feedback is received for the goal
void feedbackCb(const control_msgs::FollowJointTrajectoryFeedbackConstPtr& feedback)
{
  std::stringstream feedback_message_names;
  feedback_message_names << "Got Feedback of joints (";
  for(int i=0;i<numjoints-1;i++)
      feedback_message_names << feedback->joint_names[i] << ",";
  feedback_message_names << feedback->joint_names[numjoints-1] << ")";
  ROS_INFO_STREAM(feedback_message_names.str());

  std::stringstream feedback_message_pos;
  feedback_message_pos << "              current positions are (";
  for(int i=0;i<numjoints-1;i++)
      feedback_message_pos << feedback->actual.positions[i] << ",";
  feedback_message_pos << feedback->actual.positions[numjoints-1] << ")";
  ROS_INFO_STREAM(feedback_message_pos.str());

  std::stringstream feedback_message_vel;
  feedback_message_vel << "              current velocities are (";
  for(int i=0;i<numjoints-1;i++)
      feedback_message_vel << feedback->actual.velocities[i] << ",";
  feedback_message_vel << feedback->actual.velocities[numjoints-1] << ")";
  ROS_INFO_STREAM(feedback_message_vel.str());
}


//Function to send the goal to the FollowJointTrajectory action server.
//Waits for the result for trajduration seconds.
//If not able to reach the goal within timeout, it is cancelled
bool moveRobotTrajectory(double trajduration)
{
    ROS_INFO("Moving robot in %f", trajduration);

    //Print the trajectory to be followed
    ROS_INFO("currentjoints: (%f,%f,%f,%f,%f,%f)",
                  currentjoints[0],
                  currentjoints[1],
                  currentjoints[2],
                  currentjoints[3],
                  currentjoints[4],
                  currentjoints[5]);
    for(int i=0; i < goal.trajectory.points.size(); i++){
         ROS_INFO("%d: (%f,%f,%f,%f,%f,%f)",i,
                  goal.trajectory.points[i].positions[0],
                  goal.trajectory.points[i].positions[1],
                  goal.trajectory.points[i].positions[2],
                  goal.trajectory.points[i].positions[3],
                  goal.trajectory.points[i].positions[4],
                  goal.trajectory.points[i].positions[5]);
    }


    //Set timestamp and send goal
    goal.trajectory.header.stamp = ros::Time::now() ;//+ ros::Duration(1.0); //To cHECK the +1
    robotClient->sendGoal(goal, &doneCb, &activeCb, &feedbackCb);

    //Wait for the action to return. Timeout set to the req.trajduration plus the goal time tolerance)
    bool finished_before_timeout = robotClient->waitForResult(ros::Duration(trajduration) + goal.goal_time_tolerance);

    //Get final state
    actionlib::SimpleClientGoalState state = robotClient->getState();
    if (finished_before_timeout) {
        //Reports ABORTED if finished but goal not reached. Cause shown in error_code in doneCb callback
        //Reports SUCCEEDED if finished and goal reached
        ROS_INFO(" ***************** Robot action finished: %s  *****************",state.toString().c_str());
    } else {
        //Reports ACTIVE if not reached within timeout. Goal must be cancelled if we want to stop the motion, then the state will report PREEMPTED.
        ROS_ERROR("Robot action did not finish before the timeout: %s",
                state.toString().c_str());
        //Preempting task
        ROS_ERROR("I am going to preempt the task...");
        robotClient->cancelGoal();
    }
    //force to reset the traj before calling again move
    return finished_before_timeout;
}

//Function to set a rectilinear trajectory
// deltaconf is used to define the goal: goal=currentjoints+deltaconf
// epsilon is the step size in joint space
// timefromstart defines the total trajectory time
void setRectilinearTrajectory(std::vector<double>& newconf, double timefromstart)
{
        ROS_INFO("Setting new rectilinear trajectory ");

        //set joint names
        goal.trajectory.joint_names.resize(numjoints);
        goal.trajectory.joint_names[0] = "team_A_shoulder_pan_joint";
        goal.trajectory.joint_names[1] = "team_A_shoulder_lift_joint";
        goal.trajectory.joint_names[2] = "team_A_elbow_joint";
        goal.trajectory.joint_names[3] = "team_A_wrist_1_joint";
        goal.trajectory.joint_names[4] = "team_A_wrist_2_joint";
        goal.trajectory.joint_names[5] = "team_A_wrist_3_joint";
        goal.trajectory.points.resize( 1 );
        //goal point
        goal.trajectory.points[0].positions.resize( numjoints );
        goal.trajectory.points[0].velocities.resize( numjoints );
        goal.trajectory.points[0].accelerations.resize( numjoints );
        for(int j=0; j < numjoints; j++){
            goal.trajectory.points[0].positions[j] = newconf[j];
            goal.trajectory.points[0].velocities[j] = 0.0;
            goal.trajectory.points[0].accelerations[j] = 0.0;
        }
        goal.trajectory.points[0].time_from_start = ros::Duration(timefromstart);
}


// A callback function to update the arm joint values - gripper joint is skipped
void updateCurrentJointStates(const sensor_msgs::JointState& msg) {
  for(int i=0; i<msg.name.size();i++)
  {
    //BE CAREFUL: order of joints is set according to the controller
    if(msg.name[i] == "team_A_shoulder_pan_joint") currentjoints[0]= msg.position[i];
    else if(msg.name[i] == "team_A_shoulder_lift_joint") currentjoints[1]= msg.position[i];
    else if(msg.name[i] == "team_A_elbow_joint") currentjoints[2]= msg.position[i];
    else if(msg.name[i] == "team_A_wrist_1_joint") currentjoints[3]= msg.position[i];
    else if(msg.name[i] == "team_A_wrist_2_joint") currentjoints[4]= msg.position[i];
    else if(msg.name[i] == "team_A_wrist_3_joint") currentjoints[5]= msg.position[i];

    /*
    std::stringstream ss;
    for(int j=0; j<6;j++)
      ss<<currentjoints[j]<<" ";
    ROS_INFO_STREAM(ss.str());
    */
  }
  updatedstates = true;
}


int main(int argc, char **argv)
{
  // Initialize the ROS system and become a node.
  ros::init(argc, argv, "ur3_action_client");
  ros::NodeHandle nh;

  // Create a subscriber object.
  ros::Subscriber sub = nh.subscribe("/team_A_arm/joint_states", 1000, &updateCurrentJointStates);

  // Initialization
  currentjoints.resize(numjoints);
  robotClient = new Client("/team_A_arm/joint_trajectory_controller/follow_joint_trajectory");
  if(!robotClient->waitForServer(ros::Duration(5.0)))
  {
      ROS_ERROR(" *** action server not available *** ");
      exit(0);
  };

  // Wait for user to press a key
  std::cout<<"\nMOVE THE ROBOT TO A SAVE HOME CONFIGURATION AND PRESS A KEY TO START..."<<std::endl;
  std::cin.get();
  std::vector<double> newconf;
  newconf.resize(6);
  int signe = -1;
  double trajduration = 5;

  while(ros::ok()) {
    ros::spinOnce();
    //Wait for user to press a key
    std::cout<<"\nPRESS a key to start a rectilinear joint-space test motion" << std::endl;
    std::cin.get();
    if(signe == -1) signe = 1;
    else signe = -1;
    newconf[0] =  currentjoints[0] + signe*0.3;//pan
    newconf[1] =  currentjoints[1] + signe*0.0;
    newconf[2] =  currentjoints[2] + signe*0.3;//elbow
    newconf[3] =  currentjoints[3] + signe*0.0;
    newconf[4] =  currentjoints[4] + signe*0.0;
    newconf[5] =  currentjoints[5] + signe*0.0;
    if(updatedstates==false) continue;
    setRectilinearTrajectory(newconf, trajduration);
    moveRobotTrajectory(trajduration);
  }

}
