#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <robotiq_2f_gripper_server/Move.h>
#include <robotiq_2f_gripper_server/Status.h>
#include <robotiq_2f_gripper_server/Cmd.h>
#include <robotiq_2f_gripper_control/Robotiq2FGripper_robot_input.h>
#include <iostream>

ros::Publisher *pubPtr;
bool activated;
robotiq_2f_gripper_control::Robotiq2FGripper_robot_input status;

void statusReceived(const robotiq_2f_gripper_control::Robotiq2FGripper_robot_input &msgIn)
{
  status.gACT = msgIn.gACT;
  status.gGTO = msgIn.gGTO;
  status.gSTA = msgIn.gSTA;
  status.gOBJ = msgIn.gOBJ;
  status.gFLT = msgIn.gFLT;
  status.gPR = msgIn.gPR;
  status.gPO = msgIn.gPO;
  status.gCU = msgIn.gCU;
}

bool gripper_status(robotiq_2f_gripper_server::Status::Request  &req,
         robotiq_2f_gripper_server::Status::Response &res)
{
  ROS_INFO("Gripper Status Service");
  std::stringstream output;

  //gACT
  output << "gACT = " + std::to_string(status.gACT) + ": ";
  if(status.gACT == 0)
      output << "Gripper reset\n";
  if(status.gACT == 1)
      output << "Gripper activation\n";

  //gGTO
  output << "gGTO = " + std::to_string(status.gGTO) + ": ";
  if(status.gGTO == 0)
      output << "Standby (or performing activation/automatic release)\n";
  if(status.gGTO == 1)
      output << "Go to Position Request\n";

  //gSTA
  output << "gSTA = " + std::to_string(status.gSTA) + ": ";
  if(status.gSTA == 0)
      output << "Gripper is in reset ( or automatic release ) state. see Fault Status if Gripper is activated\n";
  if(status.gSTA == 1)
      output << "Activation in progress\n";
  if(status.gSTA == 2)
      output << "Not used\n";
  if(status.gSTA == 3)
      output << "Activation is completed\n";

  //gOBJ
  output << "gOBJ = " + std::to_string(status.gOBJ) + ": ";
  if(status.gOBJ == 0)
      output << "Fingers are in motion (only meaningful if gGTO = 1)\n";
  if(status.gOBJ == 1)
      output << "Fingers have stopped due to a contact while opening\n";
  if(status.gOBJ == 2)
      output << "Fingers have stopped due to a contact while closing \n";
  if(status.gOBJ == 3)
      output << "Fingers are at requested position\n";

  //gFLT
  output << "gFLT = " + std::to_string(status.gFLT) + ": ";
  if(status.gFLT == 0x00)
      output << "No Fault\n";
  if(status.gFLT == 0x05)
      output << "Priority Fault: Action delayed, initialization must be completed prior to action\n";
  if(status.gFLT == 0x07)
      output << "Priority Fault: The activation bit must be set prior to action\n";
  if(status.gFLT == 0x09)
      output << "Minor Fault: The communication chip is not ready (may be booting)\n";
  if(status.gFLT == 0x0B)
      output << "Minor Fault: Automatic release in progress\n";
  if(status.gFLT == 0x0E)
      output << "Major Fault: Overcurrent protection triggered\n";
  if(status.gFLT == 0x0F)
      output << "Major Fault: Automatic release completed\n";

  //gPR
  output << "gPR = " + std::to_string(status.gPR) + ": ";
  output << "Echo of the requested position for the Gripper: " + std::to_string(status.gPR) + "/255\n";

  //gPO
  output << "gPO = " + std::to_string(status.gPO) + ": ";
  output << "Position of Fingers: " + std::to_string(status.gPO) + "/255\n";

  //gCU
  output << "gCU = " + std::to_string(status.gCU) + ": ";
  output << "Current of Fingers: " + std::to_string(status.gCU * 10) + " mA\n";

  ROS_INFO_STREAM(output.str());
  res.gripperstatus=status;
  return true;
}


bool gripper_activate(std_srvs::Empty::Request  &req,
         std_srvs::Empty::Response &res)
{
  ROS_INFO("Gripper Activate Service");
      //info from Robotiq2FGripperSimpleController.py
      //command.rACT = 1
      //command.rGTO = 1
      //command.rSP = 255
      //command.rFR = 150
      robotiq_2f_gripper_control::Robotiq2FGripper_robot_output msgGripperCmd;
      msgGripperCmd.rACT = 1;
      msgGripperCmd.rGTO = 1;
      msgGripperCmd.rATR = 0;
      msgGripperCmd.rPR = 0;
      msgGripperCmd.rSP = 255;//speed
      msgGripperCmd.rFR = 150;//force
      pubPtr->publish(msgGripperCmd);
      activated = true;
  return true;
}

bool gripper_reset(std_srvs::Empty::Request  &req,
         std_srvs::Empty::Response &res)
{
  ROS_INFO("Gripper Reset Service");
  robotiq_2f_gripper_control::Robotiq2FGripper_robot_output msgGripperCmd;
      //info from Robotiq2FGripperSimpleController.py
      //command.rACT = 0
  msgGripperCmd.rACT = 0;
  msgGripperCmd.rACT = 0;
  msgGripperCmd.rGTO = 0;
  msgGripperCmd.rATR = 0;
  msgGripperCmd.rPR = 0;
  msgGripperCmd.rSP = 0;//speed
  msgGripperCmd.rFR = 0;//force
  pubPtr->publish(msgGripperCmd);
  activated = false;
  return true;
}

bool gripper_close(std_srvs::Empty::Request  &req,
         std_srvs::Empty::Response &res)
{
  ROS_INFO("Gripper Close Service");
  if(activated){
      robotiq_2f_gripper_control::Robotiq2FGripper_robot_output msgGripperCmd;
      //info from Robotiq2FGripperSimpleController.py
      //command.rPR = 255
      msgGripperCmd.rACT = 1;
      msgGripperCmd.rGTO = 1;
      msgGripperCmd.rATR = 0;
      msgGripperCmd.rPR = 255;
      msgGripperCmd.rSP = 255;//speed
      msgGripperCmd.rFR = 150;//force
      pubPtr->publish(msgGripperCmd);
  }
  else{
      ROS_INFO("Not done - Gripper should be first activated");
  }
  return true;
}

bool gripper_open(std_srvs::Empty::Request  &req,
         std_srvs::Empty::Response &res)
{
  ROS_INFO("Gripper Open Service");
  if(activated){
      robotiq_2f_gripper_control::Robotiq2FGripper_robot_output msgGripperCmd;
      //info from Robotiq2FGripperSimpleController.py
      //command.rPR = 0
      msgGripperCmd.rACT = 1;
      msgGripperCmd.rGTO = 1;
      msgGripperCmd.rATR = 0;
      msgGripperCmd.rPR = 0;
      msgGripperCmd.rSP = 255;//speed
      msgGripperCmd.rFR = 150;//force
      pubPtr->publish(msgGripperCmd);
  }
  else{
      ROS_INFO("Not done - Gripper should be first activated");
  }
  return true;
}

bool gripper_move(robotiq_2f_gripper_server::Move::Request  &req,
         robotiq_2f_gripper_server::Move::Response &res)
{
  ROS_INFO("Gripper Open Service");
  if(activated){
      robotiq_2f_gripper_control::Robotiq2FGripper_robot_output msgGripperCmd;
      //info from Robotiq2FGripperSimpleController.py
      //command.rPR = 0
      msgGripperCmd.rACT = 1;
      msgGripperCmd.rGTO = 1;
      msgGripperCmd.rATR = 0;
      msgGripperCmd.rPR = req.apperture;//apperture 0:open 255:closed
      msgGripperCmd.rSP = 255;//speed
      msgGripperCmd.rFR = 150;//force
      pubPtr->publish(msgGripperCmd);
  }
  else{
      ROS_INFO("Not done - Gripper should be first activated");
  }
  return true;
}


bool gripper_cmd(robotiq_2f_gripper_server::Cmd::Request  &req,
         robotiq_2f_gripper_server::Cmd::Response &res)
{
  ROS_INFO("Gripper Move Service");
  robotiq_2f_gripper_control::Robotiq2FGripper_robot_output msgGripperCmd;
      //info from Robotiq2FGripperSimpleController.py
  msgGripperCmd.rACT = req.grippercmd.rACT;
  msgGripperCmd.rGTO = req.grippercmd.rGTO;
  msgGripperCmd.rATR = req.grippercmd.rATR;
  msgGripperCmd.rPR = req.grippercmd.rPR;
  msgGripperCmd.rSP = req.grippercmd.rSP;
  msgGripperCmd.rFR = req.grippercmd.rFR;
  pubPtr->publish(msgGripperCmd);
  return true;
}

int main(int argc, char **argv)
{
  // Initialize the ROS system and become a node.
  ros::init(argc, argv, "gripper_server");
  ros::NodeHandle nh;

  //create publisher
  pubPtr = new ros::Publisher(nh.advertise<robotiq_2f_gripper_control::Robotiq2FGripper_robot_output>("/Robotiq2FGripperRobotOutput",1000));

  //Create a subscriber object.
  ros::Subscriber sub = nh.subscribe("Robotiq2FGripperRobotInput", 1000, &statusReceived);

  // Create a service to get the status of the gripper
  ros::ServiceServer serviceGripperStatus = nh.advertiseService("gripper/status", &gripper_status);

  // Create a service to activate the gripper
  ros::ServiceServer serviceGripperActivate = nh.advertiseService("gripper/activate", &gripper_activate);

  // Create a service to reset the gripper
  ros::ServiceServer serviceGripperReset = nh.advertiseService("gripper/reset", &gripper_reset);

  // Create a service to close the gripper
  ros::ServiceServer serviceGripperClose = nh.advertiseService("gripper/close", &gripper_close);

  // Create a service to open the gripper
  ros::ServiceServer serviceGripperOpen = nh.advertiseService("gripper/open", &gripper_open);

  // Create a service to move the gripper
  ros::ServiceServer serviceGripperMove = nh.advertiseService("gripper/move", &gripper_move);

  // Create a service to send a complete command to the gripper
  ros::ServiceServer serviceGripperCmd = nh.advertiseService("gripper/cmd", &gripper_cmd);

  activated = false;
  std::cout<<"\nGripper services ON" << std::endl;
  ros::spin();

}
