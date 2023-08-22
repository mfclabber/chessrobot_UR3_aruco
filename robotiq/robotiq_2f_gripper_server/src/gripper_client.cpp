#include <ros/ros.h>
#include <robotiq_2f_gripper_server/Move.h>
#include <robotiq_2f_gripper_server/Status.h>
#include <robotiq_2f_gripper_server/Cmd.h>
#include <std_srvs/Empty.h>
#include <iostream>



int main(int argc, char **argv)
{
  // Initialize the ROS system and become a node.
  ros::init(argc, argv, "gripper_client");
  ros::NodeHandle nh;

  //create a reset gripper client
  ros::ServiceClient gripperResetClient = nh.serviceClient<std_srvs::Empty>("gripper/reset");

  //create an activate gripper client
  ros::ServiceClient gripperActivateClient = nh.serviceClient<std_srvs::Empty>("gripper/activate");

  //create an open gripper client
  ros::ServiceClient gripperOpenClient = nh.serviceClient<std_srvs::Empty>("gripper/open");

  //create a close gripper client
  ros::ServiceClient gripperCloseClient = nh.serviceClient<std_srvs::Empty>("gripper/close");

  //create a move gripper client
  ros::ServiceClient gripperMoveClient = nh.serviceClient<robotiq_2f_gripper_server::Move>("gripper/move");

  //create a status gripper client
  ros::ServiceClient gripperStatusClient = nh.serviceClient<robotiq_2f_gripper_server::Status>("gripper/status");

  std::cout<<"\nCalling Gripper services..." << std::endl;

  std::cout<<"\nCalling RESET..." << std::endl;
  std_srvs::Empty::Request vreq;
  std_srvs::Empty::Response vres;
  ros::service::waitForService("gripper/reset", ros::Duration(5));
  gripperResetClient.call(vreq,vres);
  ros::Duration(1.0).sleep();

  //Wait for user to press a key
  std::cout<<"\nPRESS a key to ACTIVATE the gripper" << std::endl;
  std::cin.get();
  std::cout<<"\nCalling ACTIVATE..." << std::endl;
  gripperActivateClient.call(vreq,vres);
  ros::Duration(3.0).sleep();

  int signe = -1;

  while(ros::ok()){

      //Wait for user to press a key
      std::cout<<"\nPRESS a key to start moving the gripper" << std::endl;
      std::cin.get();
      if(signe == -1) {
          std::cout<<"\nCalling CLOSE..." << std::endl;
          gripperCloseClient.call(vreq,vres);
          ros::Duration(1.0).sleep();
          signe = 1;
      }
      else{
          std::cout<<"\nCalling OPEN..." << std::endl;
          gripperOpenClient.call(vreq,vres);
          ros::Duration(1.0).sleep();
          signe = -1;
      }

      std::cout<<"\nCalling STATUS..." << std::endl;
      robotiq_2f_gripper_server::Status::Request sreq;
      robotiq_2f_gripper_server::Status::Response sres;
      gripperStatusClient.call(sreq,sres);
      ROS_INFO_STREAM(sres.gripperstatus);
      ros::Duration(1.0).sleep();
/*
    std::cout<<"\nCalling MOVE..." << std::endl;
    robotiq_2f_gripper_server::Move::Request mreq;
    robotiq_2f_gripper_server::Move::Response mres;
    mreq.apperture = 100;
    gripperMoveClient.call(mreq,mres);
    ros::Duration(1.0).sleep();
*/

    ros::spinOnce();
  }
}
