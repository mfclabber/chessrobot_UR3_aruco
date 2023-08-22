#include <kinenik/kinenik_ur.h>
#include <iostream>
#include <eigen3/Eigen/Geometry>

std::vector<double> deg2rad(std::vector<double> vect_angle_deg)
{
  std::vector<double> vect_angle_rad(6);

  for(size_t i=0; i<vect_angle_deg.size();i++)
  {
    vect_angle_rad[i] =  (vect_angle_deg[i]*M_PI)/180;
  }

  return vect_angle_rad;
}

std::vector<double> euler2quat(std::vector<double> vect_euler)
{

  Eigen::Quaternionf q = Eigen::AngleAxisf(vect_euler[0], Eigen::Vector3f::UnitX())
    * Eigen::AngleAxisf(vect_euler[1], Eigen::Vector3f::UnitY())
    * Eigen::AngleAxisf(vect_euler[2], Eigen::Vector3f::UnitZ());

  std::vector<double> quat(4);
  quat[0] = q.x();
  quat[1] = q.y();
  quat[2] = q.z();
  quat[3] = q.w();

  return quat;
}

int main(int argc, char** argv)
{

    KinenikUR myrobot("UR5");
    std::vector<JointPos> theta_sol;

    // std::cout << "IK passing delta and euler" << std::endl;
    // myrobot.solveIK(-0.227,-0.02165, -0.09131, -0.4703, -1.75, 300.97, theta_sol);
    // myrobot.printSolution(theta_sol);
    //
    // std::cout << "IK passing delta and quat" << std::endl;
    // myrobot.solveIK(-0.227,-0.02165, -0.09131,  -0.1709772, -0.6362112, 0.716192, 0.2303715, theta_sol);
    // myrobot.printSolution(theta_sol);

    std::cout << "IK passing delta and euler (1)" << std::endl;
    myrobot.solveIK(-0.09255, -0.0664, 0.40349, 1.0109, -0.3939, 1.2869, theta_sol);
    myrobot.printSolution(theta_sol);

    std::cout << "==============================" << std::endl;

    std::cout << "IK passing delta and quaternion (1)" << std::endl;
    std::vector<double> quat = euler2quat({1.0109, -0.3939, 1.2869});
    myrobot.solveIK(-0.09255, -0.0664, 0.40349, quat[0], quat[1], quat[2], quat[3], theta_sol);
    myrobot.printSolution(theta_sol);

    std::cout << "==============================" << std::endl;

    std::cout << "EXPECTED RESPONSE (1)!" << std::endl;
    std::vector<double> theta_expected_deg = {132.7, -44.2, 105.86, -130.7, -86.76, -152.5};
    std::vector<double> theta_expected_rad = deg2rad(theta_expected_deg);

    std::cout << "[" << theta_expected_rad[0] << ","
                     << theta_expected_rad[1] << ","
                     << theta_expected_rad[2] << ","
                     << theta_expected_rad[3] << ","
                     << theta_expected_rad[4] << ","
                     << theta_expected_rad[5] << "]" << std::endl;

    std::cout << "++++++++++++++++++++++++++++++" << std::endl;

    std::cout << "IK passing delta and euler (2)" << std::endl;
    myrobot.solveIK(-0.08014, 0.1954, 0.12574, -0.1714, -0.5368, 1.0164, theta_sol);
    myrobot.printSolution(theta_sol);

    std::cout << "==============================" << std::endl;

    std::cout << "IK passing delta and quaternion (2)" << std::endl;
    quat = euler2quat({-0.1714, -0.5368, 1.0164});
    myrobot.solveIK(-0.08014, 0.1954, 0.12574, quat[0], quat[1], quat[2], quat[3], theta_sol);
    myrobot.printSolution(theta_sol);

    std::cout << "==============================" << std::endl;

    std::cout << "EXPECTED RESPONSE (2)!" << std::endl;
    theta_expected_deg = {132.71, -84.57, 80.3, -130.69, -86.76, -152.5};
    theta_expected_rad = deg2rad(theta_expected_deg);

    std::cout << "[" << theta_expected_rad[0] << ","
                     << theta_expected_rad[1] << ","
                     << theta_expected_rad[2] << ","
                     << theta_expected_rad[3] << ","
                     << theta_expected_rad[4] << ","
                     << theta_expected_rad[5] << "]" << std::endl;


    std::cout << "++++++++++++++++++++++++++++++" << std::endl;


    std::cout << "IK passing delta and euler (3)" << std::endl;
    myrobot.solveIK(-0.22719, -0.02165, -0.09131, -0.4703, -1.7588, 1.9777, theta_sol);
    myrobot.printSolution(theta_sol);

    std::cout << "==============================" << std::endl;

    std::cout << "IK passing delta and quaternion (3)" << std::endl;
    quat = euler2quat({-0.4703, -1.7588, 1.9777});
    myrobot.solveIK(-0.22719, -0.02165, -0.09131, quat[0], quat[1], quat[2], quat[3], theta_sol);
    myrobot.printSolution(theta_sol);

    std::cout << "==============================" << std::endl;

    std::cout << "EXPECTED RESPONSE (3)!" << std::endl;
    theta_expected_deg = {42.78, -84.57, 80.3, -130.7, -86.76, -152.5};
    theta_expected_rad = deg2rad(theta_expected_deg);

    std::cout << "[" << theta_expected_rad[0] << ","
                     << theta_expected_rad[1] << ","
                     << theta_expected_rad[2] << ","
                     << theta_expected_rad[3] << ","
                     << theta_expected_rad[4] << ","
                     << theta_expected_rad[5] << "]" << std::endl;

    return EXIT_SUCCESS;
}
