#include <kinenik/kinenik_ur.h>
#include <iostream>
#include <string>

int main(int argc, char** argv)
{
    std::string data;

    if(argc != 2){
        std::cerr << "test_kinenik_ur: This program need a UR type to test" << std::endl;
        std::cerr << "$ test_kinenik_ur [UR_TYPE]" << std::endl;
        std::cerr << "where [UR_TYPE] could be: UR3, UR5, UR10, UR3e, UR5e, UR10e, UR16e" << std::endl;
        return EXIT_FAILURE;
    }
    std::string type(argv[1]);
    if(type.compare("UR3") != 0)
        if(type.compare("UR5") != 0)
            if(type.compare("UR10") != 0)
                if(type.compare("UR3e") != 0)
                    if(type.compare("UR5e") != 0)
                        if(type.compare("UR10e") != 0)
                            if(type.compare("UR10e") != 0){
                                std::cerr << "UR_TYPE: " << type << std::endl;
                                std::cerr << "UR_TYPE must be one of this types: UR3, UR5, UR10, UR3e, UR5e, UR10e, UR16e" << std::endl;
                                return EXIT_FAILURE;
                            }
    KinenikUR myrobot(type);
    std::vector<JointPos> theta_sol;
    std::vector<double> values;

    for(;;)
    {
        std::cout << "IK passing delta and quaternion. Write 7 values separates by comma" << std::endl;
        std::cout << "Example: 0.145, -0.378, 0.505, 0.470, 0.058, 0.556, 0.683" << std::endl;
        values.resize(7);
        getline(std::cin,data);
        std::istringstream iss(data);
        std::string s;
        unsigned int i = 0;
        while (getline(iss, s,','))
        {
            if (i >= values.size())
            {
                std::cout << "The number of positions can not be more than 7" << std::endl;
                break;
            }
            try {
                  values[i++] = stod(s);
            } catch(...) {
                  std::cout << "Invalid value" << std::endl;
                  break;
            }
        }
        std::cout << "Values introduced: x, y, z: " << values[0]<< ", "<< values[1] << ", " << values[2] << std::endl;
        std::cout << "Values introduced: qx, qy, qz, qw: " << values[3] << ", " << values[4] << ", " << values[5] << ", " << values[6] << std::endl;
        myrobot.solveIK(values[0], values[1], values[2], values[3],  values[4], values[5], values[6], theta_sol);
        myrobot.printSolution(theta_sol);

    }
    return EXIT_SUCCESS;
}
