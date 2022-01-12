#include "ros/ros.h"
#include "std_srvs/Empty.h"

#define NC "\033[0m"
#define BOLD "\033[1m"

std_srvs::Empty reset;

std::string mainMenu = R"(
        
Select one of the following behavior:

1 - Reach autonomousely a given position
2 - Drive the robot with the keyboard
3 - Drive the robot with the keyboard with automatic collision avoidance
4 - Reset simulation

0 - Exit from the program


)";

int displayMainMenu()
{
    system("clear");

    std::cout << BOLD << "Welcome to the Mobile Robot Control System!" << NC;
    std::cout << mainMenu;

    int choice;

    std::cin >> choice;
    while (!std::cin.good())
    {
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        system("clear");
        std::cout << BOLD << "Welcome to the Mobile Robot Control System!" << NC;
        std::cout << mainMenu;
        std::cin >> choice;
    }

    return choice;
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "mainController");

    while (true)
    {

        switch (displayMainMenu())
        {
        case 1:
            system("rosrun RT1_Assignment3 reachPoint");
            break;
        case 2:
            system("rosrun RT1_Assignment3 driveWithKeyboard");
            break;
        case 3:
            system("rosrun RT1_Assignment3 driveWithKeyboardAssisted");
            break;
        case 4:
            ros::service::call("/gazebo/reset_simulation", reset);
            break;    
        case 0:
            system("clear");
            return 0;
            break;
        default:
            break;
        }
    }

    return 0;
}