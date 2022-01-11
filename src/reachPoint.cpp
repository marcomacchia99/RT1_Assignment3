#include "ros/ros.h"
#include "move_base_msgs/MoveBaseActionGoal.h"
#include "actionlib_msgs/GoalStatusArray.h"
#include "actionlib_msgs/GoalID.h"
#include <termios.h>
#include <time.h>

#define LENGTH 100

#define RED "\033[1;31m"
#define GREEN "\033[1;32m"
#define BLUE "\033[1;34m"
#define NC "\033[0m"
#define BOLD "\033[1m"

ros::Publisher publisher;
ros::Subscriber subscriber;

//variables for goal coordinates
float x, y;

//variable for goal id
int id;

//ariable for goal
move_base_msgs::MoveBaseActionGoal goal;

void getCoordinates();
void feedbackHandler(const actionlib_msgs::GoalStatusArray::ConstPtr &msg);

void getCoordinates()
{
    system("clear");
    std::cout << BOLD << "Robot automatic navigation \n\n"<<NC;

    //wait until two floats are inserted
    std::cout << "Insert "<<BOLD<<"x"<<NC<<" coordinate: ";
    std::cin >> x;
    while (!std::cin.good())
    {
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        system("clear");
        std::cout << BOLD << "Robot automatic navigation \n\n"<<NC;
        std::cout << "Insert "<<BOLD<<"x"<<NC<<" coordinate: ";
        std::cin >> x;
    }
    system("clear");
    std::cout << BOLD << "Robot automatic navigation \n\n"<<NC;

    std::cout << "Insert "<<BOLD<<"y"<<NC<<" coordinate: ";
    std::cin >> y;
    while (!std::cin.good())
    {
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        system("clear");
        std::cout << BOLD << "Robot automatic navigation \n\n"<<NC;
        std::cout << "Insert "<<BOLD<<"y"<<NC<<" coordinate: ";
        std::cin >> y;
    }
    id = rand();
    goal.goal.target_pose.pose.position.x = x;
    goal.goal.target_pose.pose.position.y = y;
    goal.goal.target_pose.pose.orientation.w = 1;
    goal.goal.target_pose.header.frame_id = "map";
    goal.goal_id.id = std::to_string(id);

    publisher.publish(goal);

    system("clear");

    std::cout << BOLD << "Robot automatic navigation \n"<<NC;

    std::cout << BLUE << "\nThe goal has been set to\tx: " << x << "\ty: " << y << NC << "\n\n";

    ros::NodeHandle node_handle;
    subscriber = node_handle.subscribe("/move_base/status", 500, feedbackHandler);
}

void feedbackHandler(const actionlib_msgs::GoalStatusArray::ConstPtr &msg)
{
    //status of goal
    int status = 0;

    //read only goals with correct id
    if (msg->status_list[0].goal_id.id == std::to_string(id))
        status = msg->status_list[0].status;

    if (status != 3 && status != 4)
    {
        return;
    }

    //stop receiving msg from subscribed topic
    subscriber.shutdown();

    if (status == 3) //status SUCCEDED

        std::cout
            << GREEN << "Goal reached!\n"
            << NC;
    else // status ABORTED
        std::cout
            << RED << "The goal can not be reached.\n"
            << NC;

    char answer;
    do
    {

        std::cout << "\nWould you like to set another goal? (y/n)\n";
        std::cin >> answer;

    } while (answer != 'y' && answer != 'n' && answer != 'Y' && answer != 'N' && answer != '\x03');
    if (answer == 'y' || answer == 'Y')
        getCoordinates();
    else
        exit(1);
}

int main(int argc, char **argv)
{
    //used to randomize id
    srand(time(NULL));

    ros::init(argc, argv, "reachPoint");
    ros::NodeHandle node_handle;

    publisher = node_handle.advertise<move_base_msgs::MoveBaseActionGoal>("/move_base/goal", 1);

    getCoordinates();

    ros::spin();

    return 0;
}