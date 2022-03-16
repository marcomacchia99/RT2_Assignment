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

/**
* \file reachPoint.cpp
* \brief Node that sets and manages a goal for the robot
* \author Marco Macchia
* \version 1.0
* \date 16/03/2022
*
* \details
*
* Subscribes to: <BR>
*  /move_base/status
*
*
* Publishes to: <BR>
*  /move_base/goal
*  /move_base/cancel
*
*
* Description : <BR>
* This node let the user set a goal for the robot. A goal is a position that the robot should autonomousely reach.
* While the robot is navigating towards the final position, the user can at any time cancel the goal.
* The system automatically detects if the robot has reached the goal.
*
**/

ros::Publisher pub_goal; ///< publisher used to send the goal to the goal service
ros::Publisher pub_cancel; ///< publisher used to cancel the current goal sending a message to the goal service

ros::Subscriber subscriber; ///< subscriber used to receive the current goal status

float x; ///< goal x coordinate
float y; ///< goal y coordinate
 

int id; ///< current goal id

int flag_goal_in_progress; ///< flag used to detect the user input during goal navigation

move_base_msgs::MoveBaseActionGoal goal; ///< goal message

//function prototypes
void getCoordinates();
void feedbackHandler(const actionlib_msgs::GoalStatusArray::ConstPtr &msg);
void detectInput();

/**
 * @brief sets the read operation in non-blocking mode.
 * 
 * This function is used for non-blocking keyboard inputs. 
 * taken from teleop_twist_keyboard.cpp
 * at https://github.com/methylDragon/teleop_twist_keyboard_cpp/blob/master/src/teleop_twist_keyboard.cpp 
 * 
 * @return int - the pressed key 
 * 
 */
int getch(void)
{
    int ch;
    struct termios oldt;
    struct termios newt;

    // Store old settings, and copy to new settings
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;

    // Make required changes and apply the settings
    newt.c_lflag &= ~(ICANON | ECHO);
    newt.c_iflag |= IGNBRK;
    newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
    newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
    newt.c_cc[VMIN] = 1;
    newt.c_cc[VTIME] = 0;
    tcsetattr(fileno(stdin), TCSANOW, &newt);

    // Get the current character
    ch = getchar();

    // Reapply old settings
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

    return ch;
}


/** @brief Cancel the current goal.
 *
 * This function cancel the current goal and check if the user wants to set another goal or exit.
 *
 * @param must_continue if set to 0, the program exit without asking to continue
 *
 */
void cancelGoal(int must_continue)
{
    //build cancel message with goal id
    actionlib_msgs::GoalID msg_cancel;
    msg_cancel.id = std::to_string(id);
    pub_cancel.publish(msg_cancel);

    subscriber.shutdown();

    //set flag as goal in no more in progress
    flag_goal_in_progress = 0;

    //if the program should stop then return
    if (!must_continue)
        return;

    //else clear and ask user what he wants to do
    system("clear");

    std::cout << BOLD << "Robot automatic navigation \n\n"
              << NC;
    std::cout << RED << "Goal is cancelled. \n\n"
              << NC;

    std::cout << "Would you like to set another goal? (y/n)\n";

    //detect user input
    while (true)
        detectInput();
}


/** @brief Detect and validate a user input.
 *
 * This function check if the user is typing something. Depending on the status of the goal, only some inputs can be accepted.
 *
 *
 */
void detectInput()
{
    //catch an user input
    char input_char = getch();

    //if there is a goal in progress then user is allowed to cancel goal
    if ((input_char == 'q' || input_char == 'Q') && flag_goal_in_progress)
    {
        cancelGoal(1);
    }
    else if (input_char == '\x03' && flag_goal_in_progress) //CTRL-C
    {
        cancelGoal(0);
        system("clear");
        ros::shutdown();
        exit(0);
    }
    //if there aren't goal in progress then user is allowed to choose wheter to continue or not
    else if ((input_char == 'y' || input_char == 'Y' || input_char == 'n' || input_char == 'N') && !flag_goal_in_progress)
    {
        if (input_char == 'y' || input_char == 'Y')
            getCoordinates();
        else
            exit(1);
    }
}


/** @brief Get the new goal coordinate.
 *
 * This function gets the goal coordinates, create a new goal setting a custom id and publish the new goal.
 * It also subscribes the node to the /move_base/status topic
 *
 *
 */
void getCoordinates()
{
    system("clear");
    std::cout << BOLD << "Robot automatic navigation \n\n"
              << NC;

    //wait until two floats are inserted
    std::cout << "Insert " << BOLD << "x" << NC << " coordinate: ";
    std::cin >> x;
    while (!std::cin.good())
    {
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        system("clear");
        std::cout << BOLD << "Robot automatic navigation \n\n"
                  << NC;
        std::cout << "Insert " << BOLD << "x" << NC << " coordinate: ";
        std::cin >> x;
    }
    system("clear");
    std::cout << BOLD << "Robot automatic navigation \n\n"
              << NC;

    std::cout << "Insert " << BOLD << "y" << NC << " coordinate: ";
    std::cin >> y;
    while (!std::cin.good())
    {
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        system("clear");
        std::cout << BOLD << "Robot automatic navigation \n\n"
                  << NC;
        std::cout << "Insert " << BOLD << "y" << NC << " coordinate: ";
        std::cin >> y;
    }

    //build goal message, generating random id
    id = rand();
    goal.goal.target_pose.pose.position.x = x;
    goal.goal.target_pose.pose.position.y = y;
    goal.goal.target_pose.pose.orientation.w = 1;
    goal.goal.target_pose.header.frame_id = "map";
    goal.goal_id.id = std::to_string(id);

    //publish goal message
    pub_goal.publish(goal);

    //set flag as a new goal is in progress
    flag_goal_in_progress = 1;

    //update ui
    system("clear");

    std::cout << BOLD << "Robot automatic navigation \n"
              << NC;

    std::cout << BLUE << "\nThe goal has been correctly set to\tx: " << x << "\ty: " << y << NC << "\n\n";

    std::cout << "Press q to cancel the goal, or press CTRL-C to quit\n\n";

    //subscribe to goal status
    ros::NodeHandle node_handle;
    subscriber = node_handle.subscribe("/move_base/status", 500, feedbackHandler);

    //start detecting inputs
    ros::AsyncSpinner spinner(4);

    spinner.start();
    while (true)
        detectInput();

    spinner.stop();
}


/** @brief Get the goal status.
 *
 * This function check the current goal status. If the robot has reached the goal or the robot can't reach it, the functions asks for a new goal.
 * If the robot is still trying to reach the goal it does nothing.
 *
 * @param msg the message received from the /move_base/status topic, containing the current goal status
 *
 */
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

    //no more need of detect input
    flag_goal_in_progress = 0;

    //update UI
    system("clear");

    std::cout << BOLD << "Robot automatic navigation \n\n"
              << NC;

    if (status == 3) //status SUCCEDED

        std::cout
            << GREEN << "Goal reached!\n"
            << NC;
    else // status ABORTED
        std::cout
            << RED << "The goal can not be reached.\n"
            << NC;

    std::cout << "\nWould you like to set another goal? (y/n)\n";
}

/**
 * @brief main function
 * 
 * The main function starts the current node and advertise that this node will publish into /cmd_vel and /move_base/cancel topics.
 * It then call the getCoordinates() function.
 * 
 * @param argc
 * @param argv
 * 
 * @return always 0
 * 
 */
int main(int argc, char **argv)
{
    //used to randomize id
    srand(time(NULL));

    ros::init(argc, argv, "reachPoint");
    ros::NodeHandle node_handle;

    //this node will send messages to goal and cancel(goal)
    pub_goal = node_handle.advertise<move_base_msgs::MoveBaseActionGoal>("/move_base/goal", 1);
    pub_cancel = node_handle.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 1);

    //get goal coordinates
    getCoordinates();

    ros::spin();

    return 0;
}