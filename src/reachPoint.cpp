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

ros::Publisher pub_goal;
ros::Publisher pub_cancel;

ros::Subscriber subscriber;

//variables for goal coordinates
float x, y;

//variable for goal id
int id;

//flag used to detect the user input during goal navigation
int flag_goal_in_progress;

//variable for goal
move_base_msgs::MoveBaseActionGoal goal;

//function prototypes
void getCoordinates();
void feedbackHandler(const actionlib_msgs::GoalStatusArray::ConstPtr &msg);
void detectInput();

// For non-blocking keyboard inputs, taken from teleop_twist_keyboard.cpp
// at https://github.com/methylDragon/teleop_twist_keyboard_cpp/blob/master/src/teleop_twist_keyboard.cpp
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