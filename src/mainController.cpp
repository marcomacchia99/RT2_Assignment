#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include <termios.h>

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

int displayMainMenu()
{
    system("clear");

    std::cout << BOLD << "Welcome to the Mobile Robot Control System!" << NC;
    std::cout << mainMenu;

    int choice;

    choice = getch();
    // while (!std::cin.good())
    // {
    //     std::cin.clear();
    //     std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    //     system("clear");
    //     std::cout << BOLD << "Welcome to the Mobile Robot Control System!" << NC;
    //     std::cout << mainMenu;
    //     std::cin >> choice;
    // }

    return choice;
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "mainController");

    while (true)
    {

        switch (displayMainMenu())
        {
        case '1':
            //launch reachPoint node
            system("rosrun RT1_Assignment3 reachPoint");
            break;
        case '2':
            //launch driveWithKeyboard node
            system("rosrun RT1_Assignment3 driveWithKeyboard");
            break;
        case '3':
            //launch driveWithKeyboardAssisted node
            system("rosrun RT1_Assignment3 driveWithKeyboardAssisted");
            break;
        case '4':
            //call gazebo/reset_simulation service and reset the simulation
            ros::service::call("/gazebo/reset_simulation", reset);
            break;
        case '0':
            //celar screen and exit from main
            system("clear");
            return 0;
            break;
        default:
            break;
        }
    }

    return 0;
}