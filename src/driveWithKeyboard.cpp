#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include <termios.h>

#define BLUE "\033[1;34m"
#define NC "\033[0m"
#define BOLD "\033[1m"

/**
* \file driveWithKeyboard.cpp
* \brief Controller for the robot
* \author Marco Macchia
* \version 1.0
* \date 16/03/2022
*
* \details
*
*
* Publishes to: <BR>
*  /cmd_vel
*
*
* Description : <BR>
* This node let the user drive the robot using the keyboard. According to what the user want it sets
* the linear and angular velocity of the robot.
*
**/

ros::Publisher publisher; ///< publisher used to send the velocities to the robot

//define variables for vel direction
int lin=0; ///< linear robot direction
int ang =0; ///< angular robot direction

//define variables for vel speed
double speed = 0.5; ///< robot linear speed
double turn_speed = 1; ///< robot angular speed

//define variable for Twist
geometry_msgs::Twist vel; ///< velocity message

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

/**
 * @brief 
 * This function translate the user input in robot command
 * 
 * @param inputChar the key pressed by the user
 * 
 */
void interpretInput(char inputChar)
{
    //set linear and angular direction according to user input
    switch (inputChar)
    {
    case 'u':
        lin = 1;
        ang = 1;
        break;
    case 'i':
        lin = 1;
        ang = 0;
        break;
    case 'o':
        lin = 1;
        ang = -1;
        break;
    case 'j':
        lin = 0;
        ang = 1;
        break;
    case 'k':
        lin = 0;
        ang = 0;
        break;
    case 'l':
        lin = 0;
        ang = -1;
        break;
    case 'm':
        lin = -1;
        ang = 1;
        break;
    case ',':
        lin = -1;
        ang = 0;
        break;
    case '.':
        lin = -1;
        ang = -1;
        break;
    case 'q':
        speed *= 1.1;
        turn_speed *= 1.1;
        break;
    case 'z':
        speed *= 0.9;
        turn_speed *= 0.9;
        break;
    case 'w':
        speed *= 1.1;
        break;
    case 'x':
        speed *= 0.9;
        break;
    case 'e':
        turn_speed *= 1.1;
        break;
    case 'c':
        turn_speed *= 0.9;
        break;
    case 'a':
        speed = 0.5;
        turn_speed = 1;
        break;
    case 's':
        speed = 0.5;
        break;
    case 'd':
        turn_speed = 1;
        break;
    case '\x03': //CTRL-C

        //stop robot and exit
        vel.angular.z = 0;
        vel.linear.x = 0;
        publisher.publish(vel);
        system("clear");
        ros::shutdown();
        exit(0);
        break;
    default:
        //stop robot
        lin = 0;
        ang = 0;
        break;
    }
}


/**
 * @brief 
 * This function prints the instructions, waits for a command and then sends the new velocities to the robot 
 * 
 * 
 */
void getCommand()
{
    char input_char;

    //display instructions
    std::cout << BOLD << "Drive the robot using your keyboard\n\n"
              << NC;

    std::cout << R"(Use the commands below as a joystick

   u    i    o
   j    k    l
   m    ,    .

anything else will stop the robot.

Press:
q/z : increase/decrease all speeds by 10%
a   : reset all speeds
w/x : increase/decrease only linear speed by 10%
s   : reset only linear speed
e/c : increase/decrease only angular speed by 10%
d   : reset only angular speed

press CTRL-C to quit
)";
    std::cout << BLUE << "\ncurrently:\tspeed " << speed << "\tturn " << turn_speed << "\n"
              << NC;

    //wait for user input
    input_char = getch();

    interpretInput(input_char);

    //prepare new speed
    vel.angular.z = turn_speed * ang;
    vel.linear.x = speed * lin;

    //publish the new speed to the relative topic
    publisher.publish(vel);

    system("clear");
}

/**
 * @brief main function
 * 
 * The main function starts the current node and advertise that this node will publish into /cmd_vel topic.
 * It continuosely checks the user inputs
 * 
 * @param argc
 * @param argv
 * 
 * @return always 0
 * 
 */
int main(int argc, char **argv)
{
    system("clear");
    ros::init(argc, argv, "driveWithKeyboard");
    ros::NodeHandle node_handle;

    //this node will publish updated into /cmd_vel topic
    publisher = node_handle.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    while (true)
        getCommand();

    return 0;
}