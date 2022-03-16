#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include <termios.h>

#define LENGTH 140

#define RED "\033[1;31m"
#define BLUE "\033[1;34m"
#define NC "\033[0m"
#define BOLD "\033[1m"

ros::Publisher publisher;

//define variables for vel direction
int lin=0; //linear direction
int ang =0; //angular direction

//variables for vel speed
double speed = 0.5;
double turn_speed = 1;

//variable for wall distance trashold
double wall_th = 1;

//define variable for Twist
geometry_msgs::Twist vel;

void publishVel()
{
    //prepare new speed
    vel.angular.z = turn_speed * ang;
    vel.linear.x = speed * lin;

    //publish the new speed to the relative topic
    publisher.publish(vel);
}

/**  Returns the minumim value of a given array  */
double min(double a[])
{
    double min = 100;
    for (int i = 0; i < LENGTH; i++)
    {
        if (a[i] < min)
            min = a[i];
    }
    return min;
}

/** @brief Check all the visible walls.
 *
 * This function checks all the walls that the robot can see. It uses the robot scanner, and it checks
 * all the blocks that are in front of the robot, at his left and at his right.
 * If the robot is close to a wall, it makes him stop, otherwise it let him go.
 *
 * @param msg The message published into /scan topic
 *
 */
void checkWalls(const sensor_msgs::LaserScan::ConstPtr &msg)
{

    double left[LENGTH];
    double front[LENGTH];
    double right[LENGTH];

    //the last LENGTH element of the arrays refers to the walls on the left
    for (int i = 719 - LENGTH; i <= 719; i++)
    {
        left[i - (719 - LENGTH)] = msg->ranges[i];
    }

    //the mid-LENGTH element of the arrays refers to the walls in front of the robot
    for (int i = 360 - ((LENGTH / 2) + LENGTH % 2); i < 360 + ((LENGTH / 2) + LENGTH % 2); i++)
    {
        front[i - (360 - ((LENGTH / 2) + LENGTH % 2))] = msg->ranges[i];
    }

    //the first LENGTH element of the arrays refers to the walls on the right
    for (int i = 0; i < LENGTH; i++)
    {
        right[i] = msg->ranges[i];
    }

    //warn user
    if ((min(front) < wall_th && lin > 0) || ((min(left) < wall_th) && ang > 0) || ((min(right) < wall_th) && ang < 0))
        std::cout << RED << "Wall detected!\n"
                  << NC;

    //if the nearest wall in front of the robot is too close, the robot can only turn
    if (min(front) < wall_th && lin > 0)
    {
        lin = 0;
    }
    //if the nearest wall on the left or on the right is too close, the robot can only go straight
    if (((min(left) < wall_th) && ang > 0) || ((min(right) < wall_th) && ang < 0))
    {
        ang = 0;
    }

    //publish new speed
    publishVel();
}

// For non-blocking keyboard inputs, taken from teleop_twist_keyboard_cpp
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

    //publish new speed
    // publishVel();

    system("clear");
}

int main(int argc, char **argv)
{
    system("clear");
    ros::init(argc, argv, "driveWithKeyboardAssisted");
    ros::NodeHandle node_handle;

    //subribes to /scan topic
    ros::Subscriber subscriber = node_handle.subscribe("/scan", 500, checkWalls);

    //this ndoe will publish updated into /cmd_vel topic
    publisher = node_handle.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    ros::AsyncSpinner spinner(4);
    spinner.start();
    while (true)
        getCommand();
    spinner.stop();

    return 0;
}