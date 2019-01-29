// wsn example program to illustrate LIDAR processing.  1/23/15

/*

Tyler Anderson update to change from single ping to sweep

Use teleop and/or reactive_commander for testing alarm

possible solution:
make a box around the robot, find coordinates where lidar pings intersect the box, and if radii are bigger
than intersection length, safe to move forward

a different approach would be a circuar arc in front of the robot with a box for the sides

*/

#include <ros/ros.h> //Must include this for all ROS cpp projects
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h> //Including the Float32 class from std_msgs
#include <std_msgs/Bool.h> // boolean message 

const double MIN_SAFE_DISTANCE = 0.50; // set alarm if anything is within 0.5m of the front of robot

// these values to be set within the laser callback
float ping_dist_in_front_ = 3.0; // global var to hold length of a SINGLE LIDAR ping--in front
int ping_index_front_ = -1; // NOT real; callback will have to find this
int ping_index_90deg_left = -1;
int ping_index_90deg_right = -1;
double angle_min_ = 0.0;
double angle_max_ = 0.0;
double angle_increment_ = 0.0;
double range_min_ = 0.0;
double range_max_ = 0.0;
int num_pings_ = -1;
double ping_angle_front_ = 0.0;
double ping_angle_left_ = 0.0;
double ping_angle_right_ = 0.0;
bool laser_alarm_ = false;


ros::Publisher lidar_alarm_publisher_;
ros::Publisher lidar_dist_publisher_;
// really, do NOT want to depend on a single ping.  Should consider a subset of pings
// to improve reliability and avoid false alarms or failure to see an obstacle

void laserCallback(const sensor_msgs::LaserScan& laser_scan) {
    if (ping_index_front_<0)  {
        //for first message received, set up the desired index of LIDAR range to eval
        angle_min_ = laser_scan.angle_min;
        angle_max_ = laser_scan.angle_max;
        angle_increment_ = laser_scan.angle_increment;
        range_min_ = laser_scan.range_min;
        range_max_ = laser_scan.range_max;

        num_pings_ = laser_scan.ranges.size();
        // what is the index of the ping that is straight ahead?
        // BETTER would be to use transforms, which would reference how the LIDAR is mounted;
        // but this will do for simple illustration
        ping_index_front_ = 1 + (int) ((0.0 - angle_min_)/angle_increment_);  //index of ping at angle_min is 0
        ping_index_90deg_left = 1 + (int) ((M_PI / 2.0 - angle_min_) / angle_increment_);
        ping_index_90deg_right = 1 + (int) ((-M_PI / 2.0 - angle_min_) / angle_increment_);

        ping_angle_front_ = ((angle_increment_ * ping_index_front_) + angle_min_);
        ping_angle_left_ = ((angle_increment_ * ping_index_90deg_left) + angle_min_);
        ping_angle_right_ = ((angle_increment_ * ping_index_90deg_right) + angle_min_);

        ROS_INFO("LIDAR setup: straight_ahead_ping_index = %d",ping_index_front_);       //333
        ROS_INFO("LIDAR setup: straight_left_ping_index = %d",ping_index_90deg_left);    //583
        ROS_INFO("LIDAR setup: straight_right_ping_index = %d",ping_index_90deg_right);  //84

        ROS_INFO("Number of pings = %d", num_pings_);                           //667 pings
        ROS_INFO("Min angle = %f", angle_min_);                                 //-2.094395 rad     
        ROS_INFO("Max angle = %f", angle_max_);                                 // 2.094395 rad     
        ROS_INFO("Angle increment = %f", angle_increment_);                     // 0.006289 rad
        ROS_INFO("Angle of straight ahead = %f", ping_angle_front_);            //0.000000
        ROS_INFO("Angle of straight left = %f", ping_angle_left_);              //1.572369
        ROS_INFO("Angle of straight right = %f", ping_angle_right_);            //-1.566079

    }

    laser_alarm_=false;

    int start_index = 1 + (int) ((-(M_PI/6.0) - angle_min_)/angle_increment_);  //250
    ROS_INFO("index of start = %d", start_index);  

    int end_index = 1 + (int) (((M_PI/6.0) - angle_min_)/angle_increment_);     //417
    ROS_INFO("index of start = %d", end_index);  

    //check for obstacles

    for(int i = start_index; i < end_index; i++){
        if(laser_scan.ranges[i] < MIN_SAFE_DISTANCE){
            ROS_WARN("MAIN/FRONT: DANGER, WILL ROBINSON!!");
            laser_alarm_=true;
        }
    }

    for(int j = ping_index_90deg_right; j < start_index; j++){
        if(laser_scan.ranges[j] < MIN_SAFE_DISTANCE/2.0){
            ROS_WARN("RIGHT: DANGER, WILL ROBINSON!!");
            laser_alarm_=true;
        }
    }

    for(int k = end_index; k < ping_index_90deg_left; k++){
        if(laser_scan.ranges[k] < MIN_SAFE_DISTANCE/2.0){
            ROS_WARN("LEFT: DANGER, WILL ROBINSON!!");
            laser_alarm_=true;
        }
    }

    /*
    ping_dist_in_front_ = laser_scan.ranges[ping_index_front_];
    ROS_INFO("ping dist in front = %f",ping_dist_in_front_);
    if (ping_dist_in_front_<MIN_SAFE_DISTANCE) {
        ROS_WARN("DANGER, WILL ROBINSON!!");
        laser_alarm_=true;
    }
    else {
        laser_alarm_=false;
    }
    */




    std_msgs::Bool lidar_alarm_msg;
    lidar_alarm_msg.data = laser_alarm_;
    lidar_alarm_publisher_.publish(lidar_alarm_msg);
    std_msgs::Float32 lidar_dist_msg;
    lidar_dist_msg.data = ping_dist_in_front_;
    lidar_dist_publisher_.publish(lidar_dist_msg);   
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "lidar_alarm"); //name this node
    ros::NodeHandle nh; 
    //create a Subscriber object and have it subscribe to the lidar topic
    ros::Publisher pub = nh.advertise<std_msgs::Bool>("lidar_alarm", 1);
    lidar_alarm_publisher_ = pub; // let's make this global, so callback can use it
    ros::Publisher pub2 = nh.advertise<std_msgs::Float32>("lidar_dist", 1);  
    lidar_dist_publisher_ = pub2;
    ros::Subscriber lidar_subscriber = nh.subscribe("robot0/laser_0", 1, laserCallback);
    ros::spin(); //this is essentially a "while(1)" statement, except it
    // forces refreshing wakeups upon new data arrival
    // main program essentially hangs here, but it must stay alive to keep the callback function alive
    return 0; // should never get here, unless roscore dies
}