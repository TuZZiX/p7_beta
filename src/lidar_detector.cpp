/* wsn example program to illustrate LIDAR processing.  1/23/15 */
/* modified by shipei tian*/

#include <ros/ros.h>                    /* Must include this for all ROS cpp projects */
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>              /* boolean message */
#include <math.h>

const double MIN_SAFE_DISTANCE = 2.0;   /* set alarm if anything is within 0.5m of the front of robot */
const double DISTANCE_FILTER = 0.15;

/* these values to be set within the laser callback */
int	    ping_index_		    = -1;   /* NOT real; callback will have to find this */
double	angle_min_		    = 0.0;
double	angle_max_		    = 0.0;
double	angle_increment_    = 0.0;
double	range_min_		    = 0.0;
double	range_max_		    = 0.0;

ros::Publisher	lidar_alarm_publisher_;
ros::Publisher	opt_dir_publisher_;
/*
 * really, do NOT want to depend on a single ping.  Should consider a subset of pings
 * to improve reliability and avoid false alarms or failure to see an obstacle
 */

void laserCallback( const sensor_msgs::LaserScan & laser_scan )
{
    bool avail = false;
    bool all_max = true;
    int min_dis = -1;
    int max_dis = -1;
    double opt_dir = 0;

    int opt_pin = -1;
    int opt_pin_num = -1;

    std_msgs::Float64 opt_dir_msg;
    std_msgs::Bool lidar_alarm_msg;

    if ( ping_index_ < 0 )
    {
        /* for first message received, set up the desired index of LIDAR range to eval */
        angle_min_		= laser_scan.angle_min;
        angle_max_		= laser_scan.angle_max;
        angle_increment_= laser_scan.angle_increment;
        range_min_		= laser_scan.range_min;
        range_max_		= laser_scan.range_max - 1.0;
        //range_max_		= 30.0;
        /*
         * what is the index of the ping that is straight ahead?
         * BETTER would be to use transforms, which would reference how the LIDAR is mounted;
         * but this will do for simple illustration
         */
        ping_index_ = (int) (((angle_max_ - angle_min_) / 2) / angle_increment_);
        ROS_INFO( "LIDAR setup: ping_index = %d", ping_index_ );
    }
    for (int j = 0; j < ping_index_*2; ++j) {
        if (laser_scan.ranges[j] <= range_max_ && laser_scan.ranges[j] >= DISTANCE_FILTER) {
            all_max = false;
        }
    }
    if (all_max) {
        lidar_alarm_msg.data = false;
        lidar_alarm_publisher_.publish( lidar_alarm_msg );

        opt_dir = 0.0;
        ROS_INFO("You have enter an empty world");
        opt_dir_msg.data = opt_dir;
        opt_dir_publisher_.publish( opt_dir_msg );
        return;
    }



    for (int l = 0; l < ping_index_*2; ++l) {
        if (laser_scan.ranges[l] <= MIN_SAFE_DISTANCE && laser_scan.ranges[l] >= DISTANCE_FILTER) {
            if (min_dis < 0) {
                min_dis = l;
            } else {
                if (laser_scan.ranges[l] < laser_scan.ranges[min_dis]) {
                    min_dis = l;
                }
            }
        }
    }
    lidar_alarm_msg.data = false;
    if (min_dis >= 0) {
        ROS_INFO( "TOO CLOSE TO WALL!! min distance = %f", laser_scan.ranges[min_dis] );
        lidar_alarm_msg.data = true;
        lidar_alarm_publisher_.publish( lidar_alarm_msg );

        opt_dir = (min_dis * angle_increment_ + angle_min_) + M_PI;
        ROS_INFO("Best direction: %f degree", (opt_dir/M_PI)*180);
        opt_dir_msg.data = opt_dir;
        opt_dir_publisher_.publish( opt_dir_msg );
        return;
    }
    lidar_alarm_publisher_.publish( lidar_alarm_msg );
    for (int i = 0; i < 2 * ping_index_; ++i) {
        if (max_dis < 0) {
            max_dis = i;
        } else {
            if (laser_scan.ranges[max_dis] < laser_scan.ranges[i]) {
                max_dis = i;
            }
        }
        if (laser_scan.ranges[i] >= range_max_) {
            int current_opt = 0;
            avail = true;
            for (int k = i; k < 2 * ping_index_; ++k) {
                if (laser_scan.ranges[k] >= range_max_) {
                    current_opt++;
                }
                else {
                    if (current_opt > opt_pin_num) {
                        opt_pin = i + (current_opt)/2;
                        opt_pin_num = current_opt;
                    }
                    break;
                }
            }
        }
    }
    if (avail) {
        opt_dir = opt_pin * angle_increment_ + angle_min_;
        ROS_INFO("Best direction: %f degree", (opt_dir/M_PI)*180);
        opt_dir_msg.data = opt_dir;
        opt_dir_publisher_.publish( opt_dir_msg );
    } else {
            opt_dir = max_dis * angle_increment_ + angle_min_ + M_PI/12;
        ROS_INFO("Not bad direction: %f degree", (opt_dir/M_PI)*180);
        opt_dir_msg.data = opt_dir;
        opt_dir_publisher_.publish( opt_dir_msg );
    }
    return;
}


int main( int argc, char **argv )
{
    ros::init( argc, argv, "lidar_braker" );      /* name this node */
    ros::NodeHandle nh;
    /* create a Subscriber object and have it subscribe to the lidar topic */
    ros::Publisher pub = nh.advertise<std_msgs::Bool>( "lidar_alarm", 1 );
    lidar_alarm_publisher_ = pub;                   /* let's make this global, so callback can use it */
    ros::Publisher pub2 = nh.advertise<std_msgs::Float64>( "opt_direction", 1 );
    opt_dir_publisher_ = pub2;
    ros::Subscriber lidar_subscriber = nh.subscribe( "scan", 1, laserCallback );
    ros::spin();                                    /* this is essentially a "while(1)" statement, except it */
    /*
     * forces refreshing wakeups upon new data arrival
     * main program essentially hangs here, but it must stay alive to keep the callback function alive
     */
    return(0);                                      /* should never get here, unless roscore dies */
}


