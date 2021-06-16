#include "ros/ros.h"
#include "arduino_msgs/Encoders.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_broadcaster.h"
#include <sstream>
#include <cmath>

#define R 0.065 //[m] radius
#define L 0.33  //[m] dist between axis

// Odom publisher and odom->base_link TF broadcaster
ros::Publisher odom_pub;
boost::shared_ptr<tf::TransformBroadcaster> odom_brdcast;

// Variables that needs to be remembered for every calculation
// and need initial condition
double V = 0;        //[m/s] linear velocity
double W = 0;        //[rad/s] angular velocity
double theta = 0;   //[rad] angle of robot according to X axis
double x = 0;       //[m] distance in x axis
double y = 0;       //[m] distance in y axis
// Variables that needed to be global
ros::Time currentTime, lastTime;
bool firstOdomMsg = true;

void callback( const arduino_msgs::Encoders::ConstPtr &enc)
{
    // ROS objects
    nav_msgs::Odometry odom;
    geometry_msgs::TransformStamped odom_trans;
    geometry_msgs::Quaternion odom_quat;

    // Variables for counting odometry
    double Vx;       //[m/s] linear velocity in x axis
    double Vy;       //[m/s] linear velocity in y axis
    double deltaTheta;//[rad] angle changed in sample time
    double distance; //[m] distance travelled in sample time
    double dt;        //[s] time between current and last odometry data
    double Vr;      //[rad/s] Angular velocity of right wheel
    double Vl;      //[rad/s] Angular velocity of left wheel
    double Vavrg;   //[rad/s] Average angular veolocity of right and left wheels

    // Calculation of odometry
    //Get sample time
    currentTime = ros::Time::now();
    dt = (currentTime - lastTime).toSec();
    lastTime = currentTime;
    //Count odom
    // Prevent from too high dt and ommit first odom msg
    if (!firstOdomMsg){
        deltaTheta = W*dt;
        distance = V*dt;
        x = x + distance*cos(theta + deltaTheta/2);
        y = y + distance*sin(theta + deltaTheta/2);
        theta = theta + deltaTheta;
        // Encoders fuck up driving in straight line, cuz once right is 9, left is 10 and
        // angular velocity is not zero. That's why if difference between encoders is less than 3
        // Vl and Vr are counted as the same and are proportional to (encRight + encLeft)/2
        if (abs(enc->rightEnc - enc->leftEnc) > 2){
            Vr = 2*M_PI*enc->leftEnc/(500*dt);
            Vl = 2*M_PI*enc->rightEnc/(500*dt);
        }
        else{
            Vavrg = (enc->leftEnc + enc->rightEnc)/2;
            Vr = Vl = 2*M_PI*Vavrg/(500*dt);
        }
        V = R*(Vr + Vl)/2;
        Vx = V*cos(theta);
        Vy = V*sin(theta);
        W = R*(Vr - Vl)/L;
        // Odometry needs quaternions, so angle theta is converted to quaternions.
        odom_quat = tf::createQuaternionMsgFromYaw(theta);

        // First send odom->base_link TF
        odom_trans.header.stamp = currentTime;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";

        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        odom_brdcast->sendTransform(odom_trans);

        // Then send odom msg
        odom.header.stamp = currentTime;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;

        odom.twist.twist.linear.x = Vx;
        odom.twist.twist.linear.y = Vy;
        odom.twist.twist.angular.z = W;

        odom_pub.publish(odom);
    }
    else
        // Checkbox that first message has been sent
        firstOdomMsg = false;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "odom");
    ros::NodeHandle nh;
    // Creating tf::TransformBroadcaster needs ros::init been called, so pointer to this
    // broadcaster is created as a global variable and here broadcaster is created.
    // If it is created as a global classic variable, error that ros::init needs to be called
    // before creating this occurs.
    odom_brdcast.reset(new tf::TransformBroadcaster());

    odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 1000);

    lastTime = ros::Time::now();
    ros::Subscriber sub = nh.subscribe("encoders", 1000, callback);
    ros::spin();

    return 0;
}
