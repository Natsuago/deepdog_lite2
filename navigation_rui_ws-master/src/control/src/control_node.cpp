#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_datatypes.h>

#define PI 3.14159265358979

class DogController
{
public:
    DogController();

    void initMarker();

    double getEta(const geometry_msgs::Pose& dogPose);
    double getDog2GoalDist();
    double getDog2GoalAngle();
    double getL1Distance(const double& _Vcmd);
    double getSteeringAngle(double eta);
    double getYawFromPose(const geometry_msgs::Pose& dogPose);
    geometry_msgs::Point get_odom_dog2WayPtVec(const geometry_msgs::Pose& dogPose);

private:
    ros::NodeHandle n_;
    ros::Subscriber odom_sub, goal_sub;
    ros::Publisher cmd_pub, marker_pub;
    ros::Timer timer1, timer2;

    visualization_msgs::Marker points, line_strip, goal_circle;

    geometry_msgs::Twist cmd_vel;
    geometry_msgs::Point odom_goal_pos;
    geometry_msgs::Quaternion odom_goal_quat;
    nav_msgs::Odometry odom;
    nav_msgs::Path odom_path;

    double L, Lfw, Vcmd, lfw, steering, u, v;
    double base_angle, base_speed, angle_gain, goal_radius;
    int controller_freq;
    bool foundForwardPt, goal_received, goal_reached;
    int run_status; // 0 转角度, 1 直走, 2 最终确认角度, -1 停止
    double angle_cmd;
    double min_angle_range;
    double min_goal_distance;

    void odomCB(const nav_msgs::Odometry::ConstPtr& odomMsg);
    void pathCB(const nav_msgs::Path::ConstPtr& pathMsg);
    void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goalMsg);
    void goalReachingCB(const ros::TimerEvent&);
    void controlLoopCB(const ros::TimerEvent&);
};

DogController::DogController()
{
    //Private parameters handler
    ros::NodeHandle pn("~");

    //Dog parameter
    pn.param("L", L, 0.55);// length of car, car is 0.26, dog is 0.55
    pn.param("Vcmd", Vcmd, 1.0);// reference speed (m/s)
    pn.param("lfw", lfw, 0.275); // forward look ahead distance (m) car is 0.13

    //Controller parameter
    pn.param("controller_freq", controller_freq, 20);
    pn.param("angle_gain", angle_gain, 0.2);
    pn.param("base_speed", base_speed, 1.0);
    pn.param("base_angle", base_angle, 0.0);
    pn.param("angle_cmd", angle_cmd, 0.7); // 转向速度

    // 判断转到目标距离的阈值
    pn.param("min_angle_range", min_angle_range, 1.0); //使用角度表示
    pn.param("min_goal_distance", min_goal_distance, 0.1); //到达目标点的距离

    //Publishers and Subscribers
    odom_sub = n_.subscribe("/odometry/filtered_map", 1, &DogController::odomCB, this);
    goal_sub = n_.subscribe("/move_base_simple/goal", 1, &DogController::goalCB, this);
    cmd_pub = n_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    marker_pub = n_.advertise<visualization_msgs::Marker>("dog_path", 10);

    //Timer
    timer1 = n_.createTimer(ros::Duration((1.0)/controller_freq), &DogController::controlLoopCB, this); // Duration(0.05) -> 20Hz
    timer2 = n_.createTimer(ros::Duration((0.5)/controller_freq), &DogController::goalReachingCB, this); // Duration(0.05) -> 20Hz

    //Init variables
    Lfw = goal_radius = getL1Distance(Vcmd);
    foundForwardPt = false;
    goal_received = false;
    goal_reached = false;
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
    run_status = -1;

    //Show info
    ROS_INFO("[param] base_speed: %f", base_speed);
    ROS_INFO("[param] base_angle: %f", base_angle);
    ROS_INFO("[param] angle_gain: %f", angle_gain);
    ROS_INFO("[param] Vcmd: %f", Vcmd);
    ROS_INFO("[param] Lfw: %f", Lfw);

    //Visualization Marker Settings
    initMarker();
}

void DogController::initMarker()
{
    points.header.frame_id = line_strip.header.frame_id = goal_circle.header.frame_id = "odom";
    points.ns = line_strip.ns = goal_circle.ns = "Markers";
    points.action = line_strip.action = goal_circle.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_strip.pose.orientation.w = goal_circle.pose.orientation.w = 1.0;
    points.id = 0;
    line_strip.id = 1;
    goal_circle.id = 2;

    points.type = visualization_msgs::Marker::POINTS;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    goal_circle.type = visualization_msgs::Marker::CYLINDER;
    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.2;
    points.scale.y = 0.2;

    //LINE_STRIP markers use only the x component of scale, for the line width
    line_strip.scale.x = 0.1;

    goal_circle.scale.x = goal_radius;
    goal_circle.scale.y = goal_radius;
    goal_circle.scale.z = 0.1;

    // Points are green
    points.color.g = 1.0f;
    points.color.a = 1.0;

    // Line strip is blue
    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;

    //goal_circle is yellow
    goal_circle.color.r = 1.0;
    goal_circle.color.g = 1.0;
    goal_circle.color.b = 0.0;
    goal_circle.color.a = 0.5;
}

double DogController::getEta(const geometry_msgs::Pose& dogPose)
{
    geometry_msgs::Point odom_dog2WayPtVec = get_odom_dog2WayPtVec(dogPose);

    double eta = atan2(odom_dog2WayPtVec.y, odom_dog2WayPtVec.x);
    return eta;
}

double DogController::getDog2GoalDist()
{
    geometry_msgs::Point dog_pose = odom.pose.pose.position;
    double dog2goal_x = odom_goal_pos.x - dog_pose.x;
    double dog2goal_y = odom_goal_pos.y - dog_pose.y;

    double dist2goal = sqrt(dog2goal_x*dog2goal_x + dog2goal_y*dog2goal_y);

    return dist2goal;
}

double DogController::getDog2GoalAngle()
{
    geometry_msgs::Pose dog_pose = odom.pose.pose;
    tf::Quaternion q1(dog_pose.orientation.x, dog_pose.orientation.y, dog_pose.orientation.z, dog_pose.orientation.w); // dog pose 四元数
    tf::Quaternion q2(odom_goal_quat.x, odom_goal_quat.y, odom_goal_quat.z, odom_goal_quat.w); // goal四元数

    // 归一化四元数（保证四元数的模为1，确保正确的旋转计算）
    q1.normalize();
    q2.normalize();

    // 计算四元数之间的旋转偏角
    tf::Quaternion q_relative = q2 * q1.inverse(); // 四元数乘法和逆运算

    // 将四元数转换为旋转矩阵，以获取偏角
    tf::Matrix3x3 rotation_matrix(q_relative);
    double roll, pitch, yaw;
    rotation_matrix.getRPY(roll, pitch, yaw); // 得到滚转角、俯仰角和偏航角

    return yaw;
}

double DogController::getL1Distance(const double& _Vcmd)
{
    double L1 = 0;
    if(_Vcmd < 1.34)
        L1 = 3 / 3.0;
    else if(_Vcmd > 1.34 && _Vcmd < 5.36)
        L1 = _Vcmd*2.24 / 3.0;
    else
        L1 = 12 / 3.0;
    return L1;
}

double DogController::getSteeringAngle(double eta)
{
    double steering_angle = -atan2((L*sin(eta)),(Lfw/2+lfw*cos(eta)))*(180.0/PI);
    return steering_angle;
}

double DogController::getYawFromPose(const geometry_msgs::Pose& dogPose)
{
    float x = dogPose.orientation.x;
    float y = dogPose.orientation.y;
    float z = dogPose.orientation.z;
    float w = dogPose.orientation.w;

    double tmp, yaw;
    tf::Quaternion q(x,y,z,w);
    tf::Matrix3x3 quaternion(q);
    quaternion.getRPY(tmp, tmp, yaw);

    return yaw;
}

geometry_msgs::Point DogController::get_odom_dog2WayPtVec(const geometry_msgs::Pose& dogPose)
{
    geometry_msgs::Point dogPose_pos = dogPose.position;
    double dogPose_yaw = getYawFromPose(dogPose);
    geometry_msgs::Point forwardPt;
    geometry_msgs::Point odom_dog2WayPtVec;
    foundForwardPt = false;

    if(!goal_reached){
        forwardPt = odom_goal_pos;
        foundForwardPt = true;
    }
    else if(goal_reached)
        {
            forwardPt = odom_goal_pos;
            foundForwardPt = false;
            ROS_INFO("goal REACHED!");
        }

    /*Visualized Target Point on RVIZ*/
    /*Clear former target point Marker*/
    points.points.clear();
    line_strip.points.clear();

    if(foundForwardPt && !goal_reached)
        {
            points.points.push_back(dogPose_pos);
            points.points.push_back(forwardPt);
            line_strip.points.push_back(dogPose_pos);
            line_strip.points.push_back(forwardPt);
        }

    marker_pub.publish(points);
    marker_pub.publish(line_strip);

    odom_dog2WayPtVec.x = cos(dogPose_yaw)*(forwardPt.x - dogPose_pos.x) + sin(dogPose_yaw)*(forwardPt.y - dogPose_pos.y);
    odom_dog2WayPtVec.y = -sin(dogPose_yaw)*(forwardPt.x - dogPose_pos.x) + cos(dogPose_yaw)*(forwardPt.y - dogPose_pos.y);
    return odom_dog2WayPtVec;
}

void DogController::odomCB(const nav_msgs::Odometry::ConstPtr& odomMsg)
{
    odom = *odomMsg;
}

void DogController::goalCB(const geometry_msgs::PoseStamped::ConstPtr& goalMsg)
{
    odom_goal_pos = goalMsg->pose.position;
    odom_goal_quat = goalMsg->pose.orientation;
    goal_received = true;
    goal_reached = false;
    run_status = 0;

    /*Draw Goal on RVIZ*/
    goal_circle.pose = goalMsg->pose;
    marker_pub.publish(goal_circle);
}

void DogController::goalReachingCB(const ros::TimerEvent&)
{

    if(goal_received) {
        double dog2goal_dist = getDog2GoalDist();
        double dog2goal_angle = getDog2GoalAngle() * 180 / PI;

        if(dog2goal_dist < min_goal_distance) {
            if (abs(dog2goal_angle) < min_angle_range) {
                goal_reached = true;
                goal_received = false;
                run_status = -1;
                ROS_INFO("Goal Reached !");
            } else {
                run_status = 2;
            }
        }
    }
}

void DogController::controlLoopCB(const ros::TimerEvent&)
{
    geometry_msgs::Pose dogPose = odom.pose.pose;
    cmd_vel.linear.x = 0;
    cmd_vel.angular.z = 0;

    if(goal_received) {
        /*Estimate Steering Angle*/
        double eta = getEta(dogPose);
        if(foundForwardPt) {
            double angular = getSteeringAngle(eta) * angle_gain;
            if(!goal_reached) {
                if (run_status == 0) {
                    if (angular != 0) {
                        // 首先进行角度调整
                        if (abs(eta) <= min_angle_range * PI / 180 && eta >= 0) {
                            run_status = 1;
                        } else {
                            double direction = -1 * angular / abs(angular);
                            cmd_vel.angular.z = angle_cmd * direction;
                            // if (abs(angular) > angle_cmd) {
                            //     angular = angle_cmd;
                            // }
                            // cmd_vel.angular.z = direction * abs(angular);
                        }
                    }

                } else if (run_status == 1) {
                    // 朝目标点直线前进
                    cmd_vel.linear.x = base_speed;

                    if (angular != 0) {
                        // 行驶中进行角度调整
                        double direction = -1 * angular / abs(angular);
                        // cmd_vel.angular.z = angle_cmd * direction;
                        if (abs(angular) > angle_cmd) {
                            angular = angle_cmd;
                        }
                        cmd_vel.angular.z = direction * abs(angular) * 0.5;
                    }

                } else if (run_status == 2) {
                    // 到达目标点之后转向目标角度
                    double angular_quaternion = getDog2GoalAngle(); // 计算当前机械狗面向的方向和目标点的方向之间的夹角
                    if (angular_quaternion != 0) {
                        double direction = angular_quaternion / abs(angular_quaternion);
                        // double angular_q = angular_quaternion * angle_gain;
                        // if (abs(angular_quaternion) > angle_cmd) {
                        //     angular_quaternion = angle_cmd;
                        // }
                        // cmd_vel.angular.z = abs(angular_quaternion) * direction;
                        cmd_vel.angular.z = angle_cmd * direction;
                    }
                }

                if (isnan(cmd_vel.linear.x)) {
                    cmd_vel.linear.x = 0;
                }
                if (isnan(cmd_vel.angular.z)) {
                    cmd_vel.angular.z = 0;
                }

                ROS_INFO("eta: %.2f", eta);
                ROS_INFO("\nForward speed = %.2f\nSteering angle = %.2f", cmd_vel.linear.x, cmd_vel.angular.z);
            }
        }
    }
    cmd_pub.publish(cmd_vel);

}

int main(int argc, char **argv)
{
    //Initiate ROS
    ros::init(argc, argv, "DogController");
    DogController controller;
    ros::spin();
    return 0;
}
