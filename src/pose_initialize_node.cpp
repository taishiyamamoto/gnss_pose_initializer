#include <cmath>

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>

class PoseInitializer{
    public:
        PoseInitializer() : nh_(), tfBuffer_(), tfListener_(tfBuffer_){
            ROS_INFO("pose_initializer start");
            prev_trans_.transform.translation.x = 0;
            prev_trans_.transform.translation.y = 0;
            prev_trans_.transform.translation.z = 0;
            timer_ = nh_.createTimer(ros::Duration(0.1),&PoseInitializer::timer_callback,this);
            pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped> ("initialpose",1000);
            sub_ = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped> ("fix",1000, &PoseInitializer::callback,this);
        }
        void timer_callback(const ros::TimerEvent& e);
        void callback(const geometry_msgs::PoseWithCovarianceStamped pose);
        double calc_pose_diff(const geometry_msgs::TransformStamped trans, const geometry_msgs::TransformStamped prev_trans);
    private:
        ros::NodeHandle nh_;
        ros::Timer timer_;
        ros::Publisher pub_;
        ros::Subscriber sub_;
        tf2_ros::Buffer tfBuffer_;
        tf2_ros::TransformListener tfListener_;
        geometry_msgs::TransformStamped prev_trans_;
        geometry_msgs::PoseWithCovarianceStamped gnss_pose_;
        double sum_pose_diff_;
        double count_;
};

void 
PoseInitializer::timer_callback(const ros::TimerEvent& e){
    geometry_msgs::TransformStamped trans;
      try
      {
        trans = tfBuffer_.lookupTransform("odom", "base_link", ros::Time(0));
      }
      catch (tf2::TransformException& ex)
      {
        ROS_WARN("%s", ex.what());
        return;
      }

    double pose_diff = calc_pose_diff(trans, prev_trans_);
    sum_pose_diff_ += pose_diff;
    count_++;

    ROS_INFO("pose_diff = %lf",pose_diff);
    ROS_INFO("sum_pose_diff = %lf",sum_pose_diff_);

    if(count_ >= 100){
        if(sum_pose_diff_ <= 1.0){
            ROS_WARN("low movement detected");

            geometry_msgs::PoseWithCovarianceStamped initialpose;

            initialpose.header.frame_id = "map";
            initialpose.header.stamp = ros::Time(0);
            initialpose.pose = gnss_pose_.pose;
            initialpose.pose.pose.orientation.x= 0;
            initialpose.pose.pose.orientation.y= 0;
            initialpose.pose.pose.orientation.z= 0;
            initialpose.pose.pose.orientation.w= 1;

            pub_.publish(initialpose);
            ROS_INFO("publish gnss pose");
        }
        sum_pose_diff_ = 0;
        count_ = 0;
    }

    prev_trans_ = trans;
    
}

void 
PoseInitializer::callback(const geometry_msgs::PoseWithCovarianceStamped pose){
    ROS_INFO("in gnss_callback");
    gnss_pose_.pose = pose.pose;
}

double 
PoseInitializer::calc_pose_diff(const geometry_msgs::TransformStamped trans, const geometry_msgs::TransformStamped prev_trans){
    double x,y,z; 

    x = trans.transform.translation.x - prev_trans.transform.translation.x;
    y = trans.transform.translation.y - prev_trans.transform.translation.y;
    z = trans.transform.translation.z - prev_trans.transform.translation.z;

    return std::sqrt(std::pow(x,2) + std::pow(y,2) + std::pow(z,2));
}

int main(int argc, char** argv){
    ros::init(argc, argv, "pose_initializer");
    PoseInitializer pi;
    ros::spin();
    return 0;
}