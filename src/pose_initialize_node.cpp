#include <cmath>

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>

using std::cos;
using std::sin;
using std::sqrt;
using std::pow;
using std::string;

class PoseInitializer{
    public:
        PoseInitializer() : nh_(), pnh_("~"), tfBuffer_(), tfListener_(tfBuffer_){
            ROS_INFO("pose_initializer start");

            pnh_.param<string>("map_frame",map_frame_,"map");
            pnh_.param<string>("odom_frame",odom_frame_,"odom");
            pnh_.param<string>("base_frame",base_frame_,"base_link");

            pnh_.param<string>("utm_topic",utm_topic_,"gps/odom");
            pnh_.param<string>("gnss_pose_topic",gnss_pose_topic_,"gps/position");
            pnh_.param<std::vector<double>>("map_origin",map_origin_,{492820.935847, 5527515.63636, 0});

            pnh_.param<bool>("use_initializer", use_initializer_, true);
            pnh_.param<double>("timer_duration",timer_duration_,0.1);
            pnh_.param<double>("examine_duration",examine_duration_,10.0);

            prev_trans_.transform.translation.x = 0;
            prev_trans_.transform.translation.y = 0;
            prev_trans_.transform.translation.z = 0;

            timer_ = nh_.createTimer(ros::Duration(timer_duration_),&PoseInitializer::timer_callback,this);
            pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped> ("initialpose",1000);
            pub2_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped> (gnss_pose_topic_,1000);
            sub_ = nh_.subscribe<nav_msgs::Odometry> (utm_topic_,1000, &PoseInitializer::callback,this);
        }

        void timer_callback(const ros::TimerEvent& e);
        void callback(const nav_msgs::Odometry utm_msg);
        double calc_pose_diff(const geometry_msgs::TransformStamped trans, const geometry_msgs::TransformStamped prev_trans);
        geometry_msgs::PoseWithCovariance subtract_map_origin(geometry_msgs::PoseWithCovariance from_pose);

    private:
        ros::NodeHandle nh_;
        ros::NodeHandle pnh_;
        ros::Timer timer_;
        ros::Publisher pub_;
        ros::Publisher pub2_;
        ros::Subscriber sub_;

        tf2_ros::Buffer tfBuffer_;
        tf2_ros::TransformListener tfListener_;

        geometry_msgs::TransformStamped prev_trans_;
        geometry_msgs::PoseWithCovarianceStamped map_gnss_pose_;

        double sum_pose_diff_;
        double count_;

        std::string map_frame_, odom_frame_, base_frame_, utm_topic_, gnss_pose_topic_;
        std::vector<double> map_origin_;
        bool use_initializer_;
        double examine_duration_;
        double timer_duration_;
};

void 
PoseInitializer::timer_callback(const ros::TimerEvent& e){
    if(!use_initializer_){
        return;
    }

    geometry_msgs::TransformStamped trans;
      try
      {
        trans = tfBuffer_.lookupTransform(odom_frame_, base_frame_, ros::Time(0));
      }
      catch (tf2::TransformException& ex)
      {
        ROS_WARN("%s", ex.what());
        return;
      }

    double pose_diff = calc_pose_diff(trans, prev_trans_);
    sum_pose_diff_ += pose_diff;
    count_++;

    //ROS_INFO("pose_diff = %lf",pose_diff);
    //ROS_INFO("sum_pose_diff = %lf",sum_pose_diff_);

    if(count_ >= (examine_duration_ / timer_duration_) ){
        if(sum_pose_diff_ <= 1.0){
            ROS_WARN("low movement detected");

            geometry_msgs::PoseWithCovarianceStamped initialpose;

            initialpose.header.frame_id = map_frame_;
            initialpose.header.stamp = ros::Time(0);
            initialpose.pose = map_gnss_pose_.pose;

            pub_.publish(initialpose);
            ROS_INFO("publish gnss pose");
        }
        sum_pose_diff_ = 0;
        count_ = 0;
    }

    prev_trans_ = trans;
    
}

void 
PoseInitializer::callback(const nav_msgs::Odometry utm_pose){
    //ROS_INFO("in gnss_callback");

    map_gnss_pose_.pose = subtract_map_origin(utm_pose.pose);
    map_gnss_pose_.header.frame_id = map_frame_;
    map_gnss_pose_.header.stamp = ros::Time(0);

    pub2_.publish(map_gnss_pose_);
}

double 
PoseInitializer::calc_pose_diff(const geometry_msgs::TransformStamped trans, const geometry_msgs::TransformStamped prev_trans){
    double x,y,z; 

    x = trans.transform.translation.x - prev_trans.transform.translation.x;
    y = trans.transform.translation.y - prev_trans.transform.translation.y;
    z = trans.transform.translation.z - prev_trans.transform.translation.z;

    return sqrt(pow(x,2) + pow(y,2) + pow(z,2));
}

geometry_msgs::PoseWithCovariance
PoseInitializer::subtract_map_origin(geometry_msgs::PoseWithCovariance from_pose){
    double local_x, local_y;

    local_x = from_pose.pose.position.x - map_origin_[0];
    local_y = from_pose.pose.position.y - map_origin_[1];

    ROS_INFO("local_x = %lf", local_x);
    ROS_INFO("local_y = %lf", local_y);

    double rotate_local_x = cos(map_origin_[2]) * local_x - sin(map_origin_[2]) * local_y;
    double rotate_local_y = sin(map_origin_[2]) * local_x + cos(map_origin_[2]) * local_y;

    ROS_INFO("local_x = %lf", rotate_local_x);
    ROS_INFO("local_y = %lf", rotate_local_y);

    geometry_msgs::PoseWithCovariance to_pose;
    to_pose.pose.position.x = rotate_local_x;
    to_pose.pose.position.y = rotate_local_y;
    to_pose.pose.position.z = 0;

    to_pose.pose.orientation.x = 0;
    to_pose.pose.orientation.y = 0;
    to_pose.pose.orientation.z = 0;
    to_pose.pose.orientation.w = 1;

    to_pose.covariance = from_pose.covariance;
    
    return  to_pose;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "pose_initialize_node");
    PoseInitializer pi;
    ros::spin();
    return 0;
}