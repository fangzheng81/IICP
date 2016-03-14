#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
//#include <visualization_msgs/MarkerArray.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
//#include <ros/time.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl_conversions/pcl_conversions.h>
#include <message_filters/synchronizer.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <nav_msgs/Path.h>

#include <iostream>
#include <string>
#include "common.h"
#include "iaicp.h"
#include "camera_tracker.h"


using namespace std;
namespace fs = boost::filesystem;
using namespace pcl;
using namespace sensor_msgs;
using namespace message_filters;
using namespace cv;

bool isDebug=true;
CloudPtr cFrame;

string param_sub_topic;
string param_rgb_topic;
string param_depth_topic;
ros::Publisher pub_mtrack;
ros::Publisher pub_roi;
ros::Publisher pub_particlecloud;

ros::Publisher pub_corres;
ros::Publisher pub_keyframes;
ros::Publisher pub_campose;
ros::Publisher pub_camgtpose;
ros::Publisher pub_camposepath;
ros::Publisher pub_camgtposepath;

bool usepoints;
tf::TransformListener *tf_listener;

CameraTracker *theTracker;
ofstream out("./estimation.txt");

//variables for compare with gt
float disterr=0.f;
float angerr=0.f;
tracking::ParticleXYZRPY trackerr;
int cnt = 0;

double trackingTimeSum =0.f;
double trackingTimeMax =-100.f;

nav_msgs::Path campath;
nav_msgs::Path camgtpath;

bool readParameters();
tf::StampedTransform transform1;

void cb_rgb(const sensor_msgs::PointCloud2& input)
{
    cnt++;
    double lastT=pcl::getTime();
    cFrame.reset(new Cloud);
    pcl::fromROSMsg(input, *cFrame);
    if(isDebug) cout<<cnt<<"th frame data, # of points: "<<cFrame->points.size()<<endl;
    if (isDebug) cout<<"computation time for loading: "<<pcl::getTime()-lastT<<"s"<<endl;
    lastT=pcl::getTime();
    theTracker->run(cFrame);
    double thisTime = pcl::getTime()-lastT;
    if (thisTime>trackingTimeMax) {trackingTimeMax=thisTime;}
    trackingTimeSum += thisTime;
    if (isDebug) cout<<"computation time for tracking: "<<pcl::getTime()-lastT<<"s"<<endl;
    if (isDebug) cout<<"average computation time for tracking: "<<trackingTimeSum/float(cnt)<<"s"<<endl;
    if (isDebug) cout<<"maximum computation time for tracking: "<<trackingTimeMax<<"s"<<endl;


    double visTime=pcl::getTime();
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*theTracker->getTransformedScene(), output);
//    output.header.frame_id="world";
    output.header.frame_id=input.header.frame_id;

//    output.header.stamp=ros::Time::now();
    pub_mtrack.publish (output);


    sensor_msgs::PointCloud2 roi_output;
    pcl::toROSMsg(*theTracker->getRoi(), roi_output);
    roi_output.header.frame_id=input.header.frame_id;
    pub_roi.publish (roi_output);

    sensor_msgs::PointCloud2 keys_output;
    if (cnt%20==1){
        pcl::toROSMsg(*theTracker->getKeyframes(), keys_output);
//        keys_output.header.frame_id="world";
        keys_output.header.frame_id=input.header.frame_id;
        pub_keyframes.publish (keys_output);
    }

    float x_,y_,z_,qx_,qy_,qz_,qw_;
    theTracker->getXYZQ(x_,y_,z_,qx_,qy_,qz_,qw_);
    geometry_msgs::PoseStamped camerapose_out;
    camerapose_out.header.frame_id = input.header.frame_id;
    camerapose_out.pose.position.x = x_;
    camerapose_out.pose.position.y = y_;
    camerapose_out.pose.position.z = z_;
    camerapose_out.pose.orientation.x = qx_;
    camerapose_out.pose.orientation.y = qy_;
    camerapose_out.pose.orientation.z = qz_;
    camerapose_out.pose.orientation.w = qw_;
    pub_campose.publish (camerapose_out);

    campath.header.frame_id = input.header.frame_id;
    if (int(cnt)%2==1)
    campath.poses.push_back(camerapose_out);


    pub_camposepath.publish(campath);
    out.open("./estimation.txt", std::ofstream::out | std::ofstream::app);
    out<<input.header.stamp<<" "<<x_<<" "<<y_<<" "<<z_<<" "<<qx_<<" "<<qy_<<" "<<qz_<<" "<<qw_<<endl;
    out.close();
    if (isDebug) cout<<"computation time for visualization: "<<pcl::getTime()-visTime<<"s"<<endl;
    if (isDebug) cout<<"Total computation time: "<<pcl::getTime()-lastT<<"s"<<endl;
}

// in case the input data is RGB & Depth image pair
void cb_test(const sensor_msgs::ImageConstPtr& input_color, const sensor_msgs::ImageConstPtr& input_depth)
{
    cnt++;
    cout<<cnt<<"th frame!!"<<endl;
    double lastT=pcl::getTime();

    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(input_color, "rgb8");
    cv_bridge::CvImagePtr cv_ptr2;
    cv_ptr2 = cv_bridge::toCvCopy(input_depth,  sensor_msgs::image_encodings::TYPE_32FC1);

    if (isDebug) cout<<"computation time for loading: "<<pcl::getTime()-lastT<<"s"<<endl;
    lastT=pcl::getTime();
    theTracker->run(cv_ptr->image, cv_ptr2->image);
    if (isDebug) cout<<"computation time for tracking: "<<pcl::getTime()-lastT<<"s"<<endl;

    double visTime=pcl::getTime();
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*theTracker->getTransformedScene(), output);
    output.header.frame_id=input_depth->header.frame_id;
    pub_mtrack.publish (output);

    sensor_msgs::PointCloud2 roi_output;
    pcl::toROSMsg(*theTracker->getRoi(), roi_output);
    roi_output.header.frame_id=input_depth->header.frame_id;
    pub_roi.publish (roi_output);



    sensor_msgs::PointCloud2 keys_output;
    if (cnt%20==1){
        pcl::toROSMsg(*theTracker->getKeyframes(), keys_output);
        keys_output.header.frame_id=input_depth->header.frame_id;
        pub_keyframes.publish (keys_output);
    }

    float x_,y_,z_,qx_,qy_,qz_,qw_;
    theTracker->getXYZQ(x_,y_,z_,qx_,qy_,qz_,qw_);
    geometry_msgs::PoseStamped camerapose_out;
    camerapose_out.header.frame_id = input_depth->header.frame_id;
    camerapose_out.pose.position.x = x_;
    camerapose_out.pose.position.y = y_;
    camerapose_out.pose.position.z = z_;
    camerapose_out.pose.orientation.x = qx_;
    camerapose_out.pose.orientation.y = qy_;
    camerapose_out.pose.orientation.z = qz_;
    camerapose_out.pose.orientation.w = qw_;
    pub_campose.publish (camerapose_out);

    campath.header.frame_id = input_depth->header.frame_id;
    if (int(cnt)%2==1)
    campath.poses.push_back(camerapose_out);


    pub_camposepath.publish(campath);
    out.open("./estimation.txt", std::ofstream::out | std::ofstream::app);
    out<<input_depth->header.stamp<<" "<<x_<<" "<<y_<<" "<<z_<<" "<<qx_<<" "<<qy_<<" "<<qz_<<" "<<qw_<<endl;
    out.close();
    if (isDebug) cout<<"computation time for visualization: "<<pcl::getTime()-visTime<<"s"<<endl;
    if (isDebug) cout<<"Total computation time: "<<pcl::getTime()-lastT<<"s"<<endl;
}


int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "iicp");
    ros::NodeHandle n;

    // read parameters
    if(!readParameters())   return 0;

    // publishers
    pub_mtrack = n.advertise<sensor_msgs::PointCloud2>("/dhri/mtrack", 1);
    pub_roi = n.advertise<sensor_msgs::PointCloud2>("/dhri/roi", 1);
    pub_particlecloud = n.advertise<sensor_msgs::PointCloud2>("/dhri/pcloud", 1);
    pub_corres = n.advertise<sensor_msgs::PointCloud2>("/dhri/corres", 1);
    pub_keyframes = n.advertise<sensor_msgs::PointCloud2>("/dhri/keyframes", 1);
    pub_campose = n.advertise<geometry_msgs::PoseStamped>("/dhri/camerapose", 1);
    pub_camgtpose = n.advertise<geometry_msgs::PoseStamped>("/dhri/cameragtpose", 1);

    pub_camposepath = n.advertise<nav_msgs::Path>("/dhri/cameraposepath", 1);
    pub_camgtposepath = n.advertise<nav_msgs::Path>("/dhri/cameragtposepath", 1);

    tf_listener = new tf::TransformListener(ros::Duration(5000));

    theTracker = new CameraTracker;
    cout<<param_rgb_topic.data()<<"  "<<param_depth_topic.data()<<endl;

    int buffernumber;
    ros::param::get("/buffernumber", buffernumber);
    ros::Subscriber sub_id = n.subscribe(param_sub_topic.data(), buffernumber, cb_rgb);


//    if (usepoints){
//        ros::Subscriber sub_id = n.subscribe(param_sub_topic.data(), buffernumber, cb_rgb);

//    } else{
//        // if using RGB D image pair.
//        message_filters::Subscriber<Image> sub_rgb(n, param_rgb_topic, buffernumber);
//        message_filters::Subscriber<Image> sub_depth(n, param_depth_topic, buffernumber);
//        typedef sync_policies::ApproximateTime<Image, Image> MySyncPolicy;
//        Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub_rgb, sub_depth);
//        sync.registerCallback(boost::bind(&cb_test, _1, _2));
//    }

    ros::spin();
    return 0;
}


bool readParameters()
{

    bool isOK = 1;


    if(ros::param::has("/sub/rgb/topic"))
        ros::param::get("/sub/rgb/topic", param_rgb_topic);
    else isOK = 0;

    if(ros::param::has("/sub/depth/topic"))
        ros::param::get("/sub/depth/topic", param_depth_topic);
    else isOK = 0;

    if(ros::param::has("/sub/points/topic"))
        ros::param::get("/sub/points/topic", param_sub_topic);
    else isOK = 0;

    if(ros::param::has("/usepoints"))
        ros::param::get("/usepoints", usepoints);
    else isOK = 0;

//    param_sub_topic ="/camera/depth_registered/points";
////    param_sub_topic ="/camera/rgb/points";
    if(isOK){
    }
    else{
        ROS_WARN("Run 'rosparam load param.yaml' first");
        return 0;
    }
}

