#ifndef CAMERA_TRACKER_H_
#define CAMERA_TRACKER_H_



#include "common.h"
#include "iaicp.h"

using namespace pcl;
using namespace pcl::registration;
using namespace Eigen;
using namespace std;
using namespace cv;


typedef Matrix<float,6,1> Vector6f;
class CameraTracker
{
public:
    CameraTracker();
    ~CameraTracker();

    void run(CloudPtr scene);   //feed in the currrent received Cloud
    void run(Mat rgb, Mat depth);   //feed in rgb and depth image
    void recalibrate(CloudPtr &cloud);

    CloudPtr currentFrame;
    Affine3f m_key2cur;
    Affine3f m_pose, m_lastpose;
    Affine3f m_keypose;

    CloudPtr getRoi();
    Affine3f getPose();
    void getXYZQ(float &tx, float &ty, float &tz, float &qx, float &qy, float &qz, float &qw);
    CloudPtr getKeyframes();
    CloudPtr getTransformedScene();  //get transformed current frame

    Vector6f m_speed, m_lastspeed, m_acc;
    CloudPtr m_map;

    Point2i warp(PointT pt);
    PointT unwarp(Point2i coord, float depth);
    CloudPtr Mat2Cloud(Mat &imR, Mat &imD);

    void checkAngles(Vector6f &vec);
    Affine3f toEigen(Vector6f pose);
    Vector6f toVector(Affine3f pose);





private:
    Iaicp icpEst;
    bool m_changeKey;
    double m_lasttime, m_thistime;
    CloudPtr m_current, m_keyframe, m_transformedModel;
    int m_cnt;
    float fx, fy, cx, cy; //camera parameters    
    int width, height;
};



#endif // CAMERA_TRACKER_H
