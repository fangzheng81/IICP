#include "camera_tracker.h"
#include <Eigen/Geometry>
using namespace pcl;



CameraTracker::CameraTracker()
{
    ros::param::get("/fx", fx);
    ros::param::get("/fy", fy);
    ros::param::get("/cx", cx);
    ros::param::get("/cy", cy);
    ros::param::get("/height", height);
    ros::param::get("/width", width);

    m_cnt = 0;
    m_key2cur = Affine3f::Identity();
    m_pose = Affine3f::Identity();
    m_lastpose = Affine3f::Identity();
    m_keypose = Affine3f::Identity();

    m_speed=Vector6f::Zero();
    m_acc=Vector6f::Zero();

    m_map.reset(new Cloud());
    m_current.reset(new Cloud);
}


CameraTracker::~CameraTracker()
{

}

void CameraTracker::run(CloudPtr scene)
{
    cout<<"width is "<<scene->width<<" height is: "<<scene->height<<"timestamp: "<<scene->header.stamp<<endl;
    recalibrate(scene);
    if (m_cnt==0) {  //for the first frame data
        m_current.reset(new Cloud());
        m_current = scene;
        m_keyframe.reset(new Cloud());
        m_keyframe = scene;
        icpEst.setupSource(m_keyframe);
    }else{
        Vector6f temp = toVector(m_key2cur);
        checkAngles(temp);
        //change keyframe if key2current is greater than certain threshold
        float dthr=0.08f, athr = (10.f/180)*M_PI;
        if((sqrtf(pow(temp(0),2)+ pow(temp(1),2)+ pow(temp(2),2))> dthr|| sqrtf(pow(temp(3),2) + pow(temp(4),2) + pow(temp(5),2))>athr)){
//        //change key frame very  nth frame
//        if(m_cnt%10==1){
            m_changeKey=false;
            m_keyframe.reset(new Cloud);
            m_keyframe = m_current;
            m_key2cur=Affine3f::Identity();
            m_keypose = m_pose;
            icpEst.setupSource(m_keyframe);
        }
        m_current.reset(new Cloud);
        m_current = scene;        
    }

    //estimate speed/acc
    m_lastspeed = m_speed;
    double dtime = m_thistime-m_lasttime;
    if (dtime< 1e6f*0.02f) {dtime=1e6f *0.03f;}
    m_speed= toVector(m_pose * m_lastpose.inverse());
    checkAngles(m_speed); m_speed = (m_speed/dtime) /**0.5f + m_lastspeed*0.5f*/;
    m_acc = (m_speed-m_lastspeed);
    checkAngles(m_acc);  m_acc = (m_acc/dtime);
    m_lasttime = m_thistime;
    m_thistime = (double)scene->header.stamp;
    cout<<"time interval is "<<m_thistime-m_lasttime<<endl;

    m_cnt++;
    //predict key2cur pose
    dtime = m_thistime-m_lasttime;
    if (dtime > 1e6f*0.03f) {dtime=1e6f *0.03f;}
    if(m_cnt>12)
       m_key2cur = toEigen(toVector(toEigen((m_speed*dtime + 0.5f*m_acc*pow(dtime, 2)) * 0.7f) * m_key2cur));

    icpEst.setupTarget(m_current);
    icpEst.setupPredict(m_key2cur);
    icpEst.run();
    m_key2cur =  toEigen(toVector(icpEst.getTransResult()));

    m_lastpose=m_pose;
    m_pose = toEigen(toVector(m_key2cur*m_keypose));


    cout<<"current camera pose realtive to first frame is: "<<endl<<m_pose.matrix()<<endl;

    //mapping : store frame every 20th frame
    if (m_cnt%20==1){
        CloudPtr temp(new Cloud());
        for(size_t i=0; i<width/8; i++){
            for(size_t j=0; j<height/8; j++){
                temp->points.push_back(m_current->points[j*8*width+i*8]);
            }
        }
        transformPointCloud(*temp, *temp, m_pose.inverse());
        *m_map += *temp;
    }
}


CloudPtr CameraTracker::getTransformedScene()
{
    m_transformedModel.reset(new Cloud);
    CloudPtr temp; temp.reset(new Cloud);
    for(size_t i=0; i<width/4; i++){
        for(size_t j=0; j<height/4; j++){
            temp->points.push_back(m_current->points[j*4*width+i*4]);
        }
    }
    transformPointCloud(*temp, *m_transformedModel, m_pose.inverse());
    return m_transformedModel;
}



CloudPtr CameraTracker::getRoi()
{
    return icpEst.getSalientSource();
}

Affine3f CameraTracker::getPose()
{
    return m_pose;
}

void  CameraTracker::getXYZQ(float &tx, float &ty, float &tz, float &qx, float &qy, float &qz, float &qw)
{
    Affine3f inv_ = m_pose.inverse();
    inv_ = toEigen(toVector(inv_));
    Vector6f pose = toVector(inv_);
    tx=pose(0); ty=pose(1); tz=pose(2);
    Quaternion<float> q(inv_.rotation());
    q.normalize();
    qx=q.x(); qy=q.y(); qz=q.z(); qw=q.w();
}


CloudPtr CameraTracker::getKeyframes()
{
    return m_map;
}

Point2i CameraTracker::warp(PointT pt)
{
    Point2i pt2i;
    pt2i.x = int(floor(fx/pt.z * pt.x + cx));
    pt2i.y = int(floor(fy/pt.z * pt.y + cy));
    return pt2i;
}

PointT CameraTracker::unwarp(Point2i coord, float depth)
{
    PointT pt3d;
    pt3d.z = depth;
    pt3d.x = (float(coord.x) - cx) * depth / fx;
    pt3d.y = (float(coord.y) - cy) * depth / fy;
    return pt3d;
}


void CameraTracker::recalibrate(CloudPtr &cloud)
{
    for(size_t i=0; i<width; i++){
        for(size_t j=0; j<height; j++){
            cloud->points[j*width+i].x = (float(i)-cx)*cloud->points[j*width+i].z/fx;
            cloud->points[j*width+i].y = (float(j)-cy)*cloud->points[j*width+i].z/fy;
        }
    }
}

void CameraTracker::checkAngles(Vector6f &vec)
{
   for(size_t i=3; i<6; i++){
       while (vec(i)>M_PI)  {vec(i) -= 2*M_PI;}
       while (vec(i)<-M_PI) {vec(i) += 2*M_PI;}
   }
}

Affine3f CameraTracker::toEigen(Vector6f pose)
{
    return pcl::getTransformation(pose(0),pose(1),pose(2),pose(3),pose(4),pose(5));
}

Vector6f CameraTracker::toVector(Affine3f pose)
{
    Vector6f temp;
    pcl::getTranslationAndEulerAngles(pose, temp(0),temp(1),temp(2),temp(3),temp(4),temp(5));
    checkAngles(temp);
    return temp;
}



CloudPtr CameraTracker::Mat2Cloud(Mat &imR, Mat &imD)
{
    CloudPtr cloud;
    cloud.reset(new Cloud);
    cloud->points.resize(width*height);

    for(size_t i=0; i<width; i++){
        for(size_t j=0; j<height; j++){
            PointT pt;
            if (! (imD.at<float>(j,i)==imD.at<float>(j,i)) ) {
                pt.z= 0.f/0.f;
                cloud->points.at(j*width+i) = pt;
                continue;
            }
            pt.z = imD.at<float>(j,i);
            pt.x = (float(i) - cx) * pt.z / fx;
            pt.y = (float(j) - cy) * pt.z / fy;
            Vec3b color = imR.at<Vec3b>(j,i);
            pt.r = (int)color.val[0];
            pt.g = (int)color.val[1];
            pt.b = (int)color.val[2];
            cloud->points.at(j*width+i) = pt;
        }
    }
    return cloud;
}

void CameraTracker::run(Mat rgb, Mat depth)   //feed in rgb and depth image
{
    CloudPtr scene (new Cloud());
    scene = Mat2Cloud(rgb, depth);
    if (m_cnt==0) {  //for the first frame data
        m_current.reset(new Cloud());
        m_current = scene;
        m_keyframe.reset(new Cloud());
        m_keyframe = scene;
        icpEst.setupSource(m_keyframe);
    }else{
        Vector6f temp = toVector(m_key2cur);
        checkAngles(temp);
        //change keyframe if key2current is greater than certain threshold
//        float dthr=0.08f, athr = (15.f/180)*M_PI;
//        if((sqrtf(pow(temp(0),2)+ pow(temp(1),2)+ pow(temp(2),2))> dthr|| sqrtf(pow(temp(3),2) + pow(temp(4),2) + pow(temp(5),2))>athr)){
//        //change key frame very  nth frame
        if(m_cnt%7==1){
//            CloudPtr temp(new Cloud());
//            for(size_t i=0; i<width/8; i++){
//                for(size_t j=0; j<height/8; j++){
//                    temp->points.push_back(m_current->points[j*8*width+i*8]);
//                }
//            }
//            transformPointCloud(*temp, *temp, m_pose.inverse());
//            *m_map += *temp;

            m_changeKey=false;
            m_keyframe.reset(new Cloud);
            m_keyframe = m_current;
            m_key2cur=Affine3f::Identity();
            m_keypose = m_pose;
            icpEst.setupSource(m_keyframe);
        }
        m_current.reset(new Cloud);
        m_current = scene;
    }

    //estimate speed/acc
    m_lastspeed = m_speed;
    double dtime = m_thistime-m_lasttime;
    if (dtime< 1e6f*0.02f) {dtime=1e6f *0.02f;}
    m_speed= toVector(m_pose * m_lastpose.inverse());
    checkAngles(m_speed); m_speed = (m_speed/dtime) /**0.5f + m_lastspeed*0.5f*/;
    m_acc = (m_speed-m_lastspeed);
    checkAngles(m_acc);  m_acc = (m_acc/dtime);
    m_lasttime = m_thistime;
    m_thistime = (double)scene->header.stamp;
    cout<<"time interval is "<<m_thistime-m_lasttime<<endl;

    m_cnt++;

    //predict key2cur pose
    dtime = m_thistime-m_lasttime;
    if (dtime< 1e6f*0.04f) {dtime=1e6f *0.04f;}
//    if(m_cnt>12)
//       m_key2cur = toEigen(toVector(toEigen((m_speed*dtime + 0.5f*m_acc*pow(dtime, 2)) * 0.5f) * m_key2cur));

    icpEst.setupTarget(m_current);
    icpEst.setupPredict(m_pose * m_lastpose.inverse()*m_key2cur);
    icpEst.run();
    m_key2cur =  toEigen(toVector(icpEst.getTransResult()));

    m_lastpose=m_pose;
    m_pose = toEigen(toVector(m_key2cur*m_keypose));


    cout<<"current camera pose realtive to first frame is: "<<endl<<m_pose.matrix()<<endl;

    //mapping : store frame every 20th frame
    if (m_cnt%30==1){
        CloudPtr temp(new Cloud());
        for(size_t i=0; i<width/8; i++){
            for(size_t j=0; j<height/8; j++){
                temp->points.push_back(m_current->points[j*8*width+i*8]);
            }
        }
        transformPointCloud(*temp, *temp, m_pose.inverse());
        *m_map += *temp;
    }
}
