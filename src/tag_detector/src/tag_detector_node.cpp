#include <iostream>

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Odometry.h>
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>
#include <Eigen/SVD>
//EIgen SVD libnary, may help you solve SVD
//JacobiSVD<MatrixXd> svd(A, ComputeThinU | ComputeThinV);

using namespace cv;
using namespace aruco;
using namespace Eigen;

//global varialbles for aruco detector
aruco::CameraParameters CamParam;
MarkerDetector MDetector;
vector<Marker> Markers;
float MarkerSize = 0.20 / 1.5 * 1.524;
float MarkerWithMargin = MarkerSize * 1.2;
BoardConfiguration TheBoardConfig;
BoardDetector TheBoardDetector;
Board TheBoardDetected;
ros::Publisher pub_odom_yourwork;
ros::Publisher pub_odom_ref;
ros::Publisher pub_odom_world;
cv::Mat K, D;

// test function, can be used to verify your estimation
int calculateReprojectionError(const vector<cv::Point3f> &pts_3, const vector<cv::Point2f> &pts_2, const cv::Mat R, const cv::Mat t)
{
    // puts("calculateReprojectionError begins");
    vector<cv::Point2f> un_pts_2;
    cv::undistortPoints(pts_2, un_pts_2, K, D);
    int error_count=0;
    for (unsigned int i = 0; i < pts_3.size(); i++)
    {
        cv::Mat p_mat(3, 1, CV_64FC1);
        p_mat.at<double>(0, 0) = pts_3[i].x;
        p_mat.at<double>(1, 0) = pts_3[i].y;
        p_mat.at<double>(2, 0) = pts_3[i].z;
        cv::Mat p = (R * p_mat + t);
        //        printf("(%f, %f, %f) -> (%f, %f) and (%f, %f)\n",
        //               pts_3[i].x, pts_3[i].y, pts_3[i].z,
        //               un_pts_2[i].x, un_pts_2[i].y,
        //               p.at<double>(0) / p.at<double>(2), p.at<double>(1) / p.at<double>(2));
        if(fabs(un_pts_2[i].x-(p.at<double>(0) / p.at<double>(2)))>0.2
                ||fabs(un_pts_2[i].y-(p.at<double>(1)/ p.at<double>(2)))>0.2)
        {
            error_count++;
        }
    }
    if(error_count>=(pts_3.size()/5))
    {
        return 0;
    }
    return 1;
    //    puts("calculateReprojectionError ends");
}

// the main function you need to work with
// pts_id: id of each point
// pts_3: 3D position (x, y, z) in world frame
// pts_2: 2D position (u, v) in image frame
void process(const vector<int> &pts_id, const vector<cv::Point3f> &pts_3, const vector<cv::Point2f> &pts_2, const ros::Time& frame_time)
{
    //version 1, as reference
    cv::Mat r, rvec, t;
    cv::solvePnP(pts_3, pts_2, K, D, rvec, t);
    cv::Rodrigues(rvec, r);

    if(calculateReprojectionError(pts_3,pts_2,r,t))
    {
        Matrix3d R_ref;
        for(int i=0;i<3;i++)
            for(int j=0;j<3;j++)
            {
                R_ref(i,j) = r.at<double>(i, j);
            }
        Quaterniond Q_ref;
        Q_ref = R_ref;
        nav_msgs::Odometry odom_ref;
        odom_ref.header.stamp = frame_time;
        odom_ref.header.frame_id = "world";
        odom_ref.pose.pose.position.x = t.at<double>(0, 0);
        odom_ref.pose.pose.position.y = t.at<double>(1, 0);
        odom_ref.pose.pose.position.z = t.at<double>(2, 0);
        odom_ref.pose.pose.orientation.w = Q_ref.w();
        odom_ref.pose.pose.orientation.x = Q_ref.x();
        odom_ref.pose.pose.orientation.y = Q_ref.y();
        odom_ref.pose.pose.orientation.z = Q_ref.z();
        pub_odom_ref.publish(odom_ref);

        /* ---------- camera pose in world frame ---------- */
        Eigen::Matrix3d R_x180;
        const double PI = 3.141592;
        R_x180 << 1, 0, 0, 
                  0, -1, 0, 
                  0 ,0, -1; 

        Eigen::Vector3d translation;
        Eigen::Matrix3d rotation;
        Eigen::Quaterniond q_world;

        rotation = R_ref.inverse();
        translation = -rotation * Eigen::Vector3d(t.at<double>(0, 0), t.at<double>(1, 0), t.at<double>(2, 0));

        rotation = R_x180 * rotation;
        translation = R_x180 * translation;
        q_world = rotation;

        nav_msgs::Odometry odom_world;
        odom_world.header.stamp = frame_time;
        odom_world.header.frame_id = "world";
        odom_world.pose.pose.position.x = translation(0);
        odom_world.pose.pose.position.y = translation(1);
        odom_world.pose.pose.position.z = translation(2);
        odom_world.pose.pose.orientation.w = q_world.w();
        odom_world.pose.pose.orientation.x = q_world.x();
        odom_world.pose.pose.orientation.y = q_world.y();
        odom_world.pose.pose.orientation.z = q_world.z();
        pub_odom_world.publish(odom_world);
    }

    //    Vector3d T_ref;
    //    T_ref(0) = t.at<double>(0, 0);
    //    T_ref(1) = t.at<double>(1, 0);
    //    T_ref(2) = t.at<double>(2, 0);

    //    // version 2, your work
    //    Matrix3d R;
    //    Vector3d T;
    //    R.setIdentity();
    //    T.setZero();
    //    //

    //    Eigen::Matrix3d Kcamera;
    //    Kcamera << K.at<double>(0) , K.at<double>(1), K.at<double>(2),
    //            K.at<double>(3),K.at<double>(4),K.at<double>(5),
    //            K.at<double>(6),K.at<double>(7),K.at<double>(8);
    //    float fx=K.at<double>(0);
    //    float cx=K.at<double>(2);
    //    float fy=K.at<double>(4);
    //    float cy=K.at<double>(5);
    //    vector<cv::Point2f> pts_2_undistort_uniform;
    //    cv::undistortPoints(pts_2 , pts_2_undistort_uniform, K, D);


    //    Eigen::MatrixXd A(2*pts_3.size(), 9);
    //    for( size_t i=0; i<pts_2.size(); i++)
    //    {
    //        float u, v, X, Y;
    //        u = pts_2_undistort_uniform[i].x*fx+cx;
    //        v = pts_2_undistort_uniform[i].y*fy+cy;
    //        X = pts_3[i].x;
    //        Y = pts_3[i].y;
    //        //cout << "u:" << u << "  v:" << v << "  X:" << X <<"  Y:"<< Y << endl;

    //        A(2*i,0) = X;
    //        A(2*i,1) = Y;
    //        A(2*i,2) = 1;
    //        A(2*i,3) =  0;
    //        A(2*i,4) =  0;
    //        A(2*i,5) =  0;
    //        A(2*i,6) = -X*u;
    //        A(2*i,7) = -Y*u;
    //        A(2*i,8) = -u;

    //        A(2*i+1,0) =  0;
    //        A(2*i+1,1) =  0;
    //        A(2*i+1,2) =  0;
    //        A(2*i+1,3) = X;
    //        A(2*i+1,4) = Y;
    //        A(2*i+1,5) = 1;
    //        A(2*i+1,6) = -X*v;
    //        A(2*i+1,7) = -Y*v;
    //        A(2*i+1,8) = -v;
    //    }

    //    //Step1: Solve H_EST matrix
    //    JacobiSVD<MatrixXd> svdA(A, ComputeFullU | ComputeFullV);
    //    Eigen::VectorXd h  = VectorXd::Zero(9);
    //    h = svdA.matrixV().col(8);
    //    Eigen::Matrix3d H_EST;
    //    H_EST<< h(0), h(1), h(2),
    //            h(3), h(4), h(5),
    //            h(6), h(7), h(8);
    //    if(h(8)<0)
    //    {
    //        H_EST = -H_EST;
    //    }


    //    Eigen::Matrix3d KinvHest;
    //    KinvHest = Kcamera.inverse()*H_EST;

    //    Eigen::Vector3d h1;
    //    Eigen::Vector3d h2;
    //    Eigen::Matrix3d H1H2_H1H2;
    //    h1 = KinvHest.col(0);
    //    h2 = KinvHest.col(1);
    //    H1H2_H1H2 << h1, h2, h1.cross(h2);
    //    JacobiSVD<MatrixXd> svdH(H1H2_H1H2, ComputeFullU | ComputeFullV);
    //    Eigen::Matrix3d U, VT;

    //    U = svdH.matrixU();
    //    VT = svdH.matrixV().transpose();
    //    R  = U*VT;
    //    T = KinvHest.col(2) / ((KinvHest.col(0)).norm());

    //    //...
    //    //...
    //    //...
    //    Quaterniond Q_yourwork;
    //    Q_yourwork = R;
    //    nav_msgs::Odometry odom_yourwork;
    //    odom_yourwork.header.stamp = frame_time;
    //    odom_yourwork.header.frame_id = "world";
    //    odom_yourwork.pose.pose.position.x = T(0);
    //    odom_yourwork.pose.pose.position.y = T(1);
    //    odom_yourwork.pose.pose.position.z = T(2);
    //    odom_yourwork.pose.pose.orientation.w = Q_yourwork.w();
    //    odom_yourwork.pose.pose.orientation.x = Q_yourwork.x();
    //    odom_yourwork.pose.pose.orientation.y = Q_yourwork.y();
    //    odom_yourwork.pose.pose.orientation.z = Q_yourwork.z();
    //    pub_odom_yourwork.publish(odom_yourwork);


    //    static int cnt=0;
    //    static double sum_s_pos_error = 0;
    //    static double sum_s_rot_error = 0;
    //    // calculate valid RMS error;
    //    Eigen::Vector3d T_error;
    //    T_error = T-T_ref;
    //    double pos_error = fabs(T_error(0))+fabs(T_error(1))+fabs(T_error(2));
    //    //cout << pos_error << endl;
    //    Quaterniond Q_error;
    //    Q_error = Q_yourwork * Q_ref.inverse();
    //    //cout << Q_error.w() << "  " << Q_error.x() << "  " << Q_error.y() << "  " << Q_error.z() << endl;
    //    double rot_error = fabs(Q_error.x()) + fabs(Q_error.y()) + fabs(Q_error.z());
    //    //cout << rot_error << endl;
    //    if(pos_error<0.5 && rot_error<0.5)
    //    {
    //        sum_s_pos_error+=pos_error*pos_error;
    //        sum_s_rot_error+=rot_error*rot_error;
    //        cnt++;
    //        cout << "RMS_POS:" << sqrt(sum_s_pos_error/cnt) << endl;
    //        cout << "RMS_ROT:" << sqrt(sum_s_rot_error/cnt) << endl;
    //    }else
    //    {
    //        cout << "bad 2D-3D alignment" << endl;
    //    }
}

cv::Point3f getPositionFromIndex(int idx, int nth)
{
    int idx_x = idx % 6, idx_y = idx / 6;
    double p_x = idx_x * MarkerWithMargin - (3 + 2.5 * 0.2) * MarkerSize;
    double p_y = idx_y * MarkerWithMargin - (12 + 11.5 * 0.2) * MarkerSize;
    return cv::Point3f(p_x + (nth == 1 || nth == 2) * MarkerSize, p_y + (nth == 2 || nth == 3) * MarkerSize, 0.0);
}

void img_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    ROS_INFO("1");

    double t = clock();
    cv_bridge::CvImagePtr bridge_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
    MDetector.detect(bridge_ptr->image, Markers);
    float probDetect = TheBoardDetector.detect(Markers, TheBoardConfig, TheBoardDetected, CamParam, MarkerSize);
    ROS_DEBUG("p: %f, time cost: %f\n", probDetect, (clock() - t) / CLOCKS_PER_SEC);

    vector<int> pts_id;
    vector<cv::Point3f> pts_3;
    vector<cv::Point2f> pts_2;
    for (unsigned int i = 0; i < Markers.size(); i++)
    {
        int idx = TheBoardConfig.getIndexOfMarkerId(Markers[i].id);

        char str[100];
        sprintf(str, "%d", idx);
        cv::putText(bridge_ptr->image, str, Markers[i].getCenter(), CV_FONT_HERSHEY_COMPLEX, 0.4, cv::Scalar(-1));
        for (unsigned int j = 0; j < 4; j++)
        {
            sprintf(str, "%d", j);
            cv::putText(bridge_ptr->image, str, Markers[i][j], CV_FONT_HERSHEY_COMPLEX, 0.4, cv::Scalar(-1));
        }

        for (unsigned int j = 0; j < 4; j++)
        {
            pts_id.push_back(Markers[i].id * 4 + j);
            pts_3.push_back(getPositionFromIndex(idx, j));
            pts_2.push_back(Markers[i][j]);
        }
    }

    //begin your function
    if (pts_id.size() > 6)
        process(pts_id, pts_3, pts_2, img_msg->header.stamp);

    // cv::imshow("in", bridge_ptr->image);
    // cv::waitKey(10);

    ROS_INFO("2");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tag_detector");
    ros::NodeHandle n("~");

    ros::Subscriber sub_img = n.subscribe("image_raw", 100, img_callback);
    pub_odom_yourwork = n.advertise<nav_msgs::Odometry>("odom_yourwork",10);
    pub_odom_ref = n.advertise<nav_msgs::Odometry>("tag_ref",10);
    pub_odom_world = n.advertise<nav_msgs::Odometry>("pnp_odom_world",10);
    //init aruco detector
    string cam_cal, board_config;
    n.getParam("cam_cal_file", cam_cal);
    n.getParam("board_config_file", board_config);
    CamParam.readFromXMLFile(cam_cal);
    TheBoardConfig.readFromFile(board_config);

    //init intrinsic parameters
    cv::FileStorage param_reader(cam_cal, cv::FileStorage::READ);
    param_reader["camera_matrix"] >> K;
    param_reader["distortion_coefficients"] >> D;

    //init window for visualization
    cv::namedWindow("in", 1);

    ros::spin();
}
