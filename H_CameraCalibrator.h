

#ifndef CAMERACALIBRATOR_H
#define CAMERACALIBRATOR_H



#ifdef _DEBUG
#ifndef H_PRINT
#define  H_PRINT(x) {std::cout<<#x<<" : "<<std::endl<<x<<std::endl<<std::endl;}
#endif
#else
#define H_PRINT(x)
#endif


#include <vector>
#include <iostream>

#include <opencv2/core/core.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/highgui/highgui.hpp>




double calcDistance(const cv::Point3f &Point_1, const cv::Point3f &Point_2);
std::vector<cv::Point2f> ReadPts2f(const char* filename) ;
bool savePts3f(std::vector<cv::Point3f> Pts, const char* filename);


class MonoCam
{
public:
    // input points
    std::vector<std::vector<cv::Point3f>> objectPoints;
    std::vector<std::vector<cv::Point2f>> imagePoints;

    // output Matrices
    cv::Matx33f cameraMatrix;
    cv::Vec<float,5> distCoeffs;
    //Output rotations and translations
    std::vector<cv::Mat> rvecs, tvecs;

    // flag to specify how calibration is done
    int flag;
    // used in image undistortion
    cv::Mat map1,map2;
    bool mustInitUndistort;

    cv::Size imageSize;
    int subpixlength;
    float square;

    double err;

    MonoCam(int f=CV_CALIB_FIX_K4,bool initundis=true,int subpix=5)
    {
        flag=f;
        mustInitUndistort=initundis;
        subpixlength=subpix;
    }

    int addChessboardPoints(const std::vector<std::string>& filelist, cv::Size & boardSize, float squarelength);

    void addPoints(const std::vector<cv::Point2f>& imageCorners, const std::vector<cv::Point3f>& objectCorners);

    double calibrate();
    // Set the calibration flag
    void setCalibrationFlag(bool radial8CoeffEnabled=false, bool tangentialParamEnabled=false);
    // Remove distortion in an image (after calibration)
    cv::Mat MonoCam::remap(const cv::Mat &image);

};

class BinoCam
{
public:
    MonoCam leftcam;
    MonoCam rightcam;
    double err;

    cv::Mat R;
    cv::Mat E,F;
    cv::Mat T;
    int flag;

    BinoCam(int f= CV_CALIB_USE_INTRINSIC_GUESS  )
    {
        flag=f ;
		set();
    }

    BinoCam(MonoCam &  left, MonoCam & right,int f=CV_CALIB_USE_INTRINSIC_GUESS)
    {
        leftcam=left;
        rightcam=right;
        flag=f;
    }

    void set(int f=CV_CALIB_FIX_K4,bool initundis=true,int subpix=5)
    {
        leftcam.flag=rightcam.flag=f;
        leftcam.mustInitUndistort=rightcam.mustInitUndistort=initundis;
        leftcam.subpixlength = rightcam.subpixlength=subpix;
    }

    bool addPoints(const std::vector<std::string>& leftfilelist, const std::vector<std::string>& rightfilelist, cv::Size & boardSize, float squarelength)
    {
        int l,r;
        l=leftcam.addChessboardPoints(leftfilelist, boardSize, squarelength);
        r=rightcam.addChessboardPoints(rightfilelist, boardSize, squarelength);

        return(l==r);
    }

    void calibrateBoth()
    {
        leftcam.calibrate();
        rightcam.calibrate();
    }


    bool steroCalib();

    //left camera xyz
    bool calcPoints3d(const std::vector<cv::Point2f> & leftpts, const std::vector<cv::Point2f> & rightpts, std::vector< cv::Point3f > & pts3d);

    //r1 r2 is vector  世界坐标系
    bool calcPoints3d_world(cv::Mat  r1, cv::Mat r2,
                            cv::Mat t1, cv::Mat t2,
                            std::vector<cv::Point2f>  leftpts, std::vector<cv::Point2f> rightpts,
                            std::vector< cv::Point3f >&  pts3d);

    //overload
    bool addPoints(std::vector<std::vector<cv::Point2f>> leftPoints,
                   std::vector<std::vector<cv::Point2f>> rightPoints,
                   cv::Size & boardSize,
                   float squarelength);
};





#endif // CAMERACALIBRATOR_H
