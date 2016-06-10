#include <fstream>
#include <iostream>
#include <iomanip>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "H_CameraCalibrator.h"
#include"HCamera.h"
using namespace cv;
using namespace std;



int main()
{
    BinoCam mybino ;
    mybino.set(CV_CALIB_FIX_K2,true,5);

    vector<string> leftfilelist,rightfilelist;

    for (int i=1; i<=4; i++)
        {
            std::stringstream leftstr,rightstr;

            leftstr << "D:\\TOOLBOX\\hhhh\\l\\" << std::setw(1) << std::setfill('0') << i << ".bmp";
            std::cout << leftstr.str() << std::endl;
            leftfilelist.push_back(leftstr.str());

            rightstr << "D:\\TOOLBOX\\hhhh\\r\\" << std::setw(1) << std::setfill('0') << i << ".bmp";
            std::cout << rightstr.str() << std::endl;
            rightfilelist.push_back(rightstr.str());
        }

	MonoCam mymo;
 //   mymo.addChessboardPoints(leftfilelist,Size(11,8),20);
	//mymo.calibrate();
	H_PRINT(mymo.cameraMatrix);
	H_PRINT(mymo.distCoeffs);

	std::vector<std::vector<cv::Point3f>> obj(1);
	std::vector<std::vector<cv::Point2f>> imagePoints;
	imagePoints.push_back( ReadPts2f("E:\\qqq\\oldoxyt\\mf2\\A\\ImgPt.pxy")  );
	 std::vector<cv::Point3d>  tmp= ReadPts3d("E:\\qqq\\oldoxyt\\mf2\\NewWorldPtHJ.XYZ")  ;

	 obj[0].resize(11);
	 for (size_t i = 0; i < tmp.size(); i++)
	 {
		 obj[0][i].x = tmp[i].x;
		 obj[0][i].y= tmp[i].y;
		 obj[0][i].z = tmp[i].z;
	 }



	mymo.cameraMatrix = Matx33f(4444, 0, 3333,
		0, 4444, 1555,
		0, 0, 1);

	mymo.distCoeffs = cv::Vec<float, 5>(0, 0, 0, 0, 0);
 
	H_PRINT(imagePoints[0][0])
	H_PRINT(mymo.cameraMatrix);
	H_PRINT(mymo.distCoeffs);

	 
	float err;
	std::vector<cv::Mat> rvecs, tvecs;
	err= calibrateCamera(obj, // the 3D points
		imagePoints,  // the image points
		cv::Size(4096,3072),    // image sizemymo.imageSize
		mymo.cameraMatrix, // output camera matrix
		mymo.distCoeffs,   // output distortion matrix
		 rvecs,  tvecs, // Rs, Ts
		 CV_CALIB_ZERO_TANGENT_DIST + CV_CALIB_USE_INTRINSIC_GUESS + CV_CALIB_FIX_K1 + CV_CALIB_FIX_K2 + CV_CALIB_FIX_K3,
		cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 99, 0.001)
		);

	H_PRINT(mymo.cameraMatrix);
	H_PRINT(mymo.distCoeffs);
	H_PRINT(err);
//////////*/CV_CALIB_USE_INTRINSIC_GUESS
	 

    //vector<Point2f> p1,p2;
    //vector<double> std1,std2;

    //for (int i=0; i<leftfilelist.size(); i++)
    //    {
    //        cout<<"   no: "<<i+1<<endl;

    //        p1=mybino.leftcam.imagePoints[i];
    //        p2=mybino.rightcam.imagePoints[i];

    //        //cout<<p1<<endl;

    //        vector<Point3f> p;
    //        mybino.calcPoints3d(p1,p2,p);


    //        // 		cout<<" i   pt  "<<endl<<mybino.objpts<<endl;//
    //        // 		obj<<"F:\\hhhhh\\"<<std::setw(1) << setfill('0') <<i<<".txt";//
    //        // 		SavePts3f(mybino.objpts,   obj.str().c_str());

    //        double d01,d012;
    //        d01= calcDistance( *(p.begin()),  *(p.begin()+1)  );
    //        d012=calcDistance(*(p.begin()),   *(p.begin()+10)        ) ;

    //        cout<< " 0-1 dis: "<<d01 <<endl;
    //        cout<< " 0-9: "<<d012  <<endl;

    //        std1.push_back(d01);
    //        std2.push_back(d012);


    //    }

    //Mat m1,m2,s1,s2;
    //meanStdDev(std1,m1,s1);
    //meanStdDev(std2,m2,s2);
 



//////////////////////////////////

//
//     BinoCam newbino(CV_CALIB_FIX_INTRINSIC);
//     newbino.leftcam.cameraMatrix=mybino.leftcam.cameraMatrix;
//     newbino.leftcam.distCoeffs=mybino.leftcam.distCoeffs;
//     newbino.rightcam.cameraMatrix=mybino.rightcam.cameraMatrix;
//     newbino.rightcam.distCoeffs=mybino.rightcam.distCoeffs;
//
//     newbino.addPoints(mybino.leftcam.imagePoints, mybino.rightcam.imagePoints,Size(11,8),20  );
//     //
//     newbino.steroCalib();
//
//     cout<<"R,T...   "<<newbino.R<<"   ,  "<<endl<<newbino.T<<endl;
//     cout<<"stereo err  "<< newbino.err<<endl<<endl;
//


    system("pause");
    return 0;
}