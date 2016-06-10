
#include "H_CameraCalibrator.h"

using namespace std;
using namespace cv;

int MonoCam::addChessboardPoints(const std::vector<std::string>& filelist,
                                 cv::Size & boardSize,
                                 float squarelength)
{
    objectPoints.clear();
    imagePoints.clear();

    square = squarelength;

    // the points on the chessboard
    std::vector<cv::Point2f>   imageCorners;
    std::vector<cv::Point3f> objectCorners(boardSize.area());

    for (int i=0; i<boardSize.height; i++)
        {
            for (int j=0; j<boardSize.width; j++)
                {
                    objectCorners[i*boardSize.width+j] = cv::Point3f(i*square, j*square, 0.0f);
                }
        }

    // 2D Image points:
    cv::Mat image; // to contain chessboard image
    int successes = 0;
    bool found;

    for (int i=0; i<filelist.size(); i++)
        {
            image = cv::imread(filelist[i],0);

            float scale;
            if (image.size().width>2000)
                {
                    scale =0.5;
                    Mat imagescale;
                    Size dsize;
                    resize(image,imagescale,dsize,scale,scale);
                    found = cv::findChessboardCorners(imagescale, boardSize, imageCorners,  CV_CALIB_CB_ADAPTIVE_THRESH );

                    int num=imageCorners.size();
                    for (int i=0; i<num ; i++)
                        {
                            imageCorners[i]=imageCorners[i]*(1.0f/scale);
                        }

                }
            else
                {
                    found = cv::findChessboardCorners(image, boardSize, imageCorners, CV_CALIB_CB_ADAPTIVE_THRESH );
                }

            // cout<<"  find corner "<< imageCorners[0].x <<"  "<<  imageCorners[0].y  <<endl;
            //cv::namedWindow("Corners on Chessboard",0);
            //cv::drawChessboardCorners(image, boardSize, imageCorners, found);
            //cv::imshow("Corners on Chessboard", image);
            //cv::waitKey(90);
            H_PRINT(filelist[i] );

            if (!found)
                {
                    H_PRINT("not found");
                    H_PRINT(filelist[i] );
                    continue;

                }

            cv::cornerSubPix(image, imageCorners,
                             cvSize(subpixlength,subpixlength),
                             cv::Size(-1,-1),
                             cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS,
                                              50,0.005));     // min accuracy

            // find4QuadCornerSubpix(image, imageCorners,cvSize(subpixlength,subpixlength)  );

            if (imageCorners.size() == boardSize.area())
                {
                    ////cout<<"cor s   are   "<<imageCorners.size()<<"  "<<boardSize.area()<<endl;
                    addPoints(imageCorners, objectCorners);
                    successes++;
                }

//cout<<"sub    -------image cor "<< imageCorners[0].x <<"  "<<  imageCorners[0].y  <<endl;
// 			cv::namedWindow("Corners on Chessboard",0);
// 			cv::drawChessboardCorners(image, boardSize, imageCorners, found);
// 			cv::imshow("Corners on Chessboard", image);
// 			cv::waitKey(30);
        }

    imageSize=image.size();

    return successes;
}

void MonoCam::addPoints(const std::vector<cv::Point2f>& imageCorners, const std::vector<cv::Point3f>& objectCorners)
{

    // 2D image points from one view
    imagePoints.push_back(imageCorners);
    // corresponding 3D scene points
    objectPoints.push_back(objectCorners);
}

double MonoCam::calibrate()
{
    mustInitUndistort= true;

    err=calibrateCamera(objectPoints, // the 3D points
                        imagePoints,  // the image points
                        imageSize,    // image size
                        cameraMatrix, // output camera matrix
                        distCoeffs,   // output distortion matrix
                        rvecs, tvecs, // Rs, Ts
                        flag,
                        cv::TermCriteria(cv::TermCriteria::MAX_ITER +cv::TermCriteria::EPS,	100,0.001)
                       );

    // set options//	,CV_CALIB_USE_INTRINSIC_GUESS);

    return err;
}

cv::Mat MonoCam::remap(const cv::Mat &image)
{

    cv::Mat undistorted;

    if (mustInitUndistort)   // called once per calibration
        {
            cv::initUndistortRectifyMap(
                cameraMatrix,  // computed camera matrix
                distCoeffs,    // computed distortion matrix
                cv::Mat(),     // optional rectification (none)
                cv::Mat(),     // camera matrix to generate undistorted
                image.size(),  // size of undistorted
                CV_32FC1,      // type of output map
                map1, map2);   // the x and y mapping functions

            mustInitUndistort= false;
        }

    // Apply mapping functions
    cv::remap(image, undistorted, map1, map2,cv::INTER_LINEAR); // interpolation type

    return undistorted;
}


// Set the calibration options
// 8radialCoeffEnabled should be true if 8 radial coefficients are required (5 is default)
// tangentialParamEnabled should be true if tangeantial distortion is present
void MonoCam::setCalibrationFlag(bool radial8CoeffEnabled, bool tangentialParamEnabled)
{

    // Set the flag used in cv::calibrateCamera()
    flag = 0;
    if (!tangentialParamEnabled) flag += CV_CALIB_ZERO_TANGENT_DIST;
    if (radial8CoeffEnabled) flag += CV_CALIB_RATIONAL_MODEL;
}


bool BinoCam::steroCalib()
{
    if (leftcam.imagePoints.size()!= rightcam.imagePoints.size())
        {
            return false;
        }

    if (leftcam.imagePoints.size() ==0)
        {
            return false;
        }

    err=stereoCalibrate( leftcam.objectPoints,
                         leftcam.imagePoints,
                         rightcam.imagePoints,
                         leftcam.cameraMatrix,
                         leftcam.distCoeffs,
                         rightcam.cameraMatrix,
                         rightcam.distCoeffs,
                         leftcam.imageSize,
                         R,T,E,F,
                         cv::TermCriteria(cv::TermCriteria::MAX_ITER +cv::TermCriteria::EPS,100,0.001),
                         flag);

    R.convertTo(R, CV_64F);
    T.convertTo(T, CV_64F);
    E.convertTo(E, CV_64F);
    F.convertTo(F, CV_64F); //////for safe
}


bool BinoCam::calcPoints3d( const std::vector<cv::Point2f> & leftpts, const std::vector<cv::Point2f> & rightpts, std::vector< cv::Point3f > & pts3d )
{
    int n=leftpts.size();
    if (n==0  || n!=rightpts.size())
        {
            return false;
        }

//      cv::Mat pt_3d_h(1,n,CV_64FC4);
//      cv::Mat leftprojMatr(3,4,CV_64FC1), rightprojMatr(3,4,CV_64FC1);
//
// 	 leftprojMatr= Mat::zeros(3,4,CV_64FC1);
// 	 leftprojMatr.at<double>(0,0)=leftprojMatr.at<double>(1,1)=leftprojMatr.at<double>(2,2)=1.0;
//
// 	 Mat submat = rightprojMatr.colRange(0, 3);
// 	 Mat(R).copyTo(submat);
// 	 submat = rightprojMatr.colRange(3, 4);
// 	 Mat(T).copyTo(submat);


    std::vector<cv::Point2f> leftundis(n),rightundis(n);

	//H_PRINT(leftpts[0]);

    undistortPoints(leftpts,leftundis,leftcam.cameraMatrix, leftcam.distCoeffs);
    undistortPoints(rightpts,rightundis,rightcam.cameraMatrix, rightcam.distCoeffs);

	//H_PRINT ( (leftundis[0].x    )*leftcam.cameraMatrix(0,0) +leftcam.cameraMatrix(0,2) );

    //cout<<"left undis "<< leftundis<<endl<<endl;

    Mat xt,xtt,lefthomo,righthomo;
    convertPointsToHomogeneous(leftundis,lefthomo);
    convertPointsToHomogeneous(rightundis,righthomo);

    xt=lefthomo.reshape(1);
    xtt=righthomo.reshape(1);

    //cout<<"xt  "<<xt<<endl<<endl;

    xt=xt.t();
    xtt=xtt.t();
    xt.convertTo(xt, CV_64F);
    xtt.convertTo(xtt,CV_64F);

    //cout<<"xt"<<xt<<endl<<endl;
    //cout<<"xtt "<<xtt<<endl<<endl;

    Mat u=R*xt;

    //cout<<"u"<<u<<endl<<endl;

    Mat n_xt2(1,n,CV_64FC1) , n_xtt2(1,n,CV_64FC1);

    for (int i=0; i<n; i++)
        {
            n_xt2.at<double>(0,i)= xt.col(i).dot(xt.col(i));
            n_xtt2.at<double>(0,i)= xtt.col(i).dot(xtt.col(i));
        }

    //cout<<"nxt2"<<n_xt2<<endl<<endl;

    Mat DD,tmp1(1,n,CV_64FC1) ,tmp2(1,n,CV_64FC1) ;
    multiply(n_xt2,n_xtt2,tmp1);
    for (int i=0; i<n ; i++)
        {
            tmp2.at<double>(0,i)= u.col(i).dot(xtt.col(i));
            tmp2.at<double>(0,i)=tmp2.at<double>(0,i)*tmp2.at<double>(0,i);
        }
    DD=tmp1-tmp2;

    //cout<<"DD"<<DD<<endl<<endl;

    Mat dot_uT(1,n,CV_64FC1), dot_xttT(1,n,CV_64FC1) ,dot_xttu(1,n,CV_64FC1)  ;
    for (int i=0; i<n ; i++)
        {
            dot_uT.at<double>(0,i)   = u.col(i).dot(T);
            dot_xttT.at<double>(0,i) = xtt.col(i).dot(T);
            dot_xttu.at<double>(0,i) = u.col(i).dot(xtt.col(i));
        }

    Mat NN1,nn1_1,nn1_2;
    multiply(dot_xttu,dot_xttT,nn1_1);
    multiply(n_xtt2,dot_uT,nn1_2);
    NN1=nn1_1-nn1_2;

    //cout<<"nn1"<<NN1<<endl<<endl;

    Mat NN2,nn2_1,nn2_2;
    multiply(n_xt2,dot_xttT,nn2_1);
    multiply(dot_uT,dot_xttu,nn2_2);

    NN2=nn2_1-nn2_2;

    //cout<<"nn2"<<NN2<<endl<<endl;

    Mat Zt,Ztt;
    divide(NN1,DD,Zt);
    divide(NN2,DD,Ztt);

    //cout<<"zt"<<Zt<<endl<<endl;

    Mat repzt(3,n,CV_64FC1), repztt(3,n,CV_64FC1),X1,X2;

    Zt.copyTo(repzt.row(0));
    Zt.copyTo(repzt.row(1));
    Zt.copyTo(repzt.row(2));

    Ztt.copyTo(repztt.row(0));
    Ztt.copyTo(repztt.row(1));
    Ztt.copyTo(repztt.row(2));

    //cout<<"repzt"<<repzt<<endl<<endl;
    //cout<<"repztt"<<repztt<<endl<<endl;

    multiply(xt,repzt,X1);

    //cout<<"X1"<<X1<<endl<<endl;

    Mat tmp3;
    multiply(xtt,repztt,tmp3);

    //cout<<"tmp3"<<tmp3<<endl<<endl;
    //cout<<"T  "<<T <<endl<<endl;

    Mat T_vect(3,n,CV_64FC1);
    for (int i=0; i<n; i++)
        {
            T.copyTo(T_vect.col(i));
        }

    //cout<<"Tvect  "<<T_vect<<endl<<endl;

    X2=R.t();

    //cout<< "R.t"<< X2<<endl<<endl;

    X2=X2*(tmp3-T_vect);

    //cout<<"x2"<<X2<<endl<<endl;

    Mat XL =(X1 + X2)*0.5   ;

    //cout<< " xl t "<<XL.t()<<endl<<endl;

    XL=XL.t() ;
    XL.convertTo(XL,CV_32F);
    XL=XL.reshape(3);
    pts3d = Mat_<Point3f>(XL);



    return 1;
//
//      for (int i=0; i<n ; i++)
//          {
//              leftundis[n].x = leftpts[i].x*leftcamera.cameraMatrix(0,0) +leftcamera.cameraMatrix(0,2);
//              leftundis[n].y = leftpts[i].y*leftcamera.cameraMatrix(0,1) +leftcamera.cameraMatrix(1,2);
//
// 			 rightundis[n].x = rightpts[i].x*rightcamera.cameraMatrix(0,0) +rightcamera.cameraMatrix(0,2);
// 			 rightundis[n].y = rightpts[i].y*rightcamera.cameraMatrix(0,1) +rightcamera.cameraMatrix(1,2);
// 			         //cv::triangulatePoints(projMatrleft, projMatrright     );
//
//          };



// 	 Mat pt_set1_pt,pt_set2_pt;
// 	 undistortPoints(_pt_set1_pt, pt_set1_pt, K, distcoeff);
// 	 undistortPoints(_pt_set2_pt, pt_set2_pt, K, distcoeff);

    //triangulate
// 	 Mat pt_set1_pt_2r = pt_set1_pt.reshape(1, 2);
// 	 Mat pt_set2_pt_2r = pt_set2_pt.reshape(1, 2);


// 	 Mat pt_3d_h;
//
//
//
//
//     cv::triangulatePoints(leftprojMatr,rightprojMatr,leftpts,rightpts,pt_3d_h);

}

vector<Point2f> ReadPts2f(const char* filename)    //由文件读入 单幅图中 多个 像点坐标
{
    FILE *fp = fopen(filename, "r");
    assert (fp);

    int PointsNo, PointIndex;
    double x,y;
    vector<Point2f>  Pts;

    fscanf(fp, "%d\n", &PointsNo);

    for(int i=0; i<PointsNo; i++)
        {
            fscanf(fp, "%d %lf	%lf\n", &PointIndex,&x,&y);
            Pts.push_back(Point2f(x,y));
        }

    fclose(fp);
    return Pts;
}


double calcDistance(const Point3f &Point_1, const Point3f &Point_2)
{
    return sqrt( pow((Point_1.x-Point_2.x),2) + pow((Point_1.y-Point_2.y),2) + pow((Point_1.z- Point_2.z),2) );
}

bool savePts3f(vector<Point3f> Pts, const char* filename)
{
    FILE *fp = fopen(filename, "w");
    if (fp==NULL)
        {
            return false;
        }

    int PointsNo, PointIndex;
    PointsNo = Pts.size();

    if (PointsNo==0)
        {
            return false;
        }

    fprintf(fp, "%d\n", PointsNo);

    for(int i=0; i<PointsNo; i++)
        {
            fprintf(fp, "%d	%lf	%lf	%lf\n", i,Pts[i].x,Pts[i].y,Pts[i].z);
        }

    fclose(fp);

    return true;
}




bool BinoCam::calcPoints3d_world(cv::Mat  r1, cv::Mat r2,
                                 cv::Mat t1, cv::Mat t2,
                                 vector<cv::Point2f>  leftpts, vector<cv::Point2f> rightpts,
                                 vector< cv::Point3f >&  pts3d)
{
    Mat RM1,RM2,projMatr1,projMatr2 ;

    if ( ( r1.rows*r1.cols==3 ) && (r2.rows*r2.cols==3)    )
        {
            Rodrigues(r1,RM1);
            Rodrigues(r2,RM2);
        }
    else if(  ( r1.rows*r1.cols==9 ) && (r2.rows*r2.cols==9)   )
        {
            RM1=r1;
            RM2=r2;
        }
    else
        return false;

    int n=leftpts.size();

    if( n !=  rightpts.size() || n==0 )
        {
            return false;
        }

    Mat I1= Mat::zeros(3,4, CV_64FC1);
    Mat I2= Mat::zeros(3,4, CV_64FC1);
    Mat E1= Mat::zeros(4,4, CV_64FC1);
    Mat E2= Mat::zeros(4,4, CV_64FC1);

    Mat(leftcam.cameraMatrix).copyTo(  Mat(I1, Rect(0, 0, 3, 3))  );
    Mat(rightcam.cameraMatrix).copyTo( Mat(I2, Rect(0, 0, 3, 3))  );

    RM1.copyTo( Mat(E1, Rect(0, 0, 3, 3))  );
    RM2.copyTo( Mat(E2, Rect(0, 0, 3, 3))  );

    E1.at<double>(3,3)=E2.at<double>(3,3)=1;
    Mat(t1).copyTo(  Mat(E1, Rect(3, 0, 1, 3))  );
    Mat(t2).copyTo(  Mat(E2, Rect(3, 0, 1, 3))  );

    projMatr1= I1*E1;
    projMatr2= I2*E2;

    Mat points4D, points3D_M;

    //5:magic number,for opencv bug
	if (n<5)
	{
		leftpts.insert(leftpts.end(),5,cv::Point2f(0.0,0.0) );
		rightpts.insert(rightpts.end(),5,cv::Point2f(0.0,0.0) );
	}
	
    triangulatePoints( projMatr1,  projMatr2, leftpts, rightpts,  points4D);
	
    //H_PRINT(points4D);
    points4D= points4D.t();
    convertPointsFromHomogeneous(points4D,points3D_M);

    //H_PRINT(points3D_M);

    vector< cv::Point3f >  p3= Mat_<Point3d>(points3D_M);
    pts3d.assign(p3.begin(),p3.begin()+n);

    return 1;
};

bool BinoCam::addPoints(std::vector<std::vector<cv::Point2f>> leftPoints,
                        std::vector<std::vector<cv::Point2f>> rightPoints,
                        cv::Size & boardSize,
                        float squarelength)
{
    leftcam.objectPoints.clear();
    leftcam.imagePoints.clear();
    rightcam.objectPoints.clear();
    rightcam.imagePoints.clear();

    int num=leftPoints.size();
    int total=boardSize.area();

    if (num != rightPoints.size() || num == 0  )
        {
            return false;
        }

    // the points on the chessboard
    std::vector<cv::Point3f> objectCorners(boardSize.area());

    for (int i=0; i<boardSize.height; i++)
        {
            for (int j=0; j<boardSize.width; j++)
                {
                    objectCorners[i*boardSize.width+j] = cv::Point3f(i*squarelength, j*squarelength, 0.0f);
                }
        }

    for (int i=0; i< num; i++)
        {
            if (leftPoints[i].size() == total && rightPoints[i].size() == total )
                {
                    ////cout<<"cor s   are   "<<imageCorners.size()<<"  "<<boardSize.area()<<endl;
                    leftcam.addPoints(leftPoints[i], objectCorners);
                    rightcam.addPoints(rightPoints[i], objectCorners);
                }
            else
                {
                    return false;
                }
        }
    return true;
}
