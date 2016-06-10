#ifndef _HCAMERA_H
#define _HCAMERA_H

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <sstream> 

 

using namespace std;
using namespace cv;

class CamPara                      //相机内外参-马颂德-计算机视觉 P60模型
{public:
double fx,fy,Cx,Cy,k0,k1,k2,k3,k4,Ax,Ay,Az,Tx,Ty,Tz;
};

Point3d CalcWorldPoint(const vector<CamPara>& Cams,const vector<Point2d>&  ImgPts);

CamPara ReadCamPara(char* filename);

vector<Point2d> ReadPts2d(char* filename);
vector<Point3d> ReadPts3d(char* filename);

vector<Point3d> ReadOXYT(char *filename );

CamPara CalibrationFrom3dPts(const vector<Point3d> & WorldPts, const vector<Point2d> & ImgPts );

double CalcDistance(const Point3d &Point_1,const Point3d &Point_2) ;

void SaveCamPara(CamPara cam , char* filename);

 
#endif