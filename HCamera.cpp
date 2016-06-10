#include "HCamera.h"
#include <fstream>  

using namespace cv;
using namespace std;

vector<Point3d> ReadOXYT(char *filename )   //去掉了报错信息
{
    FILE *lpFile = fopen(filename, "r");

    if (lpFile == NULL)
        {
            //AfxMessageBox("错误：不能打开世界坐标转换关系文件!");
        }

    char chOXY[4];
    char temp;
    double OriginXY[12];

    for (int i = 0; i < 4; i++)
        fscanf(lpFile, "%c%c%lf%c%lf%c%lf\n", &chOXY[i], &temp, &OriginXY[3*i], &temp, &OriginXY[3*i+1], &temp, &OriginXY[3*i+2]);
    fclose(lpFile);

    vector<Point3d> OXYT(4);

    for (int i = 0; i < 4; i++)
        {
            OXYT.at(i).x=OriginXY[3*i];
            OXYT.at(i).y=OriginXY[3*i+1];
            OXYT.at(i).z=OriginXY[3*i+2];
        }

    if((chOXY[0]=='O' || chOXY[0]=='o') && (chOXY[1]=='X' || chOXY[1]=='x') && (chOXY[2]=='Y' || chOXY[2]=='y') && (chOXY[3]=='T' || chOXY[3]=='t'))
        return OXYT;

    // return TRUE;
    //else
    //AfxMessageBox("错误：坐标系转换关系文件格式错误!");
    //return FALSE;
}

Point3d CalcWorldPoint(const vector<CamPara> &Cams,const vector<Point2d> &ImgPts)  	//两台及以上相机交会，输入相机参数和同一实际点在不同相机中的图像坐标
{
    assert(Cams.size()>=2) ;                    // 两台像机以上
    assert(Cams.size()== ImgPts.size()) ;	    // 相机数目和对应像点数量一致

    const double PI=3.1415926535897932384626433832795028841971693993751;   //精度与姜程序一致

    int N= Cams.size();				    //N个相机及其对应的像点
    Mat A(2* N ,3,CV_64FC1);			//交会公式左边
    Mat B(2* N ,1,CV_64FC1);		   //右边
    Mat x(3,1,CV_64FC1);			   //空间点坐标解

    for (int i=0; i<N; i++)
        {
            double Angx=Cams.at(i).Ax*PI/180;   //角度弧度换算
            double Angy=Cams.at(i).Ay*PI/180;
            double Angz=Cams.at(i).Az*PI/180;

            double r[9];						//旋转矩阵   先绕x轴转Ax，左旋为正向,再绕当前的Y轴转Ay
            r[0]=  cos(Angy)*cos(Angz) ;
            r[1]=  cos(Angx)*sin(Angz) - sin(Angx)*sin(Angy)*cos(Angz);
            r[2]=  sin(Angx)*sin(Angz) + cos(Angx)*sin(Angy)*cos(Angz);
            r[3]= -cos(Angy)*sin(Angz);
            r[4]=  cos(Angx)*cos(Angz) + sin(Angx)*sin(Angy)*sin(Angz);
            r[5]=  sin(Angx)*cos(Angz) - cos(Angx)*sin(Angy)*sin(Angz);
            r[6]= -sin(Angy);
            r[7]= -cos(Angy)*sin(Angx);
            r[8]=  cos(Angx)*cos(Angy);

            double x= (ImgPts.at(i).x - Cams.at(i).Cx)/Cams.at(i).fx ;		//像点归一化坐标
            double y= (ImgPts.at(i).y - Cams.at(i).Cy)/Cams.at(i).fy ;
            double xx=x*x;
            double yy=y*y;
            double xy=x*y;

            ////马颂德-《计算机视觉》中P60给出的模型
            double dex = Cams.at(i).k0*x*(xx+yy) + Cams.at(i).k1*(3*xx+yy) + 2*Cams.at(i).k2*xy + Cams.at(i).k3*(xx+yy);
            double dey = Cams.at(i).k0*y*(xx+yy) + Cams.at(i).k1*2*xy + Cams.at(i).k2*(xx+3*yy) + Cams.at(i).k4*(xx+yy);

            dex *=Cams.at(i).fx ;      // 恢复到像点像素坐标
            dey *=Cams.at(i).fy ;

            double x_ideal   = ImgPts.at(i).x - dex;   //去畸变后像点坐标
            double y_ideal   = ImgPts.at(i).y - dey;

            A.at<double>(i,0)= r[6] *(x_ideal - Cams.at(i).Cx )- (Cams.at(i).fx)*r[0];
            A.at<double>(i,1)= r[7] *(x_ideal - Cams.at(i).Cx )- (Cams.at(i).fx)*r[1];
            A.at<double>(i,2)= r[8] *(x_ideal - Cams.at(i).Cx )- (Cams.at(i).fx)*r[2];
            B.at<double>(i,0)= (Cams.at(i).fx)*( Cams.at(i).Tx)- (Cams.at(i).Tz)*(x_ideal-(Cams.at(i).Cx));

            A.at<double>(i+ N ,0)= r[6] *(y_ideal- Cams.at(i).Cy )- (Cams.at(i).fy)*r[3];
            A.at<double>(i+ N ,1)= r[7] *(y_ideal- Cams.at(i).Cy )- (Cams.at(i).fy)*r[4];
            A.at<double>(i+ N ,2)= r[8] *(y_ideal- Cams.at(i).Cy )- (Cams.at(i).fy)*r[5];
            B.at<double>(i+ N ,0)= (Cams.at(i).fy)*( Cams.at(i).Ty)- (Cams.at(i).Tz)*(y_ideal-(Cams.at(i).Cy));

            /*       //m12投影矩阵，未利用
            double m[12]=
            {
            	Cams.at(i).fx*r[0]+Cams.at(i).Cx*r[6],
            	Cams.at(i).fx*r[1]+Cams.at(i).Cx*r[7],
            	Cams.at(i).fx*r[2]+Cams.at(i).Cx*r[8],
            	Cams.at(i).fx*Cams.at(i).Tx + Cams.at(i).Cx*Cams.at(i).Tz,

            	Cams.at(i).fy*r[3]+Cams.at(i).Cy*r[6],
            	Cams.at(i).fy*r[4]+Cams.at(i).Cy*r[7],
            	Cams.at(i).fy*r[5]+Cams.at(i).Cy*r[8],
            	Cams.at(i).fy*Cams.at(i).Ty + Cams.at(i).Cy*Cams.at(i).Tz,

            	r[6],
            	r[7],
            	r[8],
            	Cams.at(i).Tz
            };   */
        };

    cv::solve(A,B,x,CV_NORMAL);    //opencv 解交会方程
    return Point3d( x.at<double>(0),x.at<double>(1), x.at<double>(2) );
}

CamPara ReadCamPara(char* filename)    //由文件读入像机参数
{
    FILE *fp = fopen(filename, "r");
    assert (fp);

    CamPara cam;

    fscanf(fp, "fx:	%lf\n", &cam.fx );
    fscanf(fp, "fy:	%lf\n", &cam.fy );
    fscanf(fp, "Cx:	%lf\n", &cam.Cx );
    fscanf(fp, "Cy:	%lf\n", &cam.Cy );
    fscanf(fp, "k:	%lf	%lf	%lf	%lf	%lf	\n",&cam.k0,&cam.k1,&cam.k2,&cam.k3,&cam.k4 );
    fscanf(fp, "A:	%lf	%lf	%lf\n", &cam.Ax, &cam.Ay, &cam.Az);
    fscanf(fp, "T:	%lf	%lf	%lf\n", &cam.Tx, &cam.Ty, &cam.Tz);
    fclose(fp);

    return cam;
};

vector<Point2d> ReadPts2d(char* filename)    //由文件读入 单幅图中 多个 像点坐标
{
    FILE *fp = fopen(filename, "r");
    assert (fp);

    int PointsNo, PointIndex;
    double x,y;
    vector<Point2d>  Pts;

    fscanf(fp, "%d\n", &PointsNo);

    for(int i=0; i<PointsNo; i++)
        {
            fscanf(fp, "%d %lf	%lf\n", &PointIndex,&x,&y);
            Pts.push_back(Point2d(x,y));
        }

    fclose(fp);
    return Pts;
}


vector<Point3d> ReadPts3d(char* filename)
{
    FILE *fp = fopen(filename, "r");
    assert (fp);

    int PointsNo, PointIndex;
    double x,y,z;
    vector<Point3d>  Pts;

    fscanf(fp, "%d\n", &PointsNo);

    for(int i=0; i<PointsNo; i++)
        {
            fscanf(fp, "%d %lf	%lf	%lf\n", &PointIndex,&x,&y, &z);
            Pts.push_back(Point3d(x,y,z));
        }

    fclose(fp);
    return Pts;
}


void SaveCamPara(CamPara cam , char* filename)
{
    FILE *fpCam= fopen(filename, "w");

    if (fpCam == NULL)  return;

    //内参数 u0, v0, fx, fy, k0, k1, k2, k3, k4
    fprintf(fpCam, "fx:	%lf\n",cam.fx);
    fprintf(fpCam, "fy:	%lf\n",cam.fy);
    fprintf(fpCam, "Cx:	%lf\n",cam.Cx);
    fprintf(fpCam, "Cy:	%lf\n",cam.Cy);
    fprintf(fpCam, "k:	%lf	%lf	%lf	%lf	%lf	\n",cam.k0, cam.k1, cam.k2,cam.k3, cam.k4);

    //外参数 Ax, Ay, Az, Tx, Ty, Tz
    fprintf(fpCam, "A:	%12.5f	%12.5f	%12.5f\n", cam.Ax, cam.Ay, cam.Az);
    fprintf(fpCam, "T:	%12.5f	%12.5f	%12.5f\n", cam.Tx, cam.Ty, cam.Tz);

    fprintf(fpCam, "MeanError:	%lf	\n", 0);  //  没算//重建误差 MeanError

    fclose(fpCam);
}




double CalcDistance(const Point3d &Point_1, const Point3d &Point_2)
{
    return sqrt( pow((Point_1.x-Point_2.x),2) + pow((Point_1.y-Point_2.y),2) + pow((Point_1.z- Point_2.z),2) );
}


/*
CamPara CalibrationFrom3dPts(const vector<Point3d> &WorldPts, const vector<Point2d> &ImgPts )
{
    assert(WorldPts.size()>=8) ;                    // 像点超过8个
    assert(WorldPts.size()== ImgPts.size()) ;	    // 世界点数目和对应像点数量一致


	CamPara  My_ParaResult={0}; 

    const double PI=3.1415926535897932384626433832795028841971693993751;

    int N= WorldPts.size();
       //
    vector<Point2d> IdealPts(N);
    double m[12],r[9], temp, tempx, tempy,   x,y,xx,yy, xy, totalErr=0;
    double *dex=new double[N], *dey=new double[N];
    Mat CT,D,X9, X3,BT ,BTB,BTBI, eigenvectors,eigenValue ;

    Mat B(2*N,9,CV_64FC1) ;            //见邱治国论文
    Mat C(2*N,3,CV_64FC1) ;            //见邱治国论文
    Mat A(2* N ,5,CV_64FC1);		   //求解 K 方程左边
    Mat DM(2* N ,1,CV_64FC1);		   //求解 K 方程右边  归一化坐标的像差
    Mat X(5,1,CV_64FC1);			   //k解

    for (int i=0; i<N; i++)                //N个点对应2N个方程
        {
            B.at<double>(2*i,0)=  WorldPts.at(i).x;
            B.at<double>(2*i,1)=  WorldPts.at(i).y;
            B.at<double>(2*i,2)=  WorldPts.at(i).z;
            B.at<double>(2*i,3)=  1.0;
            B.at<double>(2*i,4)=  0.0;
            B.at<double>(2*i,5)=  0.0;
            B.at<double>(2*i,6)=  0.0;
            B.at<double>(2*i,7)=  0.0;
            B.at<double>(2*i,8)= -ImgPts.at(i).x;

            C.at<double>(2*i,0)= -ImgPts.at(i).x * WorldPts.at(i).x;
            C.at<double>(2*i,1)= -ImgPts.at(i).x * WorldPts.at(i).y;
            C.at<double>(2*i,2)= -ImgPts.at(i).x * WorldPts.at(i).z;

            B.at<double>(2*i+1,0) =  0.0;
            B.at<double>(2*i+1,1) =  0.0;
            B.at<double>(2*i+1,2) =  0.0;
            B.at<double>(2*i+1,3) =  0.0;
            B.at<double>(2*i+1,4) =   WorldPts.at(i).x;
            B.at<double>(2*i+1,5) =   WorldPts.at(i).y;
            B.at<double>(2*i+1,6) =   WorldPts.at(i).z;
            B.at<double>(2*i+1,7) =  1.0;
            B.at<double>(2*i+1,8) = -ImgPts.at(i).y;

            C.at<double>(2*i+1,0) = -ImgPts.at(i).y * WorldPts.at(i).x;
            C.at<double>(2*i+1,1) = -ImgPts.at(i).y * WorldPts.at(i).y;
            C.at<double>(2*i+1,2) = -ImgPts.at(i).y * WorldPts.at(i).z;
        }

    CT = C.t(), BT=B.t(), BTB=B.t()*B, BTBI= BTB.inv();
    D  = CT*C- CT*B*BTBI*BT*C;
    eigen(D,eigenValue,eigenvectors );
    X3 = eigenvectors.row(2);
    X9 = -BTBI*BT*C*X3.t()  ;

    for (int i=0; i<8  ; i++)	m[i]=X9.at<double>(i);

    m[8] =X3.at<double>(0);
    m[9] =X3.at<double>(1);
    m[10]=X3.at<double>(2);
    m[11]=X9.at<double>(8);

    My_ParaResult.Cx=m[0]*m[8]+m[1]*m[9]+m[2]*m[10];
    My_ParaResult.Cy=m[4]*m[8]+m[5]*m[9]+m[6]*m[10];
    My_ParaResult.fx=sqrt((m[1]*m[10]-m[2]*m[9])*(m[1]*m[10]-m[2]*m[9])+(m[2]*m[8]-m[0]*m[10])*(m[2]*m[8]-m[0]*m[10]) + (m[0]*m[9]-m[1]*m[8])*(m[0]*m[9]-m[1]*m[8]));
    My_ParaResult.fy=sqrt((m[5]*m[10]-m[6]*m[9])*(m[5]*m[10]-m[6]*m[9])+(m[6]*m[8]-m[4]*m[10])*(m[6]*m[8]-m[4]*m[10]) + (m[4]*m[9]-m[5]*m[8])*(m[4]*m[9]-m[5]*m[8]));

    for (int i=0; i<N; i++)
        {
            temp = m[8]*WorldPts.at(i).x+ m[9]*WorldPts.at(i).y + m[10]*WorldPts.at(i).z + m[11] ;
            IdealPts.at(i).x= (m[0]*WorldPts.at(i).x + m[1]*WorldPts.at(i).y + m[2]*WorldPts.at(i).z + m[3] )/ temp ;
            IdealPts.at(i).y= (m[4]*WorldPts.at(i).x + m[5]*WorldPts.at(i).y + m[6]*WorldPts.at(i).z + m[7] )/ temp ;

            dex[i] = ImgPts.at(i).x- IdealPts.at(i).x  ;
            dey[i] = ImgPts.at(i).y- IdealPts.at(i).y  ;

            x= (ImgPts.at(i).x - My_ParaResult.Cx)/My_ParaResult.fx ;		//像点归一化坐标
            y= (ImgPts.at(i).y - My_ParaResult.Cy)/My_ParaResult.fy ;
            xx=x*x;
            yy=y*y;
            xy=x*y;

            A.at<double>(2*i,0) = x*(xx+yy);
            A.at<double>(2*i,1) = 3*xx+yy  ;
            A.at<double>(2*i,2) = 2*xy  ;
            A.at<double>(2*i,3) = xx+yy  ;
            A.at<double>(2*i,4) = 0  ;

            DM.at<double>(2*i,0) = dex[i] / My_ParaResult.fx  ;

            A.at<double>(2*i+1,0) = y*(xx+yy);
            A.at<double>(2*i+1,1) = 2*xy     ;
            A.at<double>(2*i+1,2) = xx+3*yy  ;
            A.at<double>(2*i+1,3) = 0        ;
            A.at<double>(2*i+1,4) = xx+yy    ;

            DM.at<double>(2*i+1,0) = dey[i] / My_ParaResult.fy  ;
        }

    solve(A,DM,X,CV_NORMAL);

    My_ParaResult.k0=X.at<double>(0);
    My_ParaResult.k1=X.at<double>(1);
    My_ParaResult.k2=X.at<double>(2);
    My_ParaResult.k3=X.at<double>(3);
    My_ParaResult.k4=X.at<double>(4);


    for (int i=0; i<N; i++)
        {
            x= (ImgPts.at(i).x - My_ParaResult.Cx)/My_ParaResult.fx ;		//像点归一化坐标
            y= (ImgPts.at(i).y - My_ParaResult.Cy)/My_ParaResult.fy ;
            xx=x*x;
            yy=y*y;
            xy=x*y;

            tempx = My_ParaResult.k0*x*(xx+yy) + My_ParaResult.k1*(3*xx+yy) + 2*My_ParaResult.k2*xy + My_ParaResult.k3*(xx+yy);
            tempy = My_ParaResult.k0*y*(xx+yy) + My_ParaResult.k1*2*xy + My_ParaResult.k2*(xx+3*yy) + My_ParaResult.k4*(xx+yy);

            tempx *=My_ParaResult.fx ;      // 恢复到像点像素坐标
            tempy *=My_ParaResult.fy ;

            tempx =  dex[i]-tempx; //ImgPts.at(i).x- IdealPts.at(i).x
            tempy =  dey[i] -tempy; //ImgPts.at(i).y- IdealPts.at(i).y

            totalErr += tempx * tempx + tempy * tempy;
        }

    double MeanErrors = std::sqrt(totalErr)/N;

    cout<<"err: "<<MeanErrors<<endl;

    My_ParaResult.Tx=(m[3]-My_ParaResult.Cx*My_ParaResult.Tz)/My_ParaResult.fx;
    My_ParaResult.Ty=(m[7]-My_ParaResult.Cy*My_ParaResult.Tz)/My_ParaResult.fy;
    My_ParaResult.Tz= m[11];

    r[0]=(m[0]-My_ParaResult.Cx*r[6])/My_ParaResult.fx;
    r[1]=(m[1]-My_ParaResult.Cx*r[7])/My_ParaResult.fx;
    r[2]=(m[2]-My_ParaResult.Cx*r[8])/My_ParaResult.fx;
    r[3]=(m[4]-My_ParaResult.Cy*r[6])/My_ParaResult.fy;
    r[4]=(m[5]-My_ParaResult.Cy*r[7])/My_ParaResult.fy;
    r[5]=(m[6]-My_ParaResult.Cy*r[8])/My_ParaResult.fy;
    r[6]=m[8];
    r[7]=m[9];
    r[8]=m[10];

    My_ParaResult.Az=atan2(r[3],r[0]);
    if (My_ParaResult.Az<0) My_ParaResult.Az= -My_ParaResult.Az;

    My_ParaResult.Ay=atan2(-r[6], (r[0]*cos(My_ParaResult.Az) - r[3]*sin(My_ParaResult.Az)));
    My_ParaResult.Ax=atan2(r[2]*sin(My_ParaResult.Az)+  r[5]* cos(My_ParaResult.Az) , r[4]*cos(My_ParaResult.Az)+  r[1]* sin(My_ParaResult.Az)     );

    if (My_ParaResult.Ay<-PI/2) My_ParaResult.Ay  += PI;
    if (My_ParaResult.Ay> PI/2) My_ParaResult.Ay  -= PI;

    My_ParaResult.Ax=   My_ParaResult.Ax*180/PI;
    My_ParaResult.Ay=   My_ParaResult.Ay*180/PI;
    My_ParaResult.Az=   My_ParaResult.Az*180/PI;

    // without dis


    //Matx34d ProjMatx(m[0],m[1],m[2],m[3],
    //				 m[4],m[5],m[6],m[7],
    //				 m[8],m[9],m[10],m[11]);

    cout<<endl<< " fx "<<My_ParaResult.fx<< " fy "<< My_ParaResult.fy<<" cx  "<<My_ParaResult.Cx<<" cy "<< My_ParaResult.Cy <<endl;

	cout<<" tx:"<<My_ParaResult.Tx<<"   ty"<<My_ParaResult.Ty<<"   tz"<<My_ParaResult.Tz<<endl;
    //
    return My_ParaResult;

}
*/




CamPara CalibrationFrom3dPts(const vector<Point3d> &WorldPts, const vector<Point2d> &ImgPts )
{
	assert(WorldPts.size()>=8) ;                    // 像点超过8个
	assert(WorldPts.size()== ImgPts.size()) ;	    // 世界点数目和对应像点数量一致


	CamPara  My_ParaResult={0}; 

	const double PI=3.1415926535897932384626433832795028841971693993751;

	int N= WorldPts.size();
	//
	vector<Point2d> IdealPts(N);
	double m[12];
	
	double temp, tempx, tempy,   x,y,xx,yy, xy, totalErr=0;
	double *dex=new double[N], *dey=new double[N];
	Mat CT,D,X9, X3,BT ,BTB,BTBI, eigenvectors,eigenValue ;

	Mat B(2*N,9,CV_64FC1) ;            //见邱治国论文
	Mat C(2*N,3,CV_64FC1) ;            //见邱治国论文
	Mat A(2* N ,5,CV_64FC1);		   //求解 K 方程左边
	Mat DM(2* N ,1,CV_64FC1);		   //求解 K 方程右边  归一化坐标的像差
	Mat X(5,1,CV_64FC1);			   //k解

	for (int i=0; i<N; i++)                //N个点对应2N个方程
	{
		B.at<double>(2*i,0)=  WorldPts.at(i).x;
		B.at<double>(2*i,1)=  WorldPts.at(i).y;
		B.at<double>(2*i,2)=  WorldPts.at(i).z;
		B.at<double>(2*i,3)=  1.0;
		B.at<double>(2*i,4)=  0.0;
		B.at<double>(2*i,5)=  0.0;
		B.at<double>(2*i,6)=  0.0;
		B.at<double>(2*i,7)=  0.0;
		B.at<double>(2*i,8)= -ImgPts.at(i).x;

		C.at<double>(2*i,0)= -ImgPts.at(i).x * WorldPts.at(i).x;
		C.at<double>(2*i,1)= -ImgPts.at(i).x * WorldPts.at(i).y;
		C.at<double>(2*i,2)= -ImgPts.at(i).x * WorldPts.at(i).z;

		B.at<double>(2*i+1,0) =  0.0;
		B.at<double>(2*i+1,1) =  0.0;
		B.at<double>(2*i+1,2) =  0.0;
		B.at<double>(2*i+1,3) =  0.0;
		B.at<double>(2*i+1,4) =   WorldPts.at(i).x;
		B.at<double>(2*i+1,5) =   WorldPts.at(i).y;
		B.at<double>(2*i+1,6) =   WorldPts.at(i).z;
		B.at<double>(2*i+1,7) =  1.0;
		B.at<double>(2*i+1,8) = -ImgPts.at(i).y;

		C.at<double>(2*i+1,0) = -ImgPts.at(i).y * WorldPts.at(i).x;
		C.at<double>(2*i+1,1) = -ImgPts.at(i).y * WorldPts.at(i).y;
		C.at<double>(2*i+1,2) = -ImgPts.at(i).y * WorldPts.at(i).z;
	}

	CT = C.t(), BT=B.t(), BTB=B.t()*B, BTBI= BTB.inv();
	D  = CT*C- CT*B*BTBI*BT*C;
	eigen(D,eigenValue,eigenvectors );
	X3 = eigenvectors.row(2);
	X9 = -BTBI*BT*C*X3.t()  ;

	for (int i=0; i<8  ; i++)	m[i]=X9.at<double>(i);

	m[8] =X3.at<double>(0);
	m[9] =X3.at<double>(1);
	m[10]=X3.at<double>(2);
	m[11]=X9.at<double>(8);

	My_ParaResult.Cx=m[0]*m[8]+m[1]*m[9]+m[2]*m[10];
	My_ParaResult.Cy=m[4]*m[8]+m[5]*m[9]+m[6]*m[10];
	My_ParaResult.fx=sqrt((m[1]*m[10]-m[2]*m[9])*(m[1]*m[10]-m[2]*m[9])+(m[2]*m[8]-m[0]*m[10])*(m[2]*m[8]-m[0]*m[10]) + (m[0]*m[9]-m[1]*m[8])*(m[0]*m[9]-m[1]*m[8]));
	My_ParaResult.fy=sqrt((m[5]*m[10]-m[6]*m[9])*(m[5]*m[10]-m[6]*m[9])+(m[6]*m[8]-m[4]*m[10])*(m[6]*m[8]-m[4]*m[10]) + (m[4]*m[9]-m[5]*m[8])*(m[4]*m[9]-m[5]*m[8]));

	for (int i=0; i<N; i++)
	{
		temp = m[8]*WorldPts.at(i).x+ m[9]*WorldPts.at(i).y + m[10]*WorldPts.at(i).z + m[11] ;
		IdealPts.at(i).x= (m[0]*WorldPts.at(i).x + m[1]*WorldPts.at(i).y + m[2]*WorldPts.at(i).z + m[3] )/ temp ;
		IdealPts.at(i).y= (m[4]*WorldPts.at(i).x + m[5]*WorldPts.at(i).y + m[6]*WorldPts.at(i).z + m[7] )/ temp ;

		dex[i] = ImgPts.at(i).x- IdealPts.at(i).x  ;
		dey[i] = ImgPts.at(i).y- IdealPts.at(i).y  ;

		x= (ImgPts.at(i).x - My_ParaResult.Cx)/My_ParaResult.fx ;		//像点归一化坐标
		y= (ImgPts.at(i).y - My_ParaResult.Cy)/My_ParaResult.fy ;
		xx=x*x;
		yy=y*y;
		xy=x*y;

		A.at<double>(2*i,0) = x*(xx+yy);
		A.at<double>(2*i,1) = 3*xx+yy  ;
		A.at<double>(2*i,2) = 2*xy  ;
		A.at<double>(2*i,3) = xx+yy  ;
		A.at<double>(2*i,4) = 0  ;

		DM.at<double>(2*i,0) = dex[i] / My_ParaResult.fx  ;

		A.at<double>(2*i+1,0) = y*(xx+yy);
		A.at<double>(2*i+1,1) = 2*xy     ;
		A.at<double>(2*i+1,2) = xx+3*yy  ;
		A.at<double>(2*i+1,3) = 0        ;
		A.at<double>(2*i+1,4) = xx+yy    ;

		DM.at<double>(2*i+1,0) = dey[i] / My_ParaResult.fy  ;
	}

	solve(A,DM,X,CV_NORMAL);

	My_ParaResult.k0=X.at<double>(0);
	My_ParaResult.k1=X.at<double>(1);
	My_ParaResult.k2=X.at<double>(2);
	My_ParaResult.k3=X.at<double>(3);
	My_ParaResult.k4=X.at<double>(4);

// 
// 	for (int i=0; i<N; i++)
// 	{
// 		x= (ImgPts.at(i).x - My_ParaResult.Cx)/My_ParaResult.fx ;		//像点归一化坐标
// 		y= (ImgPts.at(i).y - My_ParaResult.Cy)/My_ParaResult.fy ;
// 		xx=x*x;
// 		yy=y*y;
// 		xy=x*y;
// 
// 		tempx = My_ParaResult.k0*x*(xx+yy) + My_ParaResult.k1*(3*xx+yy) + 2*My_ParaResult.k2*xy + My_ParaResult.k3*(xx+yy);
// 		tempy = My_ParaResult.k0*y*(xx+yy) + My_ParaResult.k1*2*xy + My_ParaResult.k2*(xx+3*yy) + My_ParaResult.k4*(xx+yy);
// 
// 		tempx *=My_ParaResult.fx ;      // 恢复到像点像素坐标
// 		tempy *=My_ParaResult.fy ;
// 
// 		tempx =  dex[i]-tempx; //ImgPts.at(i).x- IdealPts.at(i).x
// 		tempy =  dey[i] -tempy; //ImgPts.at(i).y- IdealPts.at(i).y
// 
// 		totalErr += tempx * tempx + tempy * tempy;
// 	}
// 	cout<<totalErr;

	My_ParaResult.Tz= m[11];
	My_ParaResult.Tx=(m[3]-My_ParaResult.Cx* m[11])/My_ParaResult.fx;
	My_ParaResult.Ty=(m[7]-My_ParaResult.Cy* m[11])/My_ParaResult.fy;


	double r_m[9]={0};
	r_m[0]=(m[0]-My_ParaResult.Cx*r_m[6])/My_ParaResult.fx;
	r_m[1]=(m[1]-My_ParaResult.Cx*r_m[7])/My_ParaResult.fx;
	r_m[2]=(m[2]-My_ParaResult.Cx*r_m[8])/My_ParaResult.fx;
	r_m[3]=(m[4]-My_ParaResult.Cy*r_m[6])/My_ParaResult.fy;
	r_m[4]=(m[5]-My_ParaResult.Cy*r_m[7])/My_ParaResult.fy;
	r_m[5]=(m[6]-My_ParaResult.Cy*r_m[8])/My_ParaResult.fy;
	r_m[6]=m[8];
	r_m[7]=m[9];
	r_m[8]=m[10];

	My_ParaResult.Az=atan2(r_m[3],r_m[0]);
	if (My_ParaResult.Az<0) My_ParaResult.Az= -My_ParaResult.Az;

	My_ParaResult.Ay=atan2(-r_m[6], (r_m[0]*cos(My_ParaResult.Az) - r_m[3]*sin(My_ParaResult.Az)));
	My_ParaResult.Ax=atan2(r_m[2]*sin(My_ParaResult.Az)+  r_m[5]* cos(My_ParaResult.Az) , r_m[4]*cos(My_ParaResult.Az)+  r_m[1]* sin(My_ParaResult.Az)     );

	if (My_ParaResult.Ay<-PI/2) My_ParaResult.Ay  += PI;
	if (My_ParaResult.Ay> PI/2) My_ParaResult.Ay  -= PI;

	My_ParaResult.Ax=   My_ParaResult.Ax*180/PI;
	My_ParaResult.Ay=   My_ParaResult.Ay*180/PI;
	My_ParaResult.Az=   My_ParaResult.Az*180/PI;

	// without dis


	//Matx34d ProjMatx(m[0],m[1],m[2],m[3],
	//				 m[4],m[5],m[6],m[7],
	//				 m[8],m[9],m[10],m[11]);

	cout<<endl<< " fx "<<My_ParaResult.fx<< " fy "<< My_ParaResult.fy<<" cx  "<<My_ParaResult.Cx<<" cy "<< My_ParaResult.Cy <<endl;

	cout<<" tx:"<<My_ParaResult.Tx<<"   ty:"<<My_ParaResult.Ty<<"   tz:"<<My_ParaResult.Tz<<endl;
	//


	double MeanErrors = std::sqrt(totalErr)/N;

	cout<<"err: "<<MeanErrors<<endl;







	ofstream out("outm12.txt");  
	if (out.is_open())   
	{  

		for (int i=0;i< 3;i++)
		{
			for (int j=0;j< 4;j++)
			{
				out<<m[4*i+j]<<" ";
			}
			out<<endl;
		}


		for (int i=0;i< 3;i++)
		{
			for (int j=0;j< 4;j++)
			{
				cout<<m[4*i+j]<<" ";
			}
			cout<<endl;
		}





		out.close();  
	}  
 














	return My_ParaResult;


	

}


