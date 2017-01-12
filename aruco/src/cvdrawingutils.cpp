/*****************************
Copyright 2011 Rafael Muñoz Salinas. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are
permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of
conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list
of conditions and the following disclaimer in the documentation and/or other materials
provided with the distribution.

THIS SOFTWARE IS PROVIDED BY Rafael Muñoz Salinas ''AS IS'' AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL Rafael Muñoz Salinas OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those of the
authors and should not be interpreted as representing official policies, either expressed
or implied, of Rafael Muñoz Salinas.
********************************/
#include "stdafx.h"
#include "cvdrawingutils.h"
using namespace cv;
namespace aruco {
	/****
	*
	*
	*
	****/
	void CvDrawingUtils::draw3dAxis(cv::Mat &Image,Marker &m,const CameraParameters &CP)
	{

		float size=m.ssize;
		Mat objectPoints (4,3,CV_32FC1);
		objectPoints.at<float>(0,0)=0;
		objectPoints.at<float>(0,1)=0;
		objectPoints.at<float>(0,2)=0;
		objectPoints.at<float>(1,0)=size;
		objectPoints.at<float>(1,1)=0;
		objectPoints.at<float>(1,2)=0;
		objectPoints.at<float>(2,0)=0;
		objectPoints.at<float>(2,1)=size;
		objectPoints.at<float>(2,2)=0;
		objectPoints.at<float>(3,0)=0;
		objectPoints.at<float>(3,1)=0;
		objectPoints.at<float>(3,2)=size;

		vector<Point2f> imagePoints;
		cv::projectPoints( objectPoints, m.Rvec,m.Tvec, CP.CameraMatrix,CP.Distorsion,   imagePoints);
		//draw lines of different colours
		cv::line(Image,imagePoints[0],imagePoints[1],Scalar(0,0,255,255),1,CV_AA);
		cv::line(Image,imagePoints[0],imagePoints[2],Scalar(0,255,0,255),1,CV_AA);
		cv::line(Image,imagePoints[0],imagePoints[3],Scalar(255,0,0,255),1,CV_AA);
		putText(Image,"x", imagePoints[1],FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0,0,255,255),2);
		putText(Image,"y", imagePoints[2],FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0,255,0,255),2);
		putText(Image,"z", imagePoints[3],FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255,0,0,255),2);
	}

	/****
	*
	*
	*
	****/
	void CvDrawingUtils::draw3dCube(cv::Mat &Image,Marker &m,const CameraParameters &CP)
	{
		Mat objectPoints (8,3,CV_32FC1);
		double halfSize=m.ssize/2;
		objectPoints.at<float>(0,0)=-halfSize;
		objectPoints.at<float>(0,1)=0;
		objectPoints.at<float>(0,2)=-halfSize;
		objectPoints.at<float>(1,0)=halfSize;
		objectPoints.at<float>(1,1)=0;
		objectPoints.at<float>(1,2)=-halfSize;
		objectPoints.at<float>(2,0)=halfSize;
		objectPoints.at<float>(2,1)=0;
		objectPoints.at<float>(2,2)=halfSize;
		objectPoints.at<float>(3,0)=-halfSize;
		objectPoints.at<float>(3,1)=0;
		objectPoints.at<float>(3,2)=halfSize;

		objectPoints.at<float>(4,0)=-halfSize;
		objectPoints.at<float>(4,1)=m.ssize;
		objectPoints.at<float>(4,2)=-halfSize;
		objectPoints.at<float>(5,0)=halfSize;
		objectPoints.at<float>(5,1)=m.ssize;
		objectPoints.at<float>(5,2)=-halfSize;
		objectPoints.at<float>(6,0)=halfSize;
		objectPoints.at<float>(6,1)=m.ssize;
		objectPoints.at<float>(6,2)=halfSize;
		objectPoints.at<float>(7,0)=-halfSize;
		objectPoints.at<float>(7,1)=m.ssize;
		objectPoints.at<float>(7,2)=halfSize;

		vector<Point2f> imagePoints;
		projectPoints( objectPoints, m.Rvec,m.Tvec,  CP.CameraMatrix,CP.Distorsion,   imagePoints);
		//draw lines of different colours
		for (int i=0;i<4;i++)
			cv::circle(Image,imagePoints[i],5,Scalar(0,0,255,255),1);

		//     for (int i=0;i<4;i++)
		//         cv::line(Image,imagePoints[i],imagePoints[(i+1)%4],Scalar(0,0,255,255),1,CV_AA);
		// 
		//     for (int i=0;i<4;i++)
		//         cv::line(Image,imagePoints[i+4],imagePoints[4+(i+1)%4],Scalar(0,255,0,255),1,CV_AA);
		// 
		//     for (int i=0;i<4;i++)
		//         cv::line(Image,imagePoints[i],imagePoints[i+4],Scalar(255,0,0,255),1,CV_AA);

	}


	/****
	*
	*
	*
	****/
	void CvDrawingUtils::draw3dAxis(cv::Mat &Image,Board &B,const CameraParameters &CP)
	{
		Mat objectPoints (4,3,CV_32FC1);
		objectPoints.at<float>(0,0)=0;objectPoints.at<float>(0,1)=0;objectPoints.at<float>(0,2)=0;
		objectPoints.at<float>(1,0)=2*B[0].ssize;objectPoints.at<float>(1,1)=0;objectPoints.at<float>(1,2)=0;
		objectPoints.at<float>(2,0)=0;objectPoints.at<float>(2,1)=2*B[0].ssize;objectPoints.at<float>(2,2)=0;
		objectPoints.at<float>(3,0)=0;objectPoints.at<float>(3,1)=0;objectPoints.at<float>(3,2)=2*B[0].ssize;

		vector<Point2f> imagePoints;
		projectPoints( objectPoints, B.Rvec,B.Tvec, CP.CameraMatrix, CP.Distorsion,   imagePoints);
		//draw lines of different colours
		cv::line(Image,imagePoints[0],imagePoints[1],Scalar(0,0,255,255),2,CV_AA);
		cv::line(Image,imagePoints[0],imagePoints[2],Scalar(0,255,0,255),2,CV_AA);
		cv::line(Image,imagePoints[0],imagePoints[3],Scalar(255,0,0,255),2,CV_AA);

		putText(Image,"X", imagePoints[1],FONT_HERSHEY_SIMPLEX, 1, Scalar(0,0,255,255),2);
		putText(Image,"Y", imagePoints[2],FONT_HERSHEY_SIMPLEX, 1, Scalar(0,255,0,255),2);
		putText(Image,"Z", imagePoints[3],FONT_HERSHEY_SIMPLEX, 1, Scalar(255,0,0,255),2);
	}


	/****
	*
	*
	*
	****/
	void CvDrawingUtils::draw3dCube(cv::Mat &Image,Board &B,const CameraParameters &CP)
	{

		float cubeSize=B[0].ssize;
		float txz=-cubeSize/2;
		Mat objectPoints (8,3,CV_32FC1);
		objectPoints.at<float>(0,0)=txz;objectPoints.at<float>(0,1)=0;objectPoints.at<float>(0,2)=txz;
		objectPoints.at<float>(1,0)=txz+cubeSize;objectPoints.at<float>(1,1)=0;objectPoints.at<float>(1,2)=txz;
		objectPoints.at<float>(2,0)=txz+cubeSize;objectPoints.at<float>(2,1)=cubeSize;objectPoints.at<float>(2,2)=txz;
		objectPoints.at<float>(3,0)=txz;objectPoints.at<float>(3,1)=cubeSize;objectPoints.at<float>(3,2)=txz;

		objectPoints.at<float>(4,0)=txz;objectPoints.at<float>(4,1)=0;objectPoints.at<float>(4,2)=txz+cubeSize;
		objectPoints.at<float>(5,0)=txz+cubeSize;objectPoints.at<float>(5,1)=0;objectPoints.at<float>(5,2)=txz+cubeSize;
		objectPoints.at<float>(6,0)=txz+cubeSize;objectPoints.at<float>(6,1)=cubeSize;objectPoints.at<float>(6,2)=txz+cubeSize;
		objectPoints.at<float>(7,0)=txz;objectPoints.at<float>(7,1)=cubeSize;objectPoints.at<float>(7,2)=txz+cubeSize;

		vector<Point2f> imagePoints;
		projectPoints( objectPoints,B.Rvec,B.Tvec, CP.CameraMatrix, CP.Distorsion,   imagePoints);
		//draw lines of different colours
		for(int i=0;i<4;i++)
			cv::line(Image,imagePoints[i],imagePoints[(i+1)%4],Scalar(0,0,255,255),1,CV_AA);

		for(int i=0;i<4;i++)
			cv::line(Image,imagePoints[i+4],imagePoints[4+(i+1)%4],Scalar(0,0,255,255),1,CV_AA);

		for(int i=0;i<4;i++)
			cv::line(Image,imagePoints[i],imagePoints[i+4],Scalar(0,0,255,255),1,CV_AA);
	}

	/****
	* by Hengbo Tang, 2014.4.24
	* draw axis of both landmark and camera
	* observed from a third static frame
	****/
	void CvDrawingUtils::draw3dRelation(cv::Mat &Image,Board &B,const CameraParameters &CP)
	{
		Image.setTo(Scalar(0,0,0,255));

		float unitLength = 2*B[0].ssize;
		Mat objectPointsAxis (4,3,CV_32FC1);
		objectPointsAxis.at<float>(0,0)=0;objectPointsAxis.at<float>(0,1)=0;objectPointsAxis.at<float>(0,2)=0;
		objectPointsAxis.at<float>(1,0)=unitLength;objectPointsAxis.at<float>(1,1)=0;objectPointsAxis.at<float>(1,2)=0;
		objectPointsAxis.at<float>(2,0)=0;objectPointsAxis.at<float>(2,1)=unitLength;objectPointsAxis.at<float>(2,2)=0;
		objectPointsAxis.at<float>(3,0)=0;objectPointsAxis.at<float>(3,1)=0;objectPointsAxis.at<float>(3,2)=unitLength;

		Mat objectPointsRef;
		float unitLengthRef = 200;
		objectPointsRef = (Mat_<float> (20,3) << 
			-2,0,0,-1,0,0,0,0,0,1,0,0,2,0,0,
			-2,6,0,-1,6,0,0,6,0,1,6,0,2,6,0,
			-3,1,0,-3,2,0,-3,3,0,-3,4,0,-3,5,0,
			3,1,0,3,2,0,3,3,0,3,4,0,3,5,0
			);	
		objectPointsRef = unitLengthRef*objectPointsRef;

		Mat objectPointsCam;
		objectPointsCam = (Mat_<float> (5,3) << 
			0,0,0,
			unitLength,unitLength,1.5*unitLength,
			unitLength,-unitLength,1.5*unitLength,
			-unitLength,-unitLength,1.5*unitLength,
			-unitLength,unitLength,1.5*unitLength
			);	

		Mat objectPointsMark (4,3,CV_32FC1);
		objectPointsMark.at<float>(0,0)=unitLength;objectPointsMark.at<float>(0,1)=0;objectPointsMark.at<float>(0,2)=unitLength;
		objectPointsMark.at<float>(1,0)=-unitLength;objectPointsMark.at<float>(1,1)=0;objectPointsMark.at<float>(1,2)=unitLength;
		objectPointsMark.at<float>(2,0)=-unitLength;objectPointsMark.at<float>(2,1)=0;objectPointsMark.at<float>(2,2)=-unitLength;
		objectPointsMark.at<float>(3,0)=unitLength;objectPointsMark.at<float>(3,1)=0;objectPointsMark.at<float>(3,2)=-unitLength;
				
		Mat RvecStatic, TvecStatic;
		Mat RStatic, Rboard;

		//	Landmark set vertically
		RStatic = (Mat_<float>(3,3) << -1,0,0,0,1,0,0,0,-1);
		TvecStatic = (Mat_<float>(3,1) << 0,-1000,3000);
		
		//	Landmark set horizontally
// 		RStatic = (Mat_<float>(3,3) << -1,0,0,0,0,-1,0,-1,0);
// 		TvecStatic = (Mat_<float>(3,1) << 0,0,3000);
		
		RvecStatic.create(3,1,CV_32FC1);

		Rboard.create(3,3,CV_32FC1);

		Rodrigues(RStatic,RvecStatic);
		Rodrigues(B.Rvec,Rboard);

		Mat Trans31 = Mat::eye(4,4,CV_32FC1);
		Mat Trans21 = Mat::eye(4,4,CV_32FC1);

		RStatic.copyTo(Trans31.colRange(0,3).rowRange(0,3));
		TvecStatic.copyTo(Trans31.colRange(3,4).rowRange(0,3));

		Rboard.copyTo(Trans21.colRange(0,3).rowRange(0,3));
		B.Tvec.copyTo(Trans21.colRange(3,4).rowRange(0,3));

		// 	cout << Trans21 << endl;
		// 	cout << Trans31 << endl;

		Mat Trans32;
		Trans32 = Trans31*Trans21.inv(DECOMP_LU);
		//	cout << Trans32 << endl;
		Mat Rvec32, Tvec32;
		Rvec32.create(3,1,CV_32FC1);
		Tvec32.create(3,1,CV_32FC1);
		Rodrigues(Trans32.colRange(0,3).rowRange(0,3),Rvec32);
		Tvec32 = Trans32.colRange(3,4).rowRange(0,3);

		vector<Point2f> imagePoints;
		Scalar color_R = Scalar(0,0,255,255);
		Scalar color_G = Scalar(0,255,0,255);
		Scalar color_B = Scalar(255,0,0,255);
		Scalar color_W = Scalar(255,255,255,255);

		// draw mark
		projectPoints( objectPointsMark, RvecStatic, TvecStatic, CP.CameraMatrix, CP.Distorsion,   imagePoints);
		cv::line(Image,imagePoints[0],imagePoints[1],color_W,1,CV_AA);
		cv::line(Image,imagePoints[1],imagePoints[2],color_W,1,CV_AA);
		cv::line(Image,imagePoints[2],imagePoints[3],color_W,1,CV_AA);
		cv::line(Image,imagePoints[3],imagePoints[0],color_W,1,CV_AA);

		// draw mark axis
		imagePoints.clear();
		projectPoints( objectPointsAxis, RvecStatic, TvecStatic, CP.CameraMatrix, CP.Distorsion,   imagePoints);
		cv::line(Image,imagePoints[0],imagePoints[1],color_R,1,CV_AA);
		cv::line(Image,imagePoints[0],imagePoints[2],color_G,1,CV_AA);
		cv::line(Image,imagePoints[0],imagePoints[3],color_B,1,CV_AA);

		// draw camera axis
		imagePoints.clear();
		projectPoints( objectPointsAxis, Rvec32, Tvec32, CP.CameraMatrix, CP.Distorsion,   imagePoints);
		cv::line(Image,imagePoints[0],imagePoints[1],color_R,1,CV_AA);
		cv::line(Image,imagePoints[0],imagePoints[2],color_G,1,CV_AA);
		cv::line(Image,imagePoints[0],imagePoints[3],color_B,1,CV_AA);

		// draw camera
		imagePoints.clear();
		projectPoints( objectPointsCam, Rvec32, Tvec32, CP.CameraMatrix, CP.Distorsion,   imagePoints);
		cv::line(Image,imagePoints[0],imagePoints[1],color_W,1,CV_AA);
		cv::line(Image,imagePoints[0],imagePoints[2],color_W,1,CV_AA);
		cv::line(Image,imagePoints[0],imagePoints[3],color_W,1,CV_AA);
		cv::line(Image,imagePoints[0],imagePoints[4],color_W,1,CV_AA);
		cv::line(Image,imagePoints[4],imagePoints[1],color_W,1,CV_AA);
		cv::line(Image,imagePoints[1],imagePoints[2],color_W,1,CV_AA);
		cv::line(Image,imagePoints[2],imagePoints[3],color_W,1,CV_AA);
		cv::line(Image,imagePoints[3],imagePoints[4],color_W,1,CV_AA);

		// draw reference
		imagePoints.clear();
		projectPoints( objectPointsRef, RvecStatic, TvecStatic, CP.CameraMatrix, CP.Distorsion, imagePoints);
		for (int i = 0; i < 5; i ++)
		{
			cv::line(Image,imagePoints[i],imagePoints[i+5],color_G,1,CV_AA);
			cv::line(Image,imagePoints[i+10],imagePoints[i+15],color_G,1,CV_AA);
		}
	}

}
