/*
// License Agreement (3-clause BSD License)
// Copyright (c) 2015, Klaus Haag, all rights reserved.
// Third party copyrights and patents are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the names of the copyright holders nor the names of the contributors
//   may be used to endorse or promote products derived from this software
//   without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall copyright holders or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
*/

#include <tclap/CmdLine.h>
#include <iostream>
#include "kcf_tracker.hpp"
#include "tracker_run.hpp"
#include "kcf_debug.hpp"

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>

#include"KCF/BoundingBox.h"
#include<cmath>

class KcfTrackerRun : public TrackerRun
{
public:
    KcfTrackerRun() : TrackerRun("KCFcpp")
    {}

    virtual ~KcfTrackerRun()
    {
    }

    virtual cf_tracking::CfTracker* parseTrackerParas(TCLAP::CmdLine& cmd, int argc, char** argv)
    {
        cf_tracking::KcfParameters paras;
        TCLAP::SwitchArg debugOutput("v", "debug", "Output Debug info!", cmd, false);
        TCLAP::SwitchArg originalVersion("", "original_version", "Parameters and performance as close to the KCF VOT version as possible.", cmd, false);
        TCLAP::SwitchArg originalParametersWithScaleFilter("", "original_parameters_scale_filter", "KCF VOT version parameters with DSST scale filter.", cmd, false);
        TCLAP::ValueArg<int> templateSize("", "para_template_size", "template size", false, paras.templateSize, "integer", cmd);
        TCLAP::ValueArg<int> cellSize("", "para_cell_size", "cell size of fhog", false, paras.cellSize, "integer", cmd);
        TCLAP::ValueArg<double> padding("", "para_padding", "padding around the target", false, paras.padding, "double", cmd);
        TCLAP::ValueArg<double> lambda("", "para_lambda", "regularization factor", false, paras.lambda, "double", cmd);
        TCLAP::ValueArg<double> outputSigmaFactor("", "para_output_sigma_factor", "spatial bandwidth of the target", false, paras.outputSigmaFactor, "double", cmd);
        TCLAP::ValueArg<double> scaleStep("", "para_vot_scale_step", "scale_step", false, paras.votScaleStep, "double", cmd);
        TCLAP::ValueArg<double> scaleWeight("", "para_vot_scale_weight", "scale_weight", false, paras.votScaleWeight, "double", cmd);
        TCLAP::ValueArg<double> interpFactor("", "para_interpFactor", "interpolation factor for learning", false, paras.interpFactor, "double", cmd);
        TCLAP::ValueArg<double> kernelSigma("", "para_kernel_sigma", "sigma for Gaussian kernel", false, paras.kernelSigma, "double", cmd);
        TCLAP::ValueArg<double> psrThreshold("", "para_psr_threshold", "if psr is lower than "
            "psr threshold, target is assumed to be lost", false, paras.psrThreshold, "double", cmd);
        TCLAP::ValueArg<int> psrPeakDel("", "para_psr_peak_del", "amount of pixels that are "
            "deleted for psr calculation around the peak (1 means that a window of 3 by 3 is "
            "deleted; 0 means that max response is deleted; 2 * peak_del + 1 pixels are deleted)", false, paras.psrPeakDel, "integer", cmd);
        TCLAP::SwitchArg useDsstScale("", "para_use_dsst_scale", "Uses the DSST scale filter for scale estimation. "
            "Disable for more speed!", cmd, paras.useDsstScaleEstimation);
        TCLAP::ValueArg<double> scaleSigmaFactor("", "para_dsst_sigma_factor", "DSST: spatial bandwidth of the target", false, paras.scaleSigmaFactor, "double", cmd);
        TCLAP::ValueArg<double> scaleEstimatorStep("", "para_dsst_scale_step", "DSST: scale step", false, paras.scaleEstimatorStep, "double", cmd);
        TCLAP::ValueArg<double> scaleLambda("", "para_dsst_lambda", "DSST: regularization for scale estimation", false, paras.scaleLambda, "double", cmd);
        TCLAP::ValueArg<int> scaleCellSize("", "para_dsst_cell_size", "DSST: hog cell size for scale estimation", false, paras.scaleCellSize, "integer", cmd);
        TCLAP::ValueArg<int> numberOfScales("", "para_dsst_scales", "DSST: number of scales", false, paras.numberOfScales, "integer", cmd);
        TCLAP::SwitchArg enableTrackingLossDetection("", "para_enable_tracking_loss", "Enables the tracking loss detection!", cmd, paras.enableTrackingLossDetection);

        cmd.parse(argc, argv);

        paras.padding = padding.getValue();
        paras.lambda = lambda.getValue();
        paras.outputSigmaFactor = outputSigmaFactor.getValue();
        paras.votScaleStep = scaleStep.getValue();
        paras.votScaleWeight = scaleWeight.getValue();
        paras.templateSize = templateSize.getValue();
        paras.interpFactor = interpFactor.getValue();
        paras.kernelSigma = kernelSigma.getValue();
        paras.cellSize = cellSize.getValue();
        paras.psrThreshold = psrThreshold.getValue();
        paras.psrPeakDel = psrPeakDel.getValue();
        paras.enableTrackingLossDetection = enableTrackingLossDetection.getValue();

        paras.useDsstScaleEstimation = useDsstScale.getValue();
        paras.scaleSigmaFactor = scaleSigmaFactor.getValue();
        paras.scaleEstimatorStep = scaleEstimatorStep.getValue();
        paras.scaleLambda = scaleLambda.getValue();
        paras.scaleCellSize = scaleCellSize.getValue();
        paras.numberOfScales = numberOfScales.getValue();

        if (originalVersion.getValue() || originalParametersWithScaleFilter.getValue())
        {
            paras.padding = 1.5;
            paras.lambda = 0.0001;
            paras.outputSigmaFactor = 0.1;
            paras.votScaleStep = 1.05;
            paras.votScaleWeight = 0.95;
            paras.templateSize = 100;
            paras.interpFactor = 0.012;
            paras.kernelSigma = 0.6;
            paras.cellSize = 4;
            paras.pixelPadding = 0;

            paras.enableTrackingLossDetection = false;

            if (originalParametersWithScaleFilter.getValue())
            {
                paras.useVotScaleEstimation = false;
                paras.useDsstScaleEstimation = true;
            }
            else
            {
                paras.useVotScaleEstimation = true;
                paras.useDsstScaleEstimation = false;
            }

            paras.useFhogTranspose = false;
        }

        if (debugOutput.getValue())
        {
            setTrackerDebug(&_debug);
            return new cf_tracking::KcfTracker(paras, &_debug);
        }

        return new cf_tracking::KcfTracker(paras);
    }

private:
    cf_tracking::KcfDebug<cf_tracking::KcfTracker::T> _debug;
};

cv::Mat rgbimage;
cv::Mat depthimage;
void rgbimageCallback(const sensor_msgs::ImageConstPtr& msg)
    {  
      cv::Mat image;
      try
      {
       image = cv_bridge::toCvShare(msg, "bgr8")->image;
       image.copyTo(rgbimage);
       //cv::resize(rgbimage,rgbimage,cv::Size(640,480));
      // int width = rgbimage.cols;
      // ROS_INFO("image = %d",width);
       //cv::imshow("turtlebot_image", rgbimage);
      // cv::waitKey(10);
      //cv::imshow("turtlebot_image", cv_bridge::toCvShare(msg, "bgr8")->image);
     
     }
     catch (cv_bridge::Exception& e)
     {
       ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
     }
   }
void depthimageCallback(const sensor_msgs::ImageConstPtr& msg)
    {
      cv::Mat image;
      try
      {
        image = cv_bridge::toCvShare(msg,sensor_msgs::image_encodings::TYPE_32FC1)->image;
        image.copyTo(depthimage);
     //cv::resize(depthimage,depthimage,cv::Size(320,240));
       cv::imshow("KCF_depthimage",depthimage);
     //  ROS_INFO("Could not convert from '%s' to 'TYPE_32FC1'.", msg->encoding.c_str());
      //  float dist_val = depthimage.at<float>(240,320);
      //  ROS_INFO("the distance of middle point is = %f",dist_val);
      //  cv::waitKey(10);
     }
     catch (cv_bridge::Exception& e)
     {
       ROS_ERROR("Could not convert from '%s' to 'TYPE_32FC1'.", msg->encoding.c_str());
     }
   }

int main(int argc,  char** argv)
{
     ros::init(argc, argv, "KCFcpp");
     ros::NodeHandle nh;
     cv::namedWindow("Draw Bounding Box");
     //cv::namedWindow("turtlebot_image");
     cv::namedWindow("KCF_depthimage");
     cv::namedWindow("KCFcpp");
     cv::startWindowThread();

  image_transport::ImageTransport it(nh); 
  image_transport::Subscriber sub_rgb= it.subscribe("/camera/rgb/image_color", 1, rgbimageCallback);
  image_transport::Subscriber sub_depth = it.subscribe("/camera/depth_registered/sw_registered/image_rect", 3, depthimageCallback);
  ros::Publisher BoundingBox_pub = nh.advertise<KCF::BoundingBox>("BoundingBox",100);
 // ros::Rate loop_rate(10);
  KCF::BoundingBox msg;


   KcfTrackerRun mainObj;
   Parameters _paras;
  // std::cout<<_paras.device<<std::endl;
  // _paras.device = -1;
   _paras = mainObj.parseCmdArgs(argc, argv);
    cv::waitKey(700);
    //ros::spinOnce();
      
     //if (mainObj.init(rgbimage) == false)
        //    exit (-1);

     float dist_val[4] ;
     int depthpointsnum = 4;

   /*视频显示框的中心点*/
   #define VIDEO_CENTER_X 320
   #define VIDEO_CENTER_Y 240
   /*中心点横纵坐标允许的误差偏移*/
   #define ERROR_OFFSET_X 5
   #define ERROR_OFFSET_Y 2

   /*turtleBot运动线速度角速度默认值*/
   #define CONTROL_SPEED 0.1
   #define CONTROL_TURN 0.1
   /*线速度控制和与深度的比例*/
   #define CONTROL_SPEED_RATIO 0.01
   /*角速度控制和偏移距离的比例*/
   #define CONTROL_TURN_RATIO 0.3
   /*线速度和角速度最大值*/
   //#define CONTROL_SPEED_MAX 0.4
   #define CONTROL_SPEED_MAX 0.3
   #define CONTROL_TURN_MAX 0.5

   float depth_value = 0;
   double controlSpeed = 0;//turtlebot线速度控制
   double controlTurn = 0;//turtlebot角速度控制
   int depthmini = 1500;
   int depthmax = 1600;
   msg.controlSpeed = 0;
   msg.controlTurn = 0;
   ros::Rate loop_rate(10);
  //cv::Mat rgb_image_temp;
   //cv::VideoCapture capture(0);
    while(ros::ok())
   {
     ros::spinOnce();
     //capture.read(rgb_image_temp);
     //rgbimage.copyTo(rgb_image_temp);
     mainObj.update(rgbimage);
           //exit(-1);
     if(mainObj._targetOnFrame)
      {
         //publish the BoundingBox msg to "BoundingBox" topic.
      msg.x = mainObj._boundingBox.x;
      msg.y = mainObj._boundingBox.y;
      msg.width = mainObj._boundingBox.width;
      msg.height = mainObj._boundingBox.height;

      dist_val[0] =depthimage.at<float>(mainObj._boundingBox.y+mainObj._boundingBox.height/3 , mainObj._boundingBox.x+mainObj._boundingBox.width/3) ;
      dist_val[1] = depthimage.at<float>(mainObj._boundingBox.y+mainObj._boundingBox.height/3 , mainObj._boundingBox.x+2*mainObj._boundingBox.width/3) ;
      dist_val[2] = depthimage.at<float>(mainObj._boundingBox.y+2*mainObj._boundingBox.height/3 , mainObj._boundingBox.x+mainObj._boundingBox.width/3) ;
      dist_val[3] = depthimage.at<float>(mainObj._boundingBox.y+2*mainObj._boundingBox.height/3 , mainObj._boundingBox.x+2*mainObj._boundingBox.width/3) ;
      for(int i=0;i<4;i++)
      {
       if(dist_val[i] == 0 )
          --depthpointsnum;
      }
      ROS_INFO("the depth points number is = %d",depthpointsnum);
      ROS_INFO("the dist_val[0] is %f",dist_val[0]);
      depth_value = (dist_val[0]+dist_val[1]+dist_val[2]+dist_val[3])/depthpointsnum;
      msg.z = depth_value*1000;
      msg.x = mainObj._boundingBox.x + mainObj._boundingBox.width/2;
      depthpointsnum = 4;
    //  BoundingBox_pub. publish(msg);
      ROS_INFO_STREAM("msg.x =" <<msg.x
                      <<" msg.y = "<<msg.y
                      <<" msg.z = "<<msg.z);
            //控制turtleBot角速度模块
      if (  ( msg.x > (VIDEO_CENTER_X - ERROR_OFFSET_X) ) && ( msg.x < (VIDEO_CENTER_X + ERROR_OFFSET_X) )  )
      controlTurn = 0;
      else if (  ( msg.x < (VIDEO_CENTER_X - ERROR_OFFSET_X)) || ( msg.x == (VIDEO_CENTER_X - ERROR_OFFSET_X))  )
      controlTurn = std::min((CONTROL_TURN * CONTROL_TURN_RATIO * (VIDEO_CENTER_X - ERROR_OFFSET_X - msg.x)),CONTROL_TURN_MAX);
      else if (  ( msg.x > (VIDEO_CENTER_X + ERROR_OFFSET_X)) || ( msg.x == (VIDEO_CENTER_X + ERROR_OFFSET_X))  )
      controlTurn = (-1)*(std::min((CONTROL_TURN * CONTROL_TURN_RATIO * (msg.x - (VIDEO_CENTER_X + ERROR_OFFSET_X))),CONTROL_TURN_MAX));
      else
      controlTurn = 0;
      //控制turtleBot线速度模块
      if ( (msg.z > depthmini ) && (msg.z < depthmax) )
      controlSpeed = 0;
      else if ( (msg.z < depthmini) || (msg.z == depthmini))
      controlSpeed = (-1)*std::min(CONTROL_SPEED*CONTROL_SPEED_RATIO*(depthmini - msg.z),CONTROL_SPEED_MAX);
      else if ( (msg.z > depthmax) || (msg.z == depthmax))
      controlSpeed = std::min(CONTROL_SPEED*CONTROL_SPEED_RATIO*(msg.z - depthmax),CONTROL_SPEED_MAX);
      else
      controlSpeed = 0;

       msg.controlTurn = controlTurn;
      msg.controlSpeed = controlSpeed;
      BoundingBox_pub. publish(msg);
      ROS_INFO("controlSpeed, controlTurn:%lf %lf",msg.controlSpeed,msg.controlTurn) ;
      }
      if (cvWaitKey(33) == 'q')
        break;
     // loop_rate.sleep();

   }

    //mainObj.mainObj._boundingBox = _paras.initBb;
    //mainObj._isTrackerInitialzed = false;
   // cv::waitKey(0);

    return 0;
}
