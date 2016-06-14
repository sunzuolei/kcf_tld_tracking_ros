/*  Copyright 2011 AIT Austrian Institute of Technology
*
*   This file is part of OpenTLD.
*
*   OpenTLD is free software: you can redistribute it and/or modify
*   it under the terms of the GNU General Public License as published by
*    the Free Software Foundation, either version 3 of the License, or
*   (at your option) any later version.
*
*   OpenTLD is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.
*
*   You should have received a copy of the GNU General Public License
*   along with OpenTLD.  If not, see <http://www.gnu.org/licenses/>.
*
*/

/**
  * @author Georg Nebehay
  */

#include "Main.h"
#include "Config.h"
#include "ImAcq.h"
#include "Gui.h"
#include <random>

#include "TLDUtil.h"
#include "Trajectory.h"
#include "opencv2/imgproc/imgproc.hpp"
using namespace tld;
using namespace cv;

#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

//using tld::Config;
//using tld::Gui;
//using tld::Settings;

cv::Mat rgbimage;
cv::Mat depthimage;
void rgbimageCallback(const sensor_msgs::ImageConstPtr& msg)
{     
      cv::Mat image;
      try
      {
        image = cv_bridge::toCvShare(msg, "bgr8")->image;

        image.copyTo(rgbimage);
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
    cv::imshow("depthimage",depthimage);
    }
    catch (cv_bridge::Exception& e)
    {
    ROS_ERROR("Could not convert from '%s' to 'TYPE_32FC1'.", msg->encoding.c_str());
    }
}

int main(int argc, char **argv)
{
     //waitKey(500);
    Main *main = new Main();
    Config config;
    ImAcq *imAcq = imAcqAlloc();
    Gui *gui = new Gui();

    main->gui = gui;
    main->imAcq = imAcq;

   /* if (config.init(argc, argv) == PROGRAM_EXIT)
    {    
        return EXIT_FAILURE;
    }*/

    config.configure(main);
    main->tld->seed = main->seed;
    imAcqInit(imAcq);
    if (main->showOutput)
    {
        gui->init();
    printf("showOutput!\n");
    }

    printf("Before do Work!\n");

     ros::init(argc, argv, "cftld");
     ros::NodeHandle nh;
     cv::startWindowThread();
     image_transport::ImageTransport it(nh);
     image_transport::Subscriber sub = it.subscribe("/camera/rgb/image_color", 1, rgbimageCallback);
     image_transport::Subscriber sub_depth = it.subscribe("/camera/depth_registered/sw_registered/image_rect", 1, depthimageCallback);
     cv::namedWindow("depthimage");
     //namedWindow("image_test");
     waitKey(700);
     //ros::spinOnce();
    // *img = IplImage(rgbimage);
   // main->doWork();
     Trajectory trajectory;
    //IplImage *img = imAcqGetImg(imAcq);
    ros::spinOnce();

    //imshow("image_test",rgbimage);
    //waitKey(0);
    IplImage *img ;
    *img = IplImage(rgbimage);
    //cvShowImage("image_test",img);
    //waitKey(0);
    Mat colorImage = cvarrToMat(img, true);
    //imshow("image_test",colorImage);
   // waitKey(30);
     if (colorImage.channels() == 1)
        cv::cvtColor(colorImage, colorImage, cv::COLOR_GRAY2BGR);

    if (!main->showTrajectory)
    {
        trajectory.init(main->trajectoryLength);
    }
    if (main->selectManually)
    {
        CvRect box;

        if (getBBFromUser(img, box, gui) == PROGRAM_EXIT)
        {
           // return;
        }

        if (main->initialBB == NULL)
        {
            main->initialBB = new int[4];
        }

        main->initialBB[0] = box.x;
        main->initialBB[1] = box.y;
        main->initialBB[2] = box.width;
        main->initialBB[3] = box.height;
    }

    FILE *resultsFile = NULL;

    if (main->printResults != NULL)
    {
        resultsFile = fopen(main->printResults, "w");
        if (!resultsFile)
        {
            fprintf(stderr, "Error: Unable to create results-file \"%s\"\n", main->printResults);
            exit(-1);
        }
    }

    bool reuseFrameOnce = false;
    bool skipProcessingOnce = false;
    bool paused = false;
    bool step = false;
    double tic = 0;
    double toc = 0;

    if (main->initialBB != NULL)
    {
        Rect bb = tldArrayToRect(main->initialBB);

        printf("Starting at %d %d %d %d\n", bb.x, bb.y, bb.width, bb.height);
        tic = static_cast<double>(getTickCount());
        main->tld->selectObject(colorImage, &bb);
        toc = getTickCount() - tic;
        skipProcessingOnce = true;
        reuseFrameOnce = true;
    }
     imAcq->currentFrame++;
    while (imAcqHasMoreFrames(imAcq))
    {    
        //printf("start tracking!\n");
        if (!reuseFrameOnce && (!paused || step))
        {
            //cvReleaseImage(&img);
            ros::spinOnce();
            //img = imAcqGetImg(imAcq);
            imAcq->currentFrame++;
            *img = IplImage(rgbimage);
            colorImage = cvarrToMat(img, true);

            if (colorImage.channels() == 1)
                cv::cvtColor(colorImage, colorImage, cv::COLOR_GRAY2BGR);

            if (img == NULL)
            {
                printf("current image is NULL, assuming end of input.\n");
                break;
            }
        }

        if (!skipProcessingOnce && (!paused || step))
        {
            tic = static_cast<double>(getTickCount());
            main->tld->processImage(colorImage);
            toc = getTickCount() - tic;
        }
        else
        {
            skipProcessingOnce = false;
        }

        float fps = static_cast<float>(getTickFrequency() / toc);

        if (main->printResults != NULL)
        {
            if (main->tld->currBB != NULL)
            {
                fprintf(resultsFile, "%d, %.2d, %.2d, %.2d, %.2d, %f, %f\n", imAcq->currentFrame - 1,
                    main->tld->currBB->x, main->tld->currBB->y, main->tld->currBB->width, main->tld->currBB->height, main->tld->currConf,
                    fps);
            }
            else
            {
                fprintf(resultsFile, "%d, NaN, NaN, NaN, NaN, NaN, %f\n", imAcq->currentFrame - 1, fps);
            }
        }

        if (main->showOutput || main->saveDir != NULL)
        {
            char string[128];
            char learningString[10] = "";

            if (paused && step)
                step = false;

            if (main->tld->learning)
            {
                strcpy(learningString, "Learning");
            }

            sprintf(string, "#%d, fps: %.2f, #numwindows:%d, %s", imAcq->currentFrame - 1,
                fps, main->tld->detectorCascade->numWindows, learningString);
            CvScalar yellow = CV_RGB(255, 255, 0);
            CvScalar blue = CV_RGB(0, 0, 255);
            CvScalar black = CV_RGB(0, 0, 0);
            CvScalar white = CV_RGB(255, 255, 255);
            CvScalar red = CV_RGB(255, 0, 0);

            if (main->tld->currBB != NULL)
            {
                CvScalar rectangleColor = red;
                cvRectangle(img, main->tld->currBB->tl(), main->tld->currBB->br(), rectangleColor, 2, 8, 0);
            }

            CvFont font;
            cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, .5, .5, 0, 2, 8);
            //cvRectangle(img, cvPoint(0, 0), cvPoint(img->width, 50), black, CV_FILLED, 8, 0);
            cvPutText(img, string, cvPoint(25, 25), &font, CV_RGB(255, 0, 0));

            if (main->showOutput)
            {    
                gui->showImage(img);
                char key = gui->getKey();

                if (key == 'q')
                    break;

                if (key == 'p')
                    paused = !paused;

                if (paused && key == 's')
                    step = true;

                if (key == 'c')
                {
                    //clear everything
                    main->tld->release();
                }

                if (key == 'l')
                {
                    main->tld->learningEnabled = !main->tld->learningEnabled;
                    printf("LearningEnabled: %d\n", main->tld->learningEnabled);
                }

                if (key == 'a')
                {
                    main->tld->alternating = !main->tld->alternating;
                    printf("alternating: %d\n", main->tld->alternating);
                }

                if (key == 'r')
                {
                    CvRect box;

                    if (getBBFromUser(img, box, gui) == PROGRAM_EXIT)
                    {
                        break;
                    }

                    Rect r = Rect(box);
                    main->tld->selectObject(colorImage, &r);
                }
            }

            if (main->saveDir != NULL)
            {
                char fileName[256];
                sprintf(fileName, "%s/%.5d.png", main->saveDir, imAcq->currentFrame - 1);

                cvSaveImage(fileName, img);
            }
        }

        if (reuseFrameOnce)
        {
            reuseFrameOnce = false;
        }
    }

    cvReleaseImage(&img);
    img = NULL;

    if (resultsFile)
    {
        fclose(resultsFile);
    }

    delete main;
    main = NULL;
    delete gui;
    gui = NULL;

    return EXIT_SUCCESS;
}
