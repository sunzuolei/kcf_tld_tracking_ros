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

using tld::Config;
using tld::Gui;
using tld::Settings;
using namespace cv;
using namespace tld;
#include "TLDUtil.h"
#include "Trajectory.h"
#include "opencv2/imgproc/imgproc.hpp"
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

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
    //cv::waitKey(30);
    }
    catch (cv_bridge::Exception& e)
    {
    ROS_ERROR("Could not convert from '%s' to 'TYPE_32FC1'.", msg->encoding.c_str());
    }
}

int main(int argc, char **argv)
{

    Main *main = new Main();
    Config config;
    ImAcq *imAcq = imAcqAlloc();
    Gui *gui = new Gui();

    main->gui = gui;
    main->imAcq = imAcq;

    /*if(config.init(argc, argv) == PROGRAM_EXIT)
    {
        return EXIT_FAILURE;
    }*/

    config.configure(main);

    srand(main->seed);

    imAcqInit(imAcq);

    if(main->showOutput)
    {
        gui->init();
    }
    printf("Before do Work!\n");
   // main->doWork();
     ros::init(argc, argv, "opentld");
     ros::NodeHandle nh;
     cv::startWindowThread();
     image_transport::ImageTransport it(nh);
     image_transport::Subscriber sub = it.subscribe("/camera/rgb/image_color", 1, rgbimageCallback);
     image_transport::Subscriber sub_depth = it.subscribe("/camera/depth_registered/sw_registered/image_rect", 1, depthimageCallback);
     cv::namedWindow("depthimage");
     //cv::namedWindow("image");
     waitKey(700);
    Trajectory trajectory;
    //IplImage *img = imAcqGetImg(imAcq);
    ros::spinOnce();

    IplImage *img;
    *img = IplImage(rgbimage);
    //cvShowImage("depthimage",img);
    //waitKey(30);
    //Mat grey(img->height, img->width, CV_8UC1);
    //cvtColor(cvarrToMat(img), grey, CV_BGR2GRAY);
    Mat colorImage = cvarrToMat(img,true);
    Mat grey;
  
    cv::cvtColor(colorImage, grey, CV_BGR2GRAY);
    //cv::imshow("image_test",rgbimage);
    //cv::waitKey(30);

    main->tld->detectorCascade->imgWidth = grey.cols;
    main->tld->detectorCascade->imgHeight = grey.rows;
    main->tld->detectorCascade->imgWidthStep = grey.step;

    if(!main->showTrajectory)
    {    
        trajectory.init(main->trajectoryLength);
    }

    if(main->selectManually)
    {
   
        CvRect box;

        if(getBBFromUser(img, box, gui) == PROGRAM_EXIT)
        {
           // return;
        }

        if(main->initialBB == NULL)
        {
            main->initialBB = new int[4];
        }

        main->initialBB[0] = box.x;
        main->initialBB[1] = box.y;
        main->initialBB[2] = box.width;
        main->initialBB[3] = box.height;
    }

    FILE *resultsFile = NULL;

    if(main->printResults != NULL)
    {
        resultsFile = fopen(main->printResults, "w");
        if(!resultsFile)
        {
            fprintf(stderr, "Error: Unable to create results-file \"%s\"\n", main->printResults);
            exit(-1);
        }
    }

    bool reuseFrameOnce = false;
    bool skipProcessingOnce = false;

    if(main->loadModel && main->modelPath != NULL)
    {
        main->tld->readFromFile(main->modelPath);
        reuseFrameOnce = true;
    }
    else if(main->initialBB != NULL)
    {
        Rect bb = tldArrayToRect(main->initialBB);

        printf("Starting at %d %d %d %d\n", bb.x, bb.y, bb.width, bb.height);

        main->tld->selectObject(grey, &bb);
        skipProcessingOnce = true;
        reuseFrameOnce = true;
    }
    imAcq->currentFrame++;
    while(imAcqHasMoreFrames(imAcq))
    {
        double tic = cvGetTickCount();
        if(!reuseFrameOnce)
        {
            //cvReleaseImage(&img);
           // img = imAcqGetImg(imAcq);
            ros::spinOnce();
            imAcq->currentFrame++;
            *img = IplImage(rgbimage);


            //cvtColor(cvarrToMat(img), grey, CV_BGR2GRAY);
            colorImage = cvarrToMat(img,true);
            cv::cvtColor(colorImage, grey, CV_BGR2GRAY);
        }


        if(!skipProcessingOnce)
        {
            //main->tld->processImage(cvarrToMat(img));
            main->tld->processImage(colorImage);
        }
        else
        {
            skipProcessingOnce = false;
        }

        if(main->printResults != NULL)
        {
            if(main->tld->currBB != NULL)
            {
                fprintf(resultsFile, "%d %.2d %.2d %.2d %.2d %f\n", imAcq->currentFrame - 1, main->tld->currBB->x, main->tld->currBB->y, main->tld->currBB->width, main->tld->currBB->height, main->tld->currConf);
            }
            else
            {
                fprintf(resultsFile, "%d NaN NaN NaN NaN NaN\n", imAcq->currentFrame - 1);
            }
        }

        double toc = (cvGetTickCount() - tic) / cvGetTickFrequency();

        toc = toc / 1000000;

        float fps = 1 / toc;

        int confident = (main->tld->currConf >= main->threshold) ? 1 : 0;

        if(main->showOutput || main->saveDir != NULL)
        {   
            char string[128];

            char learningString[10] = "";

            if(main->tld->learning)
            {
                strcpy(learningString, "Learning");
            }

            sprintf(string, "#%d,Posterior %.2f; fps: %.2f, #numwindows:%d, %s", imAcq->currentFrame - 1, main->tld->currConf, fps, main->tld->detectorCascade->numWindows, learningString);
            CvScalar yellow = CV_RGB(255, 255, 0);
            CvScalar blue = CV_RGB(0, 0, 255);
            CvScalar black = CV_RGB(0, 0, 0);
            CvScalar white = CV_RGB(255, 255, 255);
            CvScalar red= CV_RGB(255, 0, 0);

            if(main->tld->currBB != NULL)
            {
                CvScalar rectangleColor = (confident) ? blue : yellow;
                cvRectangle(img,main->tld->currBB->tl(), main->tld->currBB->br(), rectangleColor, 8, 8, 0);
                if(main->showTrajectory)
                {
                    CvPoint center = cvPoint(main->tld->currBB->x+main->tld->currBB->width/2, main->tld->currBB->y+main->tld->currBB->height/2);
                    cvLine(img, cvPoint(center.x-2, center.y-2), cvPoint(center.x+2, center.y+2), rectangleColor, 2);
                    cvLine(img, cvPoint(center.x-2, center.y+2), cvPoint(center.x+2, center.y-2), rectangleColor, 2);
                    trajectory.addPoint(center, rectangleColor);
                }
            }
            else if(main->showTrajectory)
            {
                trajectory.addPoint(cvPoint(-1, -1), cvScalar(-1, -1, -1));
            }

            if(main->showTrajectory)
            {
                trajectory.drawTrajectory(img);
            }

            CvFont font;
            cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, .5, .5, 0, 1, 8);
            //cvRectangle(img, cvPoint(0, 0), cvPoint(img->width, 50), CV_RGB(0, 0, 0), CV_FILLED, 8, 0);
            cvPutText(img, string, cvPoint(25, 25), &font, CV_RGB(255, 0, 0));

            if(main->showForeground)
            {

                for(size_t i = 0; i < main->tld->detectorCascade->detectionResult->fgList->size(); i++)
                {
                    Rect r = main->tld->detectorCascade->detectionResult->fgList->at(i);
                    cvRectangle(img, r.tl(), r.br(), white, 1);
                }

            }


            if(main->showOutput)
            {
                gui->showImage(img);
                char key = gui->getKey();

                if(key == 'q') break;

                if(key == 'b')
                {

                    ForegroundDetector *fg = main->tld->detectorCascade->foregroundDetector;

                    if(fg->bgImg.empty())
                    {
                        fg->bgImg = grey.clone();
                    }
                    else
                    {
                        fg->bgImg.release();
                    }
                }

                if(key == 'c')
                {
                    //clear everything
                    main->tld->release();
                }

                if(key == 'l')
                {
                    main->tld->learningEnabled = !main->tld->learningEnabled;
                    printf("LearningEnabled: %d\n", main->tld->learningEnabled);
                }

                if(key == 'a')
                {
                    main->tld->alternating = !main->tld->alternating;
                    printf("alternating: %d\n", main->tld->alternating);
                }

                if(key == 'e')
                {
                    main->tld->writeToFile(main->modelExportFile);
                }

                if(key == 'i')
                {
                    main->tld->readFromFile(main->modelPath);
                }

                if(key == 'r')
                {
                    CvRect box;

                    if(getBBFromUser(img, box, gui) == PROGRAM_EXIT)
                    {
                        break;
                    }

                    Rect r = Rect(box);

                    main->tld->selectObject(grey, &r);
                }
            }

            if(main->saveDir != NULL)
            {
                char fileName[256];
                sprintf(fileName, "%s/%.5d.png", main->saveDir, imAcq->currentFrame - 1);

                cvSaveImage(fileName, img);
            }
        }

        if(reuseFrameOnce)
        {
            reuseFrameOnce = false;
        }
    }

    cvReleaseImage(&img);
    img = NULL;

    if(main->exportModelAfterRun)
    {
        main->tld->writeToFile(main->modelExportFile);
    }

    if(resultsFile)
    {
        fclose(resultsFile);
    }
    delete main;
    main = NULL;
    delete gui;
    gui = NULL;

    return EXIT_SUCCESS;
}
