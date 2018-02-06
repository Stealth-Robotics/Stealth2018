//----------------------------------------------------------------------------
//
//  $Workfile: ColorLocator.cpp$
//
//  $Revision: X$
//
//  Project:    FRC4089
//
//                            Copyright (c) 2017
//                          Cedarcrest High School
//                            All Rights Reserved
//
//  Modification History:
//  $Log:
//  $
//
//----------------------------------------------------------------------------
#include <chrono>
#include <cstdio>
#include <thread>
#include <iostream>
#include <stdio.h>
#include <time.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "ntcore.h"

using namespace cv;
using namespace std;

//----------------------------------------------------------------------------
//  Global constants
//----------------------------------------------------------------------------
const int MAX_BUFFER = 200;
const int MAX_BLOBS = 5;

//----------------------------------------------------------------------------
//  Global Variables
//----------------------------------------------------------------------------
Mat gImgOriginal;           // Image read from camera
Mat gImgHSV;                // Image turned into a HSV image
Mat gImgThresholded;        // The black and white image
Mat gImgThresholdedCopy;    // The copy of the threshold to be consumed by findContour

vector<vector<Point> > gContours;  //The blobs that we have found
vector<Vec4i> gHierarchy;          // The hierarchy of those blobs

int gLargestCont[MAX_BLOBS];        // The largest block
double gLargestArea[MAX_BLOBS];     // The largest area of that blob

int gCamWidth = 0;                  // The width of the camera
int gCamHeight = 0;                 // The height of the camera

clock_t gStartTime;                 // the FPS we are processing images
int gFramesPerSecond = 0;
int gCurFramesPerSeconds = 0;

int gX = -1;                          // Global Location vars
int gY = -1;
int gWidth = -1;
int gHeight = -1;
int gArea = -1;

int gCount = 0;

//----------------------------------------------------------------------------
//  Purpose:
//         Main function
//
// Notes:
//     None.
//  
//----------------------------------------------------------------------------
int main(int argc, char** argv)
{
  char stringBuffer[MAX_BUFFER];
  FILE* fp = fopen("/home/pi/Locate.txt","w");
  std::this_thread::sleep_for(std::chrono::seconds(20));
  nt::StartClient("roborio-4089-frc.local", 1735);
  std::this_thread::sleep_for(std::chrono::seconds(2));
  printf("After NetTables\n");

  //********************************************************
  //*
  //*  Get the web cam and display the settings
  //*
  //********************************************************
  VideoCapture webCam(0); //capture the video from web cam

  bool camOpened = webCam.isOpened(); 

  if (!camOpened)  // if not success, exit program
  {
    camOpened = webCam.isOpened();  
  }

  fprintf(fp,"After Cam\n");

  printf("CV_CAP_PROP_POS_MSEC     :%f\n", webCam.get(CV_CAP_PROP_POS_MSEC));
  printf("CV_CAP_PROP_FRAME_WIDTH  :%f\n", webCam.get(CV_CAP_PROP_FRAME_WIDTH));
  printf("CV_CAP_PROP_FRAME_HEIGHT :%f\n", webCam.get(CV_CAP_PROP_FRAME_HEIGHT));
  printf("CV_CAP_PROP_FPS          :%f\n", webCam.get(CV_CAP_PROP_FPS));
  printf("CV_CAP_PROP_FOURCC       :%f\n", webCam.get(CV_CAP_PROP_FOURCC));
  printf("CV_CAP_PROP_FORMAT       :%f\n", webCam.get(CV_CAP_PROP_FORMAT));
  printf("CV_CAP_PROP_MODE         :%f\n", webCam.get(CV_CAP_PROP_MODE));
  printf("CV_CAP_PROP_BRIGHTNESS   :%f\n", webCam.get(CV_CAP_PROP_BRIGHTNESS));
  printf("CV_CAP_PROP_CONTRAST     :%f\n", webCam.get(CV_CAP_PROP_CONTRAST));
  printf("CV_CAP_PROP_SATURATION   :%f\n", webCam.get(CV_CAP_PROP_SATURATION));
  printf("CV_CAP_PROP_EXPOSURE     :%f\n", webCam.get(CV_CAP_PROP_EXPOSURE));
  printf("CV_CAP_PROP_CONVERT_RGB  :%f\n", webCam.get(CV_CAP_PROP_CONVERT_RGB));
  printf("CV_CAP_PROP_AUTO_EXPOSURE:%f\n", webCam.get(CV_CAP_PROP_AUTO_EXPOSURE));

  fprintf(fp,"After prop\n");

  webCam.set(CV_CAP_PROP_POS_MSEC, -1.000000);
  webCam.set(CV_CAP_PROP_POS_FRAMES, -1.000000);
  webCam.set(CV_CAP_PROP_POS_AVI_RATIO, -1.000000);
  webCam.set(CV_CAP_PROP_FRAME_WIDTH, 640.000000);
  webCam.set(CV_CAP_PROP_FRAME_HEIGHT, 480.000000);
  webCam.set(CV_CAP_PROP_FPS, 0.000000);
  webCam.set(CV_CAP_PROP_FOURCC, 844715353.000000);
  webCam.set(CV_CAP_PROP_FRAME_COUNT, -1.000000);
  webCam.set(CV_CAP_PROP_FORMAT, -1.000000);
  webCam.set(CV_CAP_PROP_MODE, -1.000000);
  webCam.set(CV_CAP_PROP_BRIGHTNESS, 143.000000);
  webCam.set(CV_CAP_PROP_CONTRAST, 5.000000);
  webCam.set(CV_CAP_PROP_SATURATION, 83.000000);
  webCam.set(CV_CAP_PROP_HUE, 1781412864.000000);

  webCam.set(CV_CAP_PROP_GAIN, 1781412896.000000);
  webCam.set(CV_CAP_PROP_EXPOSURE, -6.000000);
  webCam.set(CV_CAP_PROP_CONVERT_RGB, -1.000000);
  webCam.set(CV_CAP_PROP_WHITE_BALANCE_BLUE_U, 4500.000000);
  webCam.set(CV_CAP_PROP_RECTIFICATION, -1.000000);
  webCam.set(CV_CAP_PROP_MONOCHROME, 1781413064.000000);
  webCam.set(CV_CAP_PROP_SHARPNESS, 25.000000);
  webCam.set(CV_CAP_PROP_AUTO_EXPOSURE, -1.000000);
  webCam.set(CV_CAP_PROP_GAMMA, 1781413160.000000);
  webCam.set(CV_CAP_PROP_TEMPERATURE, -1.000000);

  webCam.set(CV_CAP_PROP_TRIGGER, -1.000000);
  webCam.set(CV_CAP_PROP_TRIGGER_DELAY, -1.000000);
  webCam.set(CV_CAP_PROP_WHITE_BALANCE_RED_V, -1.000000);
  webCam.set(CV_CAP_PROP_ZOOM, 0.000000);
  webCam.set(CV_CAP_PROP_FOCUS, 1781413360.000000);
  webCam.set(CV_CAP_PROP_GUID, -1.000000);
  webCam.set(CV_CAP_PROP_ISO_SPEED, -1.000000);
  webCam.set(CV_CAP_PROP_MAX_DC1394, -1.000000);

  webCam.set(CV_CAP_PROP_BACKLIGHT, 0.000000);
  webCam.set(CV_CAP_PROP_PAN, 0.000000);
  webCam.set(CV_CAP_PROP_TILT, 0.000000);
  webCam.set(CV_CAP_PROP_ROLL, 1781413584.000000);
  webCam.set(CV_CAP_PROP_IRIS, 1781413616.000000);

  //********************************************************
  //*
  //*  Set up the control window
  //*
  //********************************************************
  //namedWindow("Control", CV_WINDOW_AUTOSIZE);

  int iLowH = 59;
  int iHighH = 75;

  int iLowS = 70;
  int iHighS = 255;

  int iLowV = 125;
  int iHighV = 177;

  //cvCreateTrackbar("LowV", "Control", &iLowV, 255);
  //cvCreateTrackbar("HighV", "Control", &iHighV, 255);

  //cvCreateTrackbar("LowH", "Control", &iLowH, 255);
  //cvCreateTrackbar("HighH", "Control", &iHighH, 255);

  //cvCreateTrackbar("LowS", "Control", &iLowS, 255);
  //cvCreateTrackbar("HighS", "Control", &iHighS, 255);


  //********************************************************
  //*
  //*  Set server socket
  //*
  //********************************************************
  bool bSuccess = webCam.read(gImgOriginal); // read a new frame from video

  fprintf(fp,"After First read\n");

  webCam.set(CV_CAP_PROP_EXPOSURE, -11.0);
//  webCam.set(CV_CAP_PROP_AUTO_EXPOSURE, 0.0);

  //********************************************************
  //*
  //*  Loop forever or until a socket error (the client socket closed)
  //*
  //********************************************************
  while (true)
  {
    //********************************************************
    //*
    //*  Read the image and turn it into a HSV image
    //*
    //********************************************************

    bool bSuccess = webCam.read(gImgOriginal); // read a new frame from video

    if (false == bSuccess)
    {
      cout << "Read from Cam failed" << endl;
    }

    gCamHeight = gImgOriginal.size().height / 2;
    gCamWidth = gImgOriginal.size().width / 2;

    if (!bSuccess) //if not success, break loop
    {
        cout << "Cannot read a frame from video stream" << endl;
    }

    cvtColor(gImgOriginal, gImgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

    //********************************************************
    //*
    //*  Threshold the image with the HSV ranges
    //*
    //********************************************************
    inRange(gImgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), gImgThresholded); //Threshold the image
    gImgThresholded.copyTo(gImgThresholdedCopy);

    //********************************************************
    //*
    //*  Find all of the blobs in the image, then find the biggest
    //*
    //********************************************************
    findContours(gImgThresholdedCopy, gContours, gHierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

    for (int i = 0; i < MAX_BLOBS; i++)
    {
      gLargestArea[i] = -1;
      gLargestCont[i] = -1;
    }

    for (int i = 0; i < (int)gContours.size(); i++)
    {
      double area = contourArea(gContours.at(i));

      for (int j = 0; j < MAX_BLOBS; j++)
      {
        if (area > gLargestArea[j])
        {
          for (int k = MAX_BLOBS - 2; k >= j; k--)
          {
            gLargestArea[k + 1] = gLargestArea[k];
            gLargestCont[k + 1] = gLargestCont[k];
          }
          gLargestArea[j] = area;
          gLargestCont[j] = i;
          break;
        }
      }
    }

    for (int j = 0; j < MAX_BLOBS; j++)
    {
      //********************************************************
      //*
      //*  There is a biggest blob (the image is not blank)
      //*
      //********************************************************
      if (-1 != gLargestCont[j])
      {
        Rect boundingRectangle = boundingRect(gContours.at(gLargestCont[j]));
        gX = (boundingRectangle.x - gCamWidth) + (boundingRectangle.width / 2);
        gY = (boundingRectangle.y - gCamHeight) + (boundingRectangle.height / 2);
        gWidth = boundingRectangle.width;
        gHeight = boundingRectangle.height;
        gArea = (int)gLargestArea[j];

        sprintf(stringBuffer, "/fromPi/blob%d/area", j);
        nt::SetEntryValue(stringBuffer, nt::Value::MakeDouble(gArea));
        sprintf(stringBuffer, "/fromPi/blob%d/x", j);
        nt::SetEntryValue(stringBuffer, nt::Value::MakeDouble(gX));
        sprintf(stringBuffer, "/fromPi/blob%d/y", j);
        nt::SetEntryValue(stringBuffer, nt::Value::MakeDouble(gY));
        sprintf(stringBuffer, "/fromPi/blob%d/height", j);
        nt::SetEntryValue(stringBuffer, nt::Value::MakeDouble(gHeight));
        sprintf(stringBuffer, "/fromPi/blob%d/width", j);
        nt::SetEntryValue(stringBuffer, nt::Value::MakeDouble(gWidth));
      }
      else
      {
        sprintf(stringBuffer, "/fromPi/blob%d/area", j);
        nt::SetEntryValue(stringBuffer, nt::Value::MakeDouble(-1));
        sprintf(stringBuffer, "/fromPi/blob%d/x", j);
        nt::SetEntryValue(stringBuffer, nt::Value::MakeDouble(-1));
        sprintf(stringBuffer, "/fromPi/blob%d/y", j);
        nt::SetEntryValue(stringBuffer, nt::Value::MakeDouble(-1));
        sprintf(stringBuffer, "/fromPi/blob%d/height", j);
        nt::SetEntryValue(stringBuffer, nt::Value::MakeDouble(-1));
        sprintf(stringBuffer, "/fromPi/blob%d/width", j);
        nt::SetEntryValue(stringBuffer, nt::Value::MakeDouble(-1));
      }
    }

    //********************************************************
    //*
    //*  Display the images
    //*
    //********************************************************
    //imshow("Thresholded Image", gImgThresholded); //show the thresholded image
    //imshow("Original", gImgOriginal); //show the original image

    try
    {
      if(gCount > 100)
      {
        imwrite("/home/pi/CHS2017/ColorLocator/Threshold.jpg", gImgThresholded);
        imwrite("/home/pi/CHS2017/ColorLocator/Original.jpg", gImgOriginal);
        gCount = 0;
      }
    }
    catch(Exception ex)
    {
    }
    gCount++;

    //********************************************************
    //*
    //*  Find out how fast the images are being processed
    //*
    //********************************************************
    gFramesPerSecond++;
    clock_t curTime = clock() / (CLOCKS_PER_SEC / 1000);

    if ((curTime - gStartTime) > 1000)
    {
        gStartTime = curTime;
        gCurFramesPerSeconds = gFramesPerSecond;
        gFramesPerSecond = 0;
    }

  }
  return 0;
}

//----------------------------------------------------------------------------
//  Purpose:
//         Take an int and cut it into four bytes and put it in a buffer
//
// Notes:
//     None.
//  
//----------------------------------------------------------------------------
void PutU32ToU8Array(int number, char data[], int location)
{
    data[location] = (char)((number >> 24) & 0xFF);
    data[location + 1] = (char)((number >> 16) & 0xFF);
    data[location + 2] = (char)((number >> 8) & 0xFF);
    data[location + 3] = (char)(number & 0xFF);
}

