/* INCLUDES FOR THIS PROJECT */
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <limits>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include "dataStructures.h"
#include "matching2D.hpp"

using namespace std;

/* MAIN PROGRAM */
int main(int argc, const char *argv[])
{

	vector<string> detectors{"SHITOMASI","HARRIS","FAST", "BRISK", "ORB", "AKAZE", "SIFT"};
	vector<string> descriptors{"BRISK","BRIEF", "ORB", "FREAK", "AKAZE", "SIFT"};
	string currDescType = "DES_BINARY";

	for(int detectIdx = 0; detectIdx<detectors.size();detectIdx++)
	{
		cout<<"###################################"<<endl;
		string currentDetector = detectors[detectIdx];
		for(int descIdx = 0; descIdx<descriptors.size();descIdx++)
		{
			cout<<"#################"<<endl;
			string currentDesc = descriptors[descIdx];

			if(currentDesc == "SIFT")
			{
				currDescType = "DES_HOG";
			}
			else
			{
				currDescType = "DES_BINARY";
			}

			if((currentDetector == "AKAZE" && currentDesc != "AKAZE") ||
			   (currentDetector != "AKAZE" && currentDesc == "AKAZE"))
			{
				continue;
			}

			if(currentDetector == "SIFT" && currentDesc == "ORB")
			{
				continue;
			}


		    /* INIT VARIABLES AND DATA STRUCTURES */

		    // data location
		    string dataPath = "../";

		    // camera
		    string imgBasePath = dataPath + "images/";
		    string imgPrefix = "KITTI/2011_09_26/image_00/data/000000"; // left camera, color
		    string imgFileType = ".png";
		    int imgStartIndex = 0; // first file index to load (assumes Lidar and camera names have identical naming convention)
		    int imgEndIndex = 9;   // last file index to load
		    int imgFillWidth = 4;  // no. of digits which make up the file index (e.g. img-0001.png)

		    // misc
		    int dataBufferSize = 2;       // no. of images which are held in memory (ring buffer) at the same time
		    vector<DataFrame> dataBuffer; // list of data frames which are held in memory at the same time
		    int currBufferIndex = 0;
		    int prevBufferIndex = 0;
		    bool bVis = false;            // visualize results

		    /* MAIN LOOP OVER ALL IMAGES */

		    int numMatchedKeypoints = 0;
		    float processingTime      = 0;

		    for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; imgIndex++)
		    {
		        /* LOAD IMAGE INTO BUFFER */

		        // assemble filenames for current index
		        ostringstream imgNumber;
		        imgNumber << setfill('0') << setw(imgFillWidth) << imgStartIndex + imgIndex;
		        string imgFullFilename = imgBasePath + imgPrefix + imgNumber.str() + imgFileType;

		        // load image from file and convert to grayscale
		        cv::Mat img, imgGray;
		        img = cv::imread(imgFullFilename);
		        cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

		        //// STUDENT ASSIGNMENT
		        //// TASK MP.1 -> replace the following code with ring buffer of size dataBufferSize
		        //cout<<"################# IMG "<<imgIndex<<"#########################"<<endl;
		        // push image into data frame buffer
		        DataFrame frame;
		        frame.cameraImg = imgGray;
		        if( dataBuffer.size() < dataBufferSize )
		        {
		            dataBuffer.push_back(frame);
		        }
		        else
		        {
		        	dataBuffer[currBufferIndex].cameraImg = frame.cameraImg;
		        	dataBuffer[currBufferIndex].descriptors = frame.descriptors;
		        	dataBuffer[currBufferIndex].keypoints = frame.keypoints;
		        	dataBuffer[currBufferIndex].kptMatches = frame.kptMatches;
		        }

		        //// EOF STUDENT ASSIGNMENT
		        //cout << "#1 : LOAD IMAGE INTO BUFFER done" << endl;

		        /* DETECT IMAGE KEYPOINTS */

		        // extract 2D keypoints from current image
		        vector<cv::KeyPoint> keypoints; // create empty feature list for current image
		        string detectorType = currentDetector;

		        //// STUDENT ASSIGNMENT
		        //// TASK MP.2 -> add the following keypoint detectors in file matching2D.cpp and enable string-based selection based on detectorType
		        //// -> HARRIS, FAST, BRISK, ORB, AKAZE, SIFT
		        double t = (double)cv::getTickCount();

		        if (detectorType.compare("SHITOMASI") == 0)
		        {
		            detKeypointsShiTomasi(keypoints, imgGray, false);
		        }
		        else if(detectorType.compare("HARRIS") == 0)
		        {
		        	detKeypointsHarris(keypoints, imgGray, false);
		        }
		        else
		        {
		        	detKeypointsModern(keypoints,imgGray, detectorType, false);

		        }
		        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();

		        processingTime += (1000 * t / 1.0);

		        //// EOF STUDENT ASSIGNMENT

		        //// STUDENT ASSIGNMENT
		        //// TASK MP.3 -> only keep keypoints on the preceding vehicle

		        // only keep keypoints on the preceding vehicle
		        bool bFocusOnVehicle = true;
		        cv::Rect vehicleRect(535, 180, 180, 150);
		        if (bFocusOnVehicle)
		        {
		            vector<cv::KeyPoint> keyPointsVeh;

		            for(auto & keypoint : keypoints)
		            {
		            	if(vehicleRect.contains(keypoint.pt))
		            	{
		            		keyPointsVeh.push_back(keypoint);
		            	}
		            }

		            keypoints = keyPointsVeh;
		        }

		        //cout<<"Num of Keypoints detected on front vehicle are "<<keypoints.size()<<endl;
		        //// EOF STUDENT ASSIGNMENT

		        // optional : limit number of keypoints (helpful for debugging and learning)
		        bool bLimitKpts = false;
		        if (bLimitKpts)
		        {
		            int maxKeypoints = 50;

		            if (detectorType.compare("SHITOMASI") == 0)
		            { // there is no response info, so keep the first 50 as they are sorted in descending quality order
		                keypoints.erase(keypoints.begin() + maxKeypoints, keypoints.end());
		            }
		            cv::KeyPointsFilter::retainBest(keypoints, maxKeypoints);
		            cout << " NOTE: Keypoints have been limited!" << endl;
		        }

		        // push keypoints and descriptor for current frame to end of data buffer
		        dataBuffer[currBufferIndex].keypoints = keypoints;
		        //cout << "#2 : DETECT KEYPOINTS done" << endl;

		        /* EXTRACT KEYPOINT DESCRIPTORS */

		        //// STUDENT ASSIGNMENT
		        //// TASK MP.4 -> add the following descriptors in file matching2D.cpp and enable string-based selection based on descriptorType
		        //// -> BRIEF, ORB, FREAK, AKAZE, SIFT

		        cv::Mat descriptors;
		        string descriptorType = currentDesc; // BRISK, BRIEF, ORB, FREAK, AKAZE, SIFT

		        t = (double)cv::getTickCount();
		        descKeypoints(dataBuffer[currBufferIndex].keypoints, dataBuffer[currBufferIndex].cameraImg, descriptors, descriptorType);

		        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();

		        processingTime += (1000 * t / 1.0);
		        //// EOF STUDENT ASSIGNMENT

		        // push descriptors for current frame to end of data buffer
		        dataBuffer[currBufferIndex].descriptors = descriptors;

		        //cout << "#3 : EXTRACT DESCRIPTORS done" << endl;

		        if (dataBuffer.size() > 1) // wait until at least two images have been processed
		        {

		            /* MATCH KEYPOINT DESCRIPTORS */

		            vector<cv::DMatch> matches;
		            string matcherType = "MAT_BF";        // MAT_BF, MAT_FLANN
		            string descriptorType = currDescType; // DES_BINARY, DES_HOG
		            string selectorType = "SEL_KNN";       // SEL_NN, SEL_KNN

		            //// STUDENT ASSIGNMENT
		            //// TASK MP.5 -> add FLANN matching in file matching2D.cpp
		            //// TASK MP.6 -> add KNN match selection and perform descriptor distance ratio filtering with t=0.8 in file matching2D.cpp

		            matchDescriptors(dataBuffer[prevBufferIndex].keypoints, dataBuffer[currBufferIndex].keypoints,
		            				 dataBuffer[prevBufferIndex].descriptors, dataBuffer[currBufferIndex].descriptors,
		                             matches, descriptorType, matcherType, selectorType);

		            numMatchedKeypoints += matches.size();
		            //// EOF STUDENT ASSIGNMENT

		            // store matches in current data frame
		            (dataBuffer.end() - 1)->kptMatches = matches;

		            //cout << "#4 : MATCH KEYPOINT DESCRIPTORS done" << endl;

		            // visualize matches between current and previous image
		            bVis = false;
		            if (bVis)
		            {
		                cv::Mat matchImg = (dataBuffer[currBufferIndex].cameraImg).clone();
		                cv::drawMatches(dataBuffer[prevBufferIndex].cameraImg, dataBuffer[prevBufferIndex].keypoints,
		                				dataBuffer[currBufferIndex].cameraImg, dataBuffer[currBufferIndex].keypoints,
		                                matches, matchImg,
		                                cv::Scalar::all(-1), cv::Scalar::all(-1),
		                                vector<char>(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

		                string windowName = "Matching keypoints between two camera images";
		                cv::namedWindow(windowName, 7);
		                cv::imshow(windowName, matchImg);
		                cout << "Press key to continue to next image" << endl;
		                //cv::waitKey(0); // wait for key to be pressed
		            }
		            bVis = false;
		        }

		    	prevBufferIndex = currBufferIndex;
		    	currBufferIndex++;

		    	if(currBufferIndex == dataBufferSize)
		    	{
		    		currBufferIndex = 0;
		    	}
		    } // eof loop over all images

		    cout<<currentDetector<<" - "<<currentDesc<<endl;
		    cout<<"Total Matched Keypoints = "<<numMatchedKeypoints/10<<endl;
		    cout<<"Total Processing Time   = "<<processingTime/10<<endl;

		}
	}



    return 0;
}
