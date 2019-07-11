/* INCLUDES FOR THIS PROJECT */
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <numeric>
#include <cmath>
#include <limits>
#include <algorithm>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include "dataStructures.h"
#include "matching2D.hpp"
#include "CircularBuffer.hpp"

using namespace std;

/* MAIN PROGRAM */
int main(int, const char*[])
{

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
    const size_t dataBufferSize = 2;       // no. of images which are held in memory (ring buffer) at the same time
    CircularBuffer<DataFrame, dataBufferSize> dataBuffer; // list of data frames which are held in memory at the same time
    bool bVis = false;            // visualize results


    // all possible combinations of detectors, descriptors, matchers, and selectors
    for (auto detector_ind : detectors)
    {
      for (size_t descriptor_ind = 0; descriptor_ind < num_of_detectors; ++descriptor_ind)
      {
        for (auto matcher : matchers)
        {
          for (auto selector : selectors)
          {
            std::ostringstream oss;
            oss << ToString(detector_ind)     << '_'
                << ToString(descriptors[descriptor_ind]) << '_'
                << ToString(CompatibleDescriptorTypes(descriptors[descriptor_ind])[0]) << '_'
                << ToString(matcher) << '_'
                << ToString(selector);

            std::string unique_prefix = oss.str();


            std::cout << "\n\n\n\n" << unique_prefix<< std::endl;

            /* MAIN LOOP OVER ALL IMAGES */
            for (int imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; imgIndex++)
            {
              /* LOAD IMAGE INTO BUFFER */
              std::cout << "Image number is " << imgIndex<< std::endl;

              // assemble filenames for current index
              ostringstream imgNumber;
              imgNumber << setfill('0') << setw(imgFillWidth) << imgStartIndex + imgIndex;
              string imgFullFilename = imgBasePath + imgPrefix + imgNumber.str() + imgFileType;

              // load image from file and convert to grayscale
              cv::Mat img, imgGray;
              img = cv::imread(imgFullFilename);
              cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

              // push image into data frame buffer
              DataFrame frame;
              frame.cameraImg = imgGray;
              dataBuffer.push_back(frame);

              cout << "#1 : LOAD IMAGE INTO BUFFER done" << endl;

              /* DETECT IMAGE KEYPOINTS */

              // extract 2D keypoints from current image
              vector<cv::KeyPoint> keypoints; // create empty feature list for current image
              Detector detector = detector_ind; // SHITOMASI, HARRIS, FAST, BRISK, ORB, AKAZE, SIFT

              switch (detector)
              {
                case detector_SHITOMASI:
                {
                  detKeypointsShiTomasi(keypoints, imgGray, bVis);
                  break;
                }
                case detector_HARRIS:
                {
                  detKeypointsHarris(keypoints, imgGray, bVis);
                  break;
                }
                default:
                {
                  detKeypointsModern(keypoints, imgGray, detector, bVis);
                }

              }

              // only keep keypoints on the preceding vehicle
              bool bFocusOnVehicle = true;
              cv::Rect vehicleRect(535, 180, 180, 150);
              if (bFocusOnVehicle)
              {
                keypoints.erase(std::remove_if(keypoints.begin(), keypoints.end(),
                                  [&vehicleRect](const auto& kp){return not vehicleRect.contains(kp.pt); }),
                                keypoints.end());
              }

              auto mean =
                  std::accumulate(keypoints.begin(), keypoints.end(), 0.0,
                                  [](const auto sum, const auto& kp2) { return sum + kp2.size; }) / keypoints.size();
              auto [min_elem, max_elem] =
              std::minmax_element(keypoints.begin(), keypoints.end(),
                                  [](const auto& kp1, const auto& kp2) { return kp1.size < kp2.size; });

              std::fstream ofs;
              ofs.open(unique_prefix + "_keypoints_number.txt", std::ios::app);
              ofs << keypoints.size() << ' '
                  << min_elem->size   << ' '
                  << max_elem->size   << ' '
                  << mean             << ' '
                  << '\n';
              ofs.close();

              // optional : limit number of keypoints (helpful for debugging and learning)
              bool bLimitKpts = false;
              if (bLimitKpts)
              {
                int maxKeypoints = 50;

                if (detector == detector_SHITOMASI)
                { // there is no response info, so keep the first 50 as they are sorted in descending quality order
                  keypoints.erase(keypoints.begin() + maxKeypoints, keypoints.end());
                }
                cv::KeyPointsFilter::retainBest(keypoints, maxKeypoints);
                cout << " NOTE: Keypoints have been limited!" << endl;
              }

              // push keypoints and descriptor for current frame to end of data buffer
              (dataBuffer.end() - 1)->keypoints = keypoints;
              cout << "#2 : DETECT KEYPOINTS done" << endl;

              /* EXTRACT KEYPOINT DESCRIPTORS */
              cv::Mat descriptors_mat;
              Descriptor descriptor = descriptors[descriptor_ind]; // BRISK, BRIEF, ORB, FREAK, AKAZE, SIFT
              descKeypoints((dataBuffer.end()-1)->keypoints, (dataBuffer.end()-1)->cameraImg,
                            descriptors_mat, descriptor);

              // push descriptors for current frame to end of data buffer
              (dataBuffer.end() - 1)->descriptors = descriptors_mat;

              cout << "#3 : EXTRACT DESCRIPTORS done" << endl;

              if (dataBuffer.size() > 1) // wait until at least two images have been processed
              {

                /* MATCH KEYPOINT DESCRIPTORS */

                vector<cv::DMatch> matches;
                // Possible matchers are MAT_BF, MAT_FLANN
                DescriptorType descriptorType = CompatibleDescriptorTypes(descriptor)[0]; // DES_BINARY, DES_HOG
                // Possible selectors are SEL_NN, SEL_KNN

                matchDescriptors((dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints,
                                 (dataBuffer.end() - 2)->descriptors, (dataBuffer.end() - 1)->descriptors,
                                 matches, descriptorType, matcher, selector);


                // store matches in current data frame
                (dataBuffer.end() - 1)->kptMatches = matches;

                std::fstream ofs_matches;
                ofs_matches.open("matches.txt", std::ios::app);
                ofs_matches << ToString(detector)       << ' '
                            << ToString(descriptor)     << ' '
                            << ToString(descriptorType) << ' '
                            << ToString(matcher)        << ' '
                            << ToString(selector)       << ' '
                            << matches.size()           << '\n';
                ofs_matches.close();

                cout << "#4 : MATCH KEYPOINT DESCRIPTORS done" << endl;

                // visualize matches between current and previous image
                bVis = false;
                if (bVis)
                {
                  cv::Mat matchImg = ((dataBuffer.end() - 1)->cameraImg).clone();
                  cv::drawMatches((dataBuffer.end() - 2)->cameraImg, (dataBuffer.end() - 2)->keypoints,
                                  (dataBuffer.end() - 1)->cameraImg, (dataBuffer.end() - 1)->keypoints,
                                  matches, matchImg,
                                  cv::Scalar::all(-1), cv::Scalar::all(-1),
                                  vector<char>(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

                  string windowName = "Matching keypoints between two camera images";
                  cv::namedWindow(windowName, 7);
                  cv::imshow(windowName, matchImg);
                  cout << "Press key to continue to next image" << endl;
                  cv::waitKey(0); // wait for key to be pressed
                }
                bVis = false;
              }

            } // eof loop over all images




          }
        }
      }
    }


    return 0;
}
