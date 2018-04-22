/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   aruco_detection.cpp
 * Author: minto
 *
 * Created on 12 de abril de 2018, 15:42
 */



#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <vector>
#include <opencv2/core/affine.hpp>
#include <opencv2/calib3d.hpp>

using namespace std;
using namespace cv;

namespace {
    const char* about = "Basic marker detection";
    const char* keys =
            "{v        |       | Input from video file, if ommited, input comes from camera }"
            "{l        | 0.096 | Marker side lenght (in meters). Needed for correct scale in camera pose }"
            "{r        |       | show rejected candidates too }"
            "{f        | 30    | frames per second, 40 max}";
}

/*
 *
 */
static bool readCameraParameters(string filename, Mat &camMatrix, Mat &distCoeffs, int &vidWidth, int &vidHeight) {
    FileStorage fs(filename, FileStorage::READ);
    if (!fs.isOpened())
        return false;
    fs["camera_matrix"] >> camMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    fs["image_width"] >> vidWidth;
    fs["image_height"] >> vidHeight;
    return true;
}

struct marker {
    int id;
    Affine3d homMatrix;
};

/*
 *
 */
int main(int argc, char *argv[]) {
    CommandLineParser parser(argc, argv, keys);
    parser.about(about);

    string camParams("out_camera_data_640_100.xml");
    /* dictionary: DICT_4X4_50 = 0, DICT_4X4_100 = 1, DICT_4X4_250 = 2,
       DICT_4X4_1000 = 3, DICT_5X5_50 = 4, DICT_5X5_100 = 5, DICT_5X5_250 = 6, DICT_5X5_1000 = 7,
       DICT_6X6_50 = 8, DICT_6X6_100 = 9, DICT_6X6_250 = 10, DICT_6X6_1000 = 11, DICT_7X7_50 = 12,
       DICT_7X7_100 = 13, DICT_7X7_250 = 14, DICT_7X7_1000 = 15, DICT_ARUCO_ORIGINAL = 16
     */
    int dictionaryId = 0;
    int camId = 0;
    int vidFps = parser.get<float>("f");
    bool showRejected = parser.has("r");
    bool estimatePose = true;
    float markerLength = parser.get<float>("l");


    cout << "The length of the marker side is: " << markerLength << " m" << endl;

    // Create DetectorParameters and refine corners
    Ptr<aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();
    detectorParams->cornerRefinementMethod = aruco::CORNER_REFINE_SUBPIX; // do corner refinement in markers  

    String video;
    if (parser.has("v")) {
        video = parser.get<String>("v");
    }

    if (!parser.check()) {
        parser.printErrors();
        return 0;
    }

    Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));

    Mat camMatrix, distCoeffs;
    int vidWidth, vidHeight;
    // Get calibration params from file
    bool readOk = readCameraParameters(camParams, camMatrix, distCoeffs, vidWidth, vidHeight);
    if (!readOk) {
        cerr << "Invalid camera file" << endl;
        return 0;
    }

    VideoCapture inputVideo;
    // Set camera parameters
    inputVideo.set(CV_CAP_PROP_FRAME_WIDTH, vidWidth);
    inputVideo.set(CV_CAP_PROP_FRAME_HEIGHT, vidHeight);
    inputVideo.set(CV_CAP_PROP_FPS, vidFps);

    int waitTime;
    if (!video.empty()) {
        inputVideo.open(video);
        waitTime = 0;
    } else {
        inputVideo.open(camId);
        waitTime = 10;
    }

    double totalTime = 0;
    int totalIterations = 0;
    
    vector< marker > poses2(8);

    while (inputVideo.grab()) {
        Mat image, imageCopy;
        inputVideo.retrieve(image);

        double tick = (double) getTickCount(); // START time measuring

        vector< int > ids;
        vector< vector< Point2f > > corners, rejected;
        vector< Vec3d > rvecs, tvecs;

        // detect markers and estimate pose
        aruco::detectMarkers(image, dictionary, corners, ids, detectorParams, rejected);
        if (ids.size() > 0) {
            // Calculate poses
            aruco::estimatePoseSingleMarkers(corners, markerLength, camMatrix, distCoeffs, rvecs, tvecs);
        }
        // END time measurint
        double currentTime = ((double) getTickCount() - tick) / getTickFrequency();
        totalTime += currentTime;
        totalIterations++;
        if (totalIterations % vidFps == 0) {
            cout << "Detection Time = " << currentTime * 1000 << " ms " << "(Mean = " << 1000 * totalTime / double(totalIterations) << " ms)" << endl;
        }

        // Draw results
        image.copyTo(imageCopy);
        if (ids.size() > 0) {
            aruco::drawDetectedMarkers(imageCopy, corners, ids);
            
            cout << "----------------" << endl;
            
            for (unsigned int i = 0; i < ids.size(); i++) {
                aruco::drawAxis(imageCopy, camMatrix, distCoeffs, rvecs[i], tvecs[i], markerLength * 0.5f);
                cout << "ID = " << ids[i] << endl << " rvecs = " << rvecs[i] << endl << " tvecs = " << tvecs[i] << endl << endl;
                                
                Mat rotMat;    
                cv::Rodrigues(rvecs[i], rotMat);
                cout << "rotMat = " << endl << " " << rotMat << endl << endl;
                Affine3d pose1=Affine3d(rotMat, tvecs[i]);
                
                Affine3d distBtnMarkers = pose1 * poses2[ids[i]].homMatrix.inv();
                cout << "Distance between markers: Rmat" << endl << " " << distBtnMarkers.translation() << endl << endl;
                cout << "Distance between markers: tvec" << endl << " " << distBtnMarkers.rotation() << endl << endl;

                marker b;
                b.id = ids[i]; b.homMatrix = pose1;
                poses2[ids[i]] = b;
                
                cout << "Affine3d Pose 1 = " << endl << " " << pose1.matrix << endl << endl;
                cout << "Affine3d Pose 2 = " << endl << " " << poses2[ids[i]].homMatrix.matrix << endl << endl;

                cout << "----------------" << endl;
            }
        }

        if (showRejected && rejected.size() > 0)
            aruco::drawDetectedMarkers(imageCopy, rejected, noArray(), Scalar(100, 0, 255));

        namedWindow("RPi Camera", CV_WINDOW_AUTOSIZE);
        imshow("RPi Camera", imageCopy);
        char key = (char) waitKey(waitTime);
        if (key == 27) break;
    }

    return 0;
}









//void detectMarkers(InputArray _image, const Ptr<Dictionary> &_dictionary, OutputArrayOfArrays _corners,
//                   OutputArray _ids, const Ptr<DetectorParameters> &_params,
//                   OutputArrayOfArrays _rejectedImgPoints, InputArrayOfArrays camMatrix, InputArrayOfArrays distCoeff) {
//
//    CV_Assert(!_image.empty());
//
//    Mat grey;
//    _convertToGrey(_image.getMat(), grey);
//
//    /// STEP 1: Detect marker candidates
//    vector< vector< Point2f > > candidates;
//    vector< vector< Point > > contours;
//    vector< int > ids;
//
//    /// STEP 1.a Detect marker candidates :: using AprilTag
//    if(_params->cornerRefinementMethod == CORNER_REFINE_APRILTAG)
//        _apriltag(grey, _params, candidates, contours);
//
//    /// STEP 1.b Detect marker candidates :: traditional way
//    else
//        _detectCandidates(grey, candidates, contours, _params);
//
//    /// STEP 2: Check candidate codification (identify markers)
//    _identifyCandidates(grey, candidates, contours, _dictionary, candidates, ids, _params,
//                        _rejectedImgPoints);
//
//    /// STEP 3: Filter detected markers;
//    _filterDetectedMarkers(candidates, ids, contours);
//
//    // copy to output arrays
//    _copyVector2Output(candidates, _corners);
//    Mat(ids).copyTo(_ids);
//
//    /// STEP 4: Corner refinement :: use corner subpix
//    if( _params->cornerRefinementMethod == CORNER_REFINE_SUBPIX ) {
//        CV_Assert(_params->cornerRefinementWinSize > 0 && _params->cornerRefinementMaxIterations > 0 &&
//                  _params->cornerRefinementMinAccuracy > 0);
//
//        //// do corner refinement for each of the detected markers
//        // for (unsigned int i = 0; i < _corners.cols(); i++) {
//        //    cornerSubPix(grey, _corners.getMat(i),
//        //                 Size(params.cornerRefinementWinSize, params.cornerRefinementWinSize),
//        //                 Size(-1, -1), TermCriteria(TermCriteria::MAX_ITER | TermCriteria::EPS,
//        //                                            params.cornerRefinementMaxIterations,
//        //                                            params.cornerRefinementMinAccuracy));
//        //}
//
//        // this is the parallel call for the previous commented loop (result is equivalent)
//        parallel_for_(Range(0, _corners.cols()),
//                      MarkerSubpixelParallel(&grey, _corners, _params));
//    }
//
//    /// STEP 4, Optional : Corner refinement :: use contour container
//    if( _params->cornerRefinementMethod == CORNER_REFINE_CONTOUR){
//
//        if(! _ids.empty()){
//
//            // do corner refinement using the contours for each detected markers
//            parallel_for_(Range(0, _corners.cols()), MarkerContourParallel(contours, candidates, camMatrix.getMat(), distCoeff.getMat()));
//
//            // copy the corners to the output array
//            _copyVector2Output(candidates, _corners);
//        }
//    }
//}