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
#include <fstream>
#include <vector>
#include <opencv2/core/affine.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <math.h>
#include <unistd.h>

#include "rt_communication.h"

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

static void distanceBtwMarkers(ofstream& file, vector< int >& ids, vector< Vec3d >& rvecs, vector< Vec3d >& tvecs, vector< Affine3d >& poses2) {
    if (ids.size() > 0) {
        file.open("cmdout.txt", fstream::in | fstream::out | fstream::app);
        for (int i = 0; i < ids.size(); i++) {
            cout << "ID = " << ids[i] << endl << " rvecs = " << rvecs[i] << endl << " tvecs = " << tvecs[i] << endl << endl;

            Affine3d pose1 = Affine3d(rvecs[i], tvecs[i]);

            Affine3d distBtnMarkers = pose1 * poses2[ids[i]].inv();
            //            cout << "Distance between markers: Rmat" << endl << " " << distBtnMarkers.translation() << endl << endl;
            //            cout << "Distance between markers: tvec" << endl << " " << distBtnMarkers.rotation() << endl << endl;

            poses2[ids[i]] = pose1;

            //            cout << "Affine3d Pose 1 = " << endl << " " << pose1.matrix << endl << endl;
            //            cout << "Affine3d Pose 2 = " << endl << " " << poses2[ids[i]].matrix << endl << endl;
//            fprintf(file, "Reference Base:");
            
            file << "----------------" << endl;
            file << "ID:" << ids[i] << endl;
            file << "Reference Base:" << endl;
            char ak[100];
//            sprintf(ak, "movej(p[%f, %f, %f, %f, %f, %f], a=1.0, v=1.0)\n", tvecs[0].val[0], tvecs[0].val[1], tvecs[0].val[2], fmod(rvecs[0].val[0] + M_PI, M_PI), rvecs[0].val[1], rvecs[0].val[2]);
            sprintf(ak, "movej(p[%f, %f, %f, 0.0, 1.57079, 0.0], a=1.0, v=1.0)\n", tvecs[0].val[0], tvecs[0].val[1], tvecs[0].val[2], rvecs[0].val[0], rvecs[0].val[1], rvecs[0].val[2]);

            file << ak;
//            file << "Distance btw markers:" << endl;
//            char ka[100];
//            sprintf(ka, "movej(p[%f, %f, %f, %f, %f, %f], a=1.0, v=1.0)\n", distBtnMarkers.translation()[0], distBtnMarkers.translation()[1], distBtnMarkers.translation()[2], distBtnMarkers.rvec()[0], distBtnMarkers.rvec()[1], distBtnMarkers.rvec()[2]);
//            file << ka;
            file << "----------------" << endl;

            file.close();
        }
    }
}

static void markerProcesor(RtCommunication& com, Affine3d& offset, int id, bool detectedBfr) { // Marcador antiguo marcador nuevo
    if (!detectedBfr) {
        switch (id) {
            case 0:
                com.movej(0, -M_PI / 2, -M_PI / 2, 0, M_PI / 2, 0);
                break;
            case 1:
                com.movej(-M_PI / 2, -M_PI / 2, -M_PI / 2, 0, M_PI / 2, 0);
                break;
            case 2:
                com.movej(-M_PI, -M_PI / 2, -M_PI / 2, 0, M_PI / 2, 0);
                break;
            case 3:
                com.movej(M_PI / 2, -M_PI / 2, -M_PI / 2, 0, M_PI / 2, 0);
                break;
            case 4:
                com.movej(0, -M_PI / 2, -M_PI / 2, -M_PI / 2, M_PI / 2, 0);
                break;
            case 5:
                com.movej(0, -M_PI / 2, -M_PI / 2, -M_PI / 2, -M_PI / 2, 0);
                break;
            case 6:
                com.set_digital_out(4, true);
                break;
            case 7:
                com.set_digital_out(4, false);
                break;
        }
        usleep(10000000);

    } else {
        if (id != 6 || id != 7)
            com.movejp(offset);
    }
}

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
    float markerLength = parser.get<float>("l");
    
    ofstream file;

    

    cout << "The length of the marker side is: " << markerLength << " m" << endl;
    // COMMUNICATION !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!





    //    RtCommunication com("192.168.238.142");


    //    RtDataHandler dataHandler;
    //    RtCommunication com("158.42.206.10");
    //    com.start();
    //    usleep(2000000);
    //    print_debug("WWWWW");
    //    usleep(2000000);
    //    com.addCommandToQueue("set_digital_out(4,True)");
    //    usleep(2000000);
    //    print_debug("MMMMM");
    //    com.addCommandToQueue("set_digital_out(4,False)");
    //    usleep(2000000);    

















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

    vector< Affine3d > poses2(8);
    vector< bool > detectedBefore(8, false);
    vector< int > idsPast;

    while (inputVideo.grab()) {
        //        cout << "Version = " << dataHandler.getVersion() << endl;    

        Mat image, imageCopy;
        inputVideo.retrieve(image);

        // START time measuring
        double tick = (double) getTickCount();

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

        // Draw midle rectangle and marker vectors
        image.copyTo(imageCopy);
        int x = 270;
        int y = 190;
        int width = 100;
        int height = width;
        Rect rect(x, y, width, height);
        Point pt1(x, y), pt2(x + width, y + height);
        ; // tvecs ~ 0.49
        rectangle(imageCopy, pt1, pt2, Scalar(255, 0, 0), 2);

        if (ids.size() > 0) {
            aruco::drawDetectedMarkers(imageCopy, corners, ids);

            cout << "----------------" << endl;
            // get the distance btw markers
            distanceBtwMarkers(file, ids, rvecs, tvecs, poses2);

            for (unsigned int i = 0; i < ids.size(); i++) {
                aruco::drawAxis(imageCopy, camMatrix, distCoeffs, rvecs[i], tvecs[i], markerLength * 0.5f);
            }
            // Change idsPast
            cout << "idsCurrent = " << ids.size() << endl;
            cout << "idsPast = " << idsPast.size() << endl;

            if (idsPast.size() != ids.size())
                cout << "DASDOANSDKNFWLJEBFWILANKWDFBAOIDFBWEKFBAWDBASDPFI" << endl;
            idsPast = ids;


        }

        // Show rejected markers if there are
        if (showRejected && rejected.size() > 0)
            aruco::drawDetectedMarkers(imageCopy, rejected, noArray(), Scalar(100, 0, 255));

        namedWindow("RPi Camera", CV_WINDOW_AUTOSIZE);

        cout << "----------------" << endl;
        imshow("RPi Camera", imageCopy);
        char key = (char) waitKey(waitTime);
        if (key == 27) {
            break;
        }
    }

    return 0;
}


