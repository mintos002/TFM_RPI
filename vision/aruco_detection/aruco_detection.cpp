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

static void setPointOfView(int id, Vec3d& rvec, Vec3d& tvec, Affine3d poseRef, Vec3d& robotToMarkerTvec/*, Vec3d& robotToMarkerRvec*/) {
    ofstream file;
    file.open("robotToMarkerTvec.txt", fstream::in | fstream::out | fstream::app);
    file << "______________________" << endl;

    double x;
    double y;
    double z;
    // x, y, z reference values in real robot for the different positions
    switch (id) {
        case 0:
            x = 0.38883;
            y = -0.11571;
            z = 0.48144;
            break;
        case 1:
            x = -0.11571;
            y = -0.38883;
            z = 0.48144;
            break;
        case 2:
            x = -0.38883;
            y = 0.11571;
            z = 0.48144;
            break;
        case 3:
            x = 0.11571;
            y = 0.38883;
            z = 0.48144;
            break;
        case 4:
            x = 0.29764;
            y = -0.11476;
            z = 0.21986;
            break;
        case 5:
            x = 0.29764;
            y = -0.11476;
            z = 0.57258;
            break;
        default:
            break;
    }

    Affine3d poseActual = Affine3d(rvec, tvec);
    Affine3d distFromRef = poseActual * poseRef.inv();
//    Vec3d ntvec = distFromRef.translation();
    Vec3d ntvec = tvec-poseRef.translation();
    //    Vec3d robotToMarker;
    
    // change perspective, where markerToRobot = [z,x,-y]
    robotToMarkerTvec(0) = -ntvec(2) + x;
    robotToMarkerTvec(1) = ntvec(0) + y;
    robotToMarkerTvec(2) = -ntvec(1) + z;
    
//    robotToMarkerRvec(0) = rvec(0);
//    robotToMarkerRvec(1) = rvec(1);
//    robotToMarkerRvec(2) = rvec(2);
    
    
    cout << "OOOOOOOOOOOOOOOOOOOOOOOOOOOOO" << endl;
    cout << "OOOOOOOOOOOOOOOOOOOOOOOOOOOOO" << endl;
    cout << robotToMarkerTvec << endl;
    
    file << "tvec: " << endl;
    file << "[";
    file << tvec(0) << ", ";
    file << tvec(1) << ", ";
    file << tvec(2) << "]" << endl;
    
    file << "refvec: " << endl;
    file << "[";
    file << poseRef.translation()(0) << ", ";
    file << poseRef.translation()(1) << ", ";
    file << poseRef.translation()(2) << "]" << endl;

    file << "ntvec: " << endl;
    file << "[";
    file << ntvec(0) << ", ";
    file << ntvec(1) << ", ";
    file << ntvec(2) << "]" << endl;
    
    file << "robotToMarkerTvec: " << endl;
    file << "[";
    file << robotToMarkerTvec(0) << ", ";
    file << robotToMarkerTvec(1) << ", ";
    file << robotToMarkerTvec(2);
    file << "]" << endl;
    cout << "OOOOOOOOOOOOOOOOOOOOOOOOOOOOO" << endl;
    cout << "OOOOOOOOOOOOOOOOOOOOOOOOOOOOO" << endl;


    cout << "++++++++++++++++++" << endl;
    cout << "rvec: " << endl << rvec << endl;
    cout << "tvec: " << endl << tvec << endl;
    cout << "poseActual: " << endl << poseActual.matrix << endl;
    cout << "poseRef: " << endl << poseRef.matrix << endl;
    cout << "distFromRef: " << endl << distFromRef.matrix << endl;
    cout << "TRANSLATION REF POINT" << endl << ntvec << endl;
    cout << "++++++++++++++++++" << endl;
    
    file << "______________________" << endl;
    file.close();
}

static void distanceBtwMarkers(ofstream& file, vector< int >& ids, vector< Vec3d >& rvecs, vector< Vec3d >& tvecs, vector< Affine3d >& poses2) {
    if (ids.size() > 0) {
//        file.open("cmdout.txt", fstream::in | fstream::out | fstream::app);
        for (int i = 0; i < ids.size(); i++) {
            // Define xyz rxryrz and some security checks
            double x = tvecs[i].val[0];
            double y = tvecs[i].val[1];
            double z = tvecs[i].val[2];
            double rx = rvecs[i].val[0];
            double ry = rvecs[i].val[1];
            double rz = rvecs[i].val[2];

            // Change perspective respect camera
            //Security limit cap
            //            if(abs(x) > 0.5)
            //                x = 0.5;
            //            if(abs(y) > 0.5)
            //                y = 0.5;
            //            if(z > 0.5)
            //                z = 0.5;
            //            if(z < -0.10)
            //                z = -0.10;

            //            cout << "ID real =" << ids[i] << endl << " rvecs = " << rvecs[i] << endl << " tvecs = " << tvecs[i] << endl << endl;
            //            cout << "ID secured =" << ids[i] << endl << " rvecs = [" << rx << ", " << ry << ", " << rz << "]" << endl << " tvecs = [" << x << ", " << y << ", " << z << "]" << endl << endl;

            Affine3d pose1 = Affine3d(rvecs[i], tvecs[i]);

            Affine3d distBtnMarkers = pose1 * poses2[ids[i]].inv();
            //            cout << "Distance between markers: Rmat" << endl << " " << distBtnMarkers.translation() << endl << endl;
            //            cout << "Distance between markers: tvec" << endl << " " << distBtnMarkers.rotation() << endl << endl;



//            cout << "Affine3d Pose 1 = " << endl << " " << pose1.matrix << endl << endl;
//            cout << "Affine3d Pose 2 = " << endl << " " << poses2[ids[i]].matrix << endl << endl;

            //            file << "----------------" << endl;
            //            file << "ID:" << ids[i] << endl;
            //            file << "Reference Base:" << endl;
            //            char ak[100];
            ////            sprintf(ak, "movej(p[%f, %f, %f, %f, %f, %f], a=1.0, v=1.0)\n", tvecs[0].val[0], tvecs[0].val[1], tvecs[0].val[2], fmod(rvecs[0].val[0] + M_PI, M_PI), rvecs[0].val[1], rvecs[0].val[2]);
            //            sprintf(ak, "movej(p[%f, %f, %f, 2.399, -2.422, 2.424], a=1.0, v=1.0)\n", x, y, z);
            //            file << ak;

            //            file << "Distance btw markers:" << endl;
            //            char ka[100];
            //            sprintf(ka, "movej(p[%f, %f, %f, %f, %f, %f], a=1.0, v=1.0)\n", distBtnMarkers.translation()[0], distBtnMarkers.translation()[1], distBtnMarkers.translation()[2], distBtnMarkers.rvec()[0], distBtnMarkers.rvec()[1], distBtnMarkers.rvec()[2]);
            //            file << ka;
            file << "----------------" << endl;

            file.close();
        }
    }
}

static void markerProcesor(RtCommunication& com, vector< int >& ids, vector< Vec3d >& rvecs, vector< Vec3d >& tvecs, int& remainId, Affine3d& poseRef, vector< Affine3d >& poses2) { // Marcador antiguo marcador nuevo
    // check if there are any marker detected
    if (ids.size() < 1) return;
    // initatie detection tag and posible next position
    bool detected = false;
    int nextpp = -1;
    int w = -1;
    // check for marker ids, if id = 6 or 7 open/close the tool, if is = to last Id, move to that point.
    for (int i = 0; i < ids.size(); i++) {
        if (ids[i] == 6 || ids[i] == 7) {
            switch (ids[i]) {
                case 6:
                    com.set_digital_out(4, true);
                case 7:
                    com.set_digital_out(4, false);
            }
        } else {
            if (ids[i] == remainId) {
                // if detected, send offset movement and exit the function
                detected = true;
                Vec3d poseFinal;
                Vec3d poseDif = tvecs[i] - poses2[ids[i]].translation();
                ofstream file;
                file.open("poseDif.txt", fstream::in | fstream::out | fstream::app);
                file << "Posedif: " << endl << "[" << poseDif(0) << ", " << poseDif(2) << ", " << poseDif(2) << "]" << endl;
                file.close();
                double lim = 0.02;
                // setting a limit to not overload the server by positions
                if (abs(poseDif(0)) > lim || abs(poseDif(1)) > lim || abs(poseDif(2)) > lim) {
                    setPointOfView(ids[i], rvecs[i], tvecs[i], poseRef, poseFinal);
                    com.movejp(poseFinal(0), poseFinal(1), poseFinal(2));
                }
                return;
            } else {
                w = i;
                nextpp = ids[i];
            }
        }
    }
    // if remainId is not in ids, go to nextpp reference position
    if (!detected) {
        remainId = nextpp;
        switch (remainId) {
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
            default:
                return;
        }

        // wait 5s, the robot will take 4s to reach the reference position
        print_info("The robot is going to the initial position...");

        // save reference position
        poseRef = Affine3d(rvecs[w], tvecs[w]);
        // wait for the robot to go to refPoint
        usleep(5000000);

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





    //        RtCommunication com("192.168.238.142");


    //        RtDataHandler dataHandler;
    RtCommunication com("158.42.206.10");
    com.start();
    //        usleep(5000000);
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
    Affine3d poseRef;
    int remainId = -1;

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

        // Draw midle rectangle and lines in image
        image.copyTo(imageCopy);
        int x = 270;
        int y = 190;
        int width = 100;
        int height = width;
        //        Rect rect(x, y, width, height);
        Point pt1(x, y), pt2(x + width, y + height);
        // tvecs ~ 0.49
        rectangle(imageCopy, pt1, pt2, Scalar(255, 0, 0), 2);


        line(imageCopy, Point(0, 360), Point(640, 360), Scalar(255, 0, 255), 2, 8);
        line(imageCopy, Point(320, 0), Point(320, 480), Scalar(255, 0, 255), 2, 8);

        // Draw marker vectors
        if (ids.size() > 0) {
            aruco::drawDetectedMarkers(imageCopy, corners, ids);
            cout << "%%%%%%%%%%%%%%%%%%%%%%%%%%" << endl << "CORNERS: " << endl << corners[0] << endl << "%%%%%%%%%%%%%%%%%%%%%%%%%%" << endl;

            cout << "----------------" << endl;
            // get the distance btw markers
            distanceBtwMarkers(file, ids, rvecs, tvecs, poses2);
            markerProcesor(com, ids, rvecs, tvecs, remainId, poseRef, poses2);


            for (unsigned int i = 0; i < ids.size(); i++) {
                aruco::drawAxis(imageCopy, camMatrix, distCoeffs, rvecs[i], tvecs[i], markerLength * 0.5f);
            }
            // Actualize carrying
            cout << "idsCurrent = " << ids.size() << endl;
            cout << "remainId = " << remainId << endl;
            
            for (unsigned int i = 0; i < ids.size(); i++) {
                poses2[ids[i]] = Affine3d(rvecs[i],tvecs[i]);
            }
        }

        // Show rejected markers if there are
        if (showRejected && rejected.size() > 0)
            aruco::drawDetectedMarkers(imageCopy, rejected, noArray(), Scalar(100, 0, 255));

        cout << "----------------" << endl;

        namedWindow("RPi Camera", CV_WINDOW_AUTOSIZE);
        imshow("RPi Camera", imageCopy);

        char key = (char) waitKey(waitTime);
        if (key == 27) {
            //            com.halt();
            //            destroyWindow("RPi Camera");
            file.close();
            break;
        }
    }

    return 0;
}


