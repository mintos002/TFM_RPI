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
#include <condition_variable>
#include <atomic>
#include <thread>

#include "communication.h"
#include "rt_communication.h"
#include "led_handler.h"
#include "capture_frame.h"

#define STOP_ID 49
#define TOOL_DO 4

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
// Read camera calibration parameters

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
// Rotate an angle defined by rot to rvec vector

static void rotation(Vec3d& rvec, Vec3d& tvec, Vec3d& rot, Vec3d& nrvec) {
    Affine3d pose = Affine3d(rvec, tvec);
    Affine3d rotation = Affine3d::Identity().rotate(rot);
    pose = pose * rotation;
    nrvec = pose.rvec();
}
// Bonding aruco vectors with robot tool vectors to be able to control the robot more intuitively

static void setPointOfView(int id, Vec3d& rvec, Vec3d& tvec, Affine3d poseRef, int actualPlane, Vec3d& robotToMarkerTvec, Vec3d& robotToMarkerRvec) {
    ofstream file;
    //    ofstream fileX;
    //    ofstream fileY;
    //    ofstream fileZ;
    //    ofstream fileXt;
    //    ofstream fileYt;
    //    ofstream fileZt;
    file.open("setPointOfView.txt", fstream::in | fstream::out | fstream::app);
    //    fileX.open("forExcel_X.txt", fstream::in | fstream::out | fstream::app);
    //    fileY.open("forExcel_Y.txt", fstream::in | fstream::out | fstream::app);
    //    fileZ.open("forExcel_Z.txt", fstream::in | fstream::out | fstream::app);
    //    fileXt.open("forExcel_Xt.txt", fstream::in | fstream::out | fstream::app);
    //    fileYt.open("forExcel_Yt.txt", fstream::in | fstream::out | fstream::app);
    //    fileZt.open("forExcel_Zt.txt", fstream::in | fstream::out | fstream::app);
    file << "______________________" << endl;

    double x;
    double y;
    double z;

    Vec3d vr90X = Vec3d(CV_PI / 2, 0, 0);
    Vec3d vr90Y = Vec3d(0, CV_PI / 2, 0);
    Vec3d vr90Z = Vec3d(0, 0, CV_PI / 2);

    Vec3d vr90mX = Vec3d(-CV_PI / 2, 0, 0);
    Vec3d vr90mY = Vec3d(0, -CV_PI / 2, 0);
    Vec3d vr90mZ = Vec3d(0, 0, -CV_PI / 2);

    Vec3d nrvec;

    // x, y, z reference values in real robot for the different positions
    switch (id) {
        case 0:
            //TCP_3
            //            x = 0.38883;
            //            y = -0.11571;
            //            z = 0.48144;
            //TCP_2_up
            //            x = 0.29681;
            //            y = -0.11403;
            //            z = 0.61060;

            //TCP_2_mid
            //            x = 0.42680;
            //            y = -0.11684;
            //            z = 0.48060;
            //            rotation(rvec, tvec, vr90mY, nrvec);
            //            rotation(nrvec, tvec, vr90mZ, nrvec);
            //            rotation(nrvec, tvec, vr90mZ, nrvec);
            //            nrvec = rvec;

            //            robotToMarkerRvec(0) = nrvec(0);
            //            robotToMarkerRvec(1) = nrvec(1);
            //            robotToMarkerRvec(2) = nrvec(2);

            //TCP_2_down
            x = 0.29680;
            y = -0.11554;
            z = 0.18190;

            nrvec = rvec;

            break;
        case 1:
            //TCP_3
            //            x = -0.11571;
            //            y = -0.38883;
            //            z = 0.48144;
            //TCP_2_up
            //            x = -0.11403;
            //            y = -0.29681;
            //            z = 0.61060;

            //TCP_2_mid
            //            x = -0.11684;
            //            y = -0.42680;
            //            z = 0.48060;
            //            rotation(rvec, tvec, vr90mZ, nrvec);
            //            rotation(nrvec, tvec, vr90mY, nrvec);
            //            
            //            cout << "nrvec: " << endl << nrvec << endl;
            //            
            //            robotToMarkerRvec(0) = nrvec(0)/*-rvec(1)*/;
            //            robotToMarkerRvec(1) = nrvec(1)/*fmod(rvec(0) - M_PI / 2, 2 * M_PI)*/;
            //            robotToMarkerRvec(2) = nrvec(2) /*rvec(2)*/;

            //TCP_2_down
            x = -0.11554;
            y = -0.29680;
            z = 0.18190;
            nrvec = rvec;
            rotation(nrvec, tvec, vr90mY, nrvec);

            break;
        case 2:
            //TCP_3
            //            x = -0.38883;
            //            y = 0.11571;
            //            z = 0.48144;

            //TCP_2_up
            //            x = -0.29681;
            //            y = 0.11403;
            //            z = 0.61060;

            //TCP_2_mid
            //            x = -0.42680;
            //            y = 0.11684;
            //            z = 0.48060;
            //            nrvec = rvec;
            //            rotation(nrvec, tvec, vr90mY, nrvec);
            ////            rotation(nrvec, tvec, vr90mY, nrvec);
            //            
            //            cout << "nrvec: " << endl << nrvec << endl;
            //            
            //            robotToMarkerRvec(0) = nrvec(0)/*-rvec(1)*/;
            //            robotToMarkerRvec(1) = nrvec(1)/*fmod(rvec(0) - M_PI / 2, 2 * M_PI)*/;
            //            robotToMarkerRvec(2) = nrvec(2) /*rvec(2)*/;

            //TCP_2_down
            x = -0.29680;
            y = 0.11554;
            z = 0.18190;

            nrvec = rvec;
            rotation(nrvec, tvec, vr90Y, nrvec);
            rotation(nrvec, tvec, vr90Y, nrvec);

            break;
        case 3:
            //TCP_3
            //            x = 0.11571;
            //            y = 0.38883;
            //            z = 0.48144;

            //TCP_2_up
            //            x = 0.11403;
            //            y = 0.29681;
            //            z = 0.61060;

            //TCP_2_mid
            //            x = 0.11684;
            //            y = 0.42680;
            //            z = 0.48060;
            //            nrvec = rvec;
            //            rotation(nrvec, tvec, vr90Z, nrvec);
            //            rotation(nrvec, tvec, vr90mY, nrvec);
            //            
            //            cout << "nrvec: " << endl << nrvec << endl;
            //            
            //            robotToMarkerRvec(0) = nrvec(0)/*-rvec(1)*/;
            //            robotToMarkerRvec(1) = nrvec(1)/*fmod(rvec(0) - M_PI / 2, 2 * M_PI)*/;
            //            robotToMarkerRvec(2) = nrvec(2) /*rvec(2)*/;

            //TCP_2_down
            x = 0.11554;
            y = 0.29680;
            z = 0.18190;

            nrvec = rvec;
            rotation(nrvec, tvec, vr90Y, nrvec);

            break;
        case 4:
            //TCP_3
            //            x = 0.29764;
            //            y = -0.11476;
            //            z = 0.21986;

            //TCP_2_down
            //            x = 0.29689;
            //            y = -0.11553;
            //            z = 0.18189;

            //TCP_2_up
            //            x = 0.29680;
            //            y = -0.11405;
            //            z = 0.61058;

            switch (actualPlane) {
                case 40:
                    x = 0.29681;
                    y = -0.11403;
                    z = 0.61060;

                    nrvec = rvec;
                    rotation(nrvec, tvec, vr90mZ, nrvec);
                    rotation(nrvec, tvec, vr90mZ, nrvec);

                    break;
                case 41:
                    x = -0.11403;
                    y = -0.29681;
                    z = 0.61060;

                    nrvec = rvec;
                    rotation(nrvec, tvec, vr90mY, nrvec);
                    rotation(nrvec, tvec, vr90mZ, nrvec);
                    rotation(nrvec, tvec, vr90mZ, nrvec);

                    break;
                case 42:
                    x = -0.29681;
                    y = 0.11403;
                    z = 0.61060;

                    nrvec = rvec;
                    rotation(nrvec, tvec, vr90mY, nrvec);
                    rotation(nrvec, tvec, vr90mY, nrvec);
                    rotation(nrvec, tvec, vr90mZ, nrvec);
                    rotation(nrvec, tvec, vr90mZ, nrvec);

                    break;
                case 43:
                    x = 0.11403;
                    y = 0.29681;
                    z = 0.61060;

                    nrvec = rvec;
                    rotation(nrvec, tvec, vr90Y, nrvec);
                    rotation(nrvec, tvec, vr90mZ, nrvec);
                    rotation(nrvec, tvec, vr90mZ, nrvec);

                    break;
                default:
                    break;
            }
            break;
        case 5:
            //TCP_3
            //            x = 0.29764;
            //            y = -0.11476;
            //            z = 0.57258;

            //TCP_2_up
            //            x = 0.29689;
            //            y = -0.11401;
            //            z = 0.61057;

            //TCP_2_down
            //            x = 0.42680;
            //            y = -0.11684;
            //            z = 0.48060;

            switch (actualPlane) {
                case 50:
                    x = 0.42680;
                    y = -0.11684;
                    z = 0.48060;

                    nrvec = rvec;
                    rotation(nrvec, tvec, vr90mX, nrvec);

                    break;
                case 51:
                    x = -0.11684;
                    y = -0.42680;
                    z = 0.48060;

                    nrvec = rvec;
                    rotation(nrvec, tvec, vr90mY, nrvec);
                    rotation(nrvec, tvec, vr90mX, nrvec);

                    break;
                case 52:
                    x = -0.42680;
                    y = 0.11684;
                    z = 0.48060;

                    nrvec = rvec;
                    rotation(nrvec, tvec, vr90mY, nrvec);
                    rotation(nrvec, tvec, vr90mY, nrvec);
                    rotation(nrvec, tvec, vr90mX, nrvec);

                    break;
                case 53:
                    x = 0.11684;
                    y = 0.42680;
                    z = 0.48060;

                    nrvec = rvec;
                    rotation(nrvec, tvec, vr90Y, nrvec);
                    rotation(nrvec, tvec, vr90mX, nrvec);

                    break;
                default:
                    break;
            }
            break;
        default:
            break;
    }

    Affine3d poseActual = Affine3d(rvec, tvec);
    Affine3d distFromRef = poseActual * poseRef.inv();
    //    Vec3d ntvec = distFromRef.translation();
    Vec3d ntvec = tvec - poseRef.translation();
    //    Vec3d robotToMarker;

    // change perspective, where markerToRobot = [z,x,-y]
    // translation vector
    robotToMarkerTvec(0) = -ntvec(2) + x;
    robotToMarkerTvec(1) = ntvec(0) + y;
    robotToMarkerTvec(2) = -ntvec(1) + z;
    // rotation vector
    robotToMarkerRvec(0) = -nrvec(2);
    robotToMarkerRvec(1) = nrvec(0);
    robotToMarkerRvec(2) = -nrvec(1);

    //Plots
    //    cout << "_________________________" << endl;
    //    cout << "RobotToMarkerTvec: " << endl << robotToMarkerTvec << endl;
    //    cout << "RobotToMarkerRvec: " << endl << robotToMarkerRvec << endl;

    //    fileX << rvec(0) << endl;
    //    fileY << rvec(1) << endl;
    //    fileZ << rvec(2) << endl;
    //    fileXt << tvec(0) << endl;
    //    fileYt << tvec(1) << endl;
    //    fileZt << tvec(2) << endl;
    //    fileX.close();
    //    fileY.close();
    //    fileZ.close();
    //    fileXt.close();
    //    fileYt.close();
    //    fileZt.close();

    file << "tvec: " << endl;
    file << "[";
    file << tvec(0) << ", ";
    file << tvec(1) << ", ";
    file << tvec(2) << "]" << endl;

    file << "rvec: " << endl;
    file << "[";
    file << rvec(0) << ", ";
    file << rvec(1) << ", ";
    file << rvec(2) << "]" << endl;

    file << "reftvec: " << endl;
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

    file << "robotToMarkerRvec: " << endl;
    file << "[";
    file << robotToMarkerRvec(0) << ", ";
    file << robotToMarkerRvec(1) << ", ";
    file << robotToMarkerRvec(2);
    file << "]" << endl;

    //    cout << "rvec: " << endl << rvec << endl;
    //    cout << "tvec: " << endl << tvec << endl;
    //    cout << "poseActual: " << endl << poseActual.matrix << endl;
    //    cout << "poseRef: " << endl << poseRef.matrix << endl;
    //    cout << "distFromRef: " << endl << distFromRef.matrix << endl;
    //    cout << "TRANSLATION REF POINT" << endl << ntvec << endl;
    //    cout << "_________________________" << endl;
    file << "_________________________" << endl;
    file.close();
}
// Check all markers in the frame and sends the necessary information to the robot

static void markerProcesor(RtCommunication* com, Communication* comN, vector< int >& ids, vector< Vec3d >& rvecs, vector< Vec3d >& tvecs, atomic<bool>* e_stop, bool& ToolIsOpen, int& remainId, int& actualPlane, Affine3d& poseRef, vector< Affine3d >& poses2, Vec3d& lastSPOVrvec) { // Marcador antiguo marcador nuevo
    // check if there are any marker detected
    if (ids.size() < 1) return;

    //////    // if STOP marker is detected stop robot
    //////    bool ok = false;
    //////    for (int i = 0; i < ids.size(); i++) {
    //////        if (ids[i] == STOP_ID) {
    //////            while (!ok) {
    //////                com->setSpeed(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    //////                print_warning("STOP marker detected!");
    //////                return;
    //////            }
    //////        }
    //////    }

    // initatie detection tag and posible next position
    bool detected = false;
    int nextpp = -1;
    int w = -1;
    bool err = true;

    //////    // if robot movement is not waiting
    //////    if (wait == -1) {
    // check for marker ids, if id = 6 or 7 open/close the tool, if is = to last Id, move to that point.
    for (int i = 0; i < ids.size(); i++) {
        if (ids[i] == 6 || ids[i] == 7) {
            switch (ids[i]) {
                case 6:
                    if (!ToolIsOpen) {
                        if (com->set_digital_out(TOOL_DO, true)) ToolIsOpen = true;
                    }
                    break;
                case 7:
                    if (ToolIsOpen) {
                        if (com->set_digital_out(TOOL_DO, false)) ToolIsOpen = false;
                    }
                    break;
            }
        } else {
            if (ids[i] >= 0 && ids[i] <= 5) {
                if (ids[i] == remainId && (ids[i] == actualPlane || ids[i] == (int) actualPlane / 10)) {
                    // if detected, send offset movement and exit the function
                    detected = true;
                    Vec3d poseTFinal;
                    Vec3d poseRFinal;
                    Vec3d poseDif = tvecs[i] - poses2[ids[i]].translation();

                    ofstream file;
                    file.open("poseDif.txt", fstream::in | fstream::out | fstream::app);
                    file << "Posedif: " << endl << "[" << poseDif(0) << ", " << poseDif(2) << ", " << poseDif(2) << "]" << endl;
                    file.close();

                    // setting a limit to not overload the server by positions
                    double limT = 0.02;
                    double limR = 0.02;
                    setPointOfView(ids[i], rvecs[i], tvecs[i], poseRef, actualPlane, poseTFinal, poseRFinal);
                    Vec3d rotDif = poseRFinal - lastSPOVrvec;
                    if (abs(poseDif(0)) > limT || abs(poseDif(1)) > limT || abs(poseDif(2)) > limT ||
                            abs(rotDif(0)) > limR || abs(rotDif(1)) > limR || abs(rotDif(2)) > limR) {
                        com->movejp(poseTFinal(0), poseTFinal(1), poseTFinal(2), poseRFinal(0), poseRFinal(1), poseRFinal(2));
                    }
                    lastSPOVrvec = poseRFinal;
                    //                return;                                                                                                                               CHANGES
                } else {
                    w = i;
                    nextpp = ids[i];
                }
            }
        }
    }
    //////    }

    // if remainId is not in ids, go to nextpp reference position
    //////    if (!detected && wait == -1) {
    if (!detected) {
        remainId = nextpp;
        // To be able to switch betwen planes without issues, ap needs to be substracted by 40 or 50
        int ap = actualPlane;
        if (ap < 45 && ap > 39) ap = actualPlane - 40;
        if (ap < 55 && ap > 49) ap = actualPlane - 50;

        switch (remainId) {
            case 0:
                //                com->movej(0, -M_PI / 2, -M_PI / 2, 0, M_PI / 2, M_PI / 2);
                // Check if it was send correctly before updateing 'actualPlane' variable
                if (com->movej(0, -M_PI / 2, -M_PI / 2, -M_PI / 2, M_PI / 2, M_PI / 2)) {
                    actualPlane = 0;
                    err = false;
                }
                break;

            case 1:
                //                com->movej(-M_PI / 2, -M_PI / 2, -M_PI / 2, 0, M_PI / 2, M_PI / 2);
                if (com->movej(-M_PI / 2, -M_PI / 2, -M_PI / 2, -M_PI / 2, M_PI / 2, M_PI / 2)) {
                    actualPlane = 1;
                    err = false;
                }
                break;

            case 2:
                //                com->movej(-M_PI, -M_PI / 2, -M_PI / 2, 0, M_PI / 2, M_PI / 2);
                if (com->movej(-M_PI, -M_PI / 2, -M_PI / 2, -M_PI / 2, M_PI / 2, M_PI / 2)) {
                    actualPlane = 2;
                    err = false;
                }
                break;

            case 3:
                //                com->movej(M_PI / 2, -M_PI / 2, -M_PI / 2, 0, M_PI / 2, M_PI / 2);
                if (com->movej(M_PI / 2, -M_PI / 2, -M_PI / 2, -M_PI / 2, M_PI / 2, M_PI / 2)) {
                    actualPlane = 3;
                    err = false;
                }
                break;

            case 4:
                //                com->movej(0, -M_PI / 2, -M_PI / 2, -M_PI / 2, M_PI / 2, M_PI / 2);
                //                com->movej(0, -M_PI / 2, -M_PI / 2, -M_PI / 2, -M_PI / 2, M_PI / 2);
                // ActualPlane depends ond last 'actualPlane' variable
                switch (ap) {
                    case 0:
                        if (com->movej(0, -M_PI / 2, -M_PI / 2, -M_PI / 2, -M_PI / 2, M_PI / 2)) {
                            actualPlane = 40;
                            err = false;
                        }
                        break;
                    case 1:
                        if (com->movej(-M_PI / 2, -M_PI / 2, -M_PI / 2, -M_PI / 2, -M_PI / 2, M_PI / 2)) {
                            actualPlane = 41;
                            err = false;
                        }
                        break;
                    case 2:
                        if (com->movej(-M_PI, -M_PI / 2, -M_PI / 2, -M_PI / 2, -M_PI / 2, M_PI / 2)) {
                            actualPlane = 42;
                            err = false;
                        }
                        break;
                    case 3:
                        if (com->movej(M_PI / 2, -M_PI / 2, -M_PI / 2, -M_PI / 2, -M_PI / 2, M_PI / 2)) {
                            actualPlane = 43;
                            err = false;
                        }
                        break;
                    default:
                        if (com->movej(0, -M_PI / 2, -M_PI / 2, -M_PI / 2, -M_PI / 2, M_PI / 2)) {
                            actualPlane = 40;
                            err = false;
                        }
                        break;

                }
                //                actualPlane = 4;
                break;

            case 5:
                //                com->movej(0, -M_PI / 2, -M_PI / 2, -M_PI / 2, -M_PI / 2, M_PI / 2);
                //                com->movej(0, -M_PI / 2, -M_PI / 2, 0, M_PI / 2, M_PI / 2);
                //                actualPlane = 5;
                switch (ap) {
                    case 0:
                        if (com->movej(0, -M_PI / 2, -M_PI / 2, 0, M_PI / 2, M_PI / 2)) {
                            actualPlane = 50;
                            err = false;
                        }
                        break;
                    case 1:
                        if (com->movej(-M_PI / 2, -M_PI / 2, -M_PI / 2, 0, M_PI / 2, M_PI / 2)) {
                            actualPlane = 51;
                            err = false;
                        }
                        break;
                    case 2:
                        if (com->movej(-M_PI, -M_PI / 2, -M_PI / 2, 0, M_PI / 2, M_PI / 2)) {
                            actualPlane = 52;
                            err = false;
                        }
                        break;
                    case 3:
                        if (com->movej(M_PI / 2, -M_PI / 2, -M_PI / 2, 0, M_PI / 2, M_PI / 2)) {
                            actualPlane = 53;
                            err = false;
                        }
                        break;
                    default:
                        if (com->movej(0, -M_PI / 2, -M_PI / 2, 0, M_PI / 2, M_PI / 2)) {
                            actualPlane = 50;
                            err = false;
                        }
                        break;
                }
                break;

            default:
                return;
        }
        //////        // increment wait variable
        //////        if (!err) {
        //////            // wait for the robot to go to refPoint if there is no error
        //////            wait++;
        //////            // save reference position
        //////            poseRef = Affine3d(rvecs[w], tvecs[w]);
        //////        }
        //////    }

        //////    if (wait > -1) {
        //////        // wait 5s, the robot will take 4s to reach the reference position
        //////        if (wait == 0) {
        //////            print_info("The robot is going to the initial position...");
        //////            // wait a second because communication in port 30002 is 10Hz
        //////            usleep(1000000);
        //////        }
        //////
        //////        // check if port comN is conected and program is running
        //////        if (comN->connected && comN->robot_state->isProgramRunning() && wait < 22) {
        //////            // wait while program is running
        //////
        //////            // Wait till robot stop moveing
        //////            cout << "Waiting" << endl;
        //////            usleep(150000);
        //////            wait++;
        //////
        //////        } else { // else, wait 4.5 seconds
        //////            if (wait < 22) {
        //////                usleep(150000);
        //////                wait++;
        //////            }
        //////        }
        //////
        //////        if (wait >= 22) wait = -1;

        // wait 5s, the robot will take 4s to reach the reference position
        print_info("The robot is going to the initial position...");

        // save reference position
        poseRef = Affine3d(rvecs[w], tvecs[w]);
        // wait for the robot to go to refPoint if there is no error
        if (!err) {
            usleep(1000000);
            // check if port comN is conected and program is running
            if (comN->connected && comN->robot_state->isProgramRunning()) {
                int i = 0;
                // wait while program is running
                while (comN->robot_state->isProgramRunning() && i < 44 && !*e_stop) {
                    cout << "MP E_STOP: " << *e_stop << endl;
                    // Wait till robot stop moveing
                    //                    cout << "Waiting" << endl;
                    // because every frame takes 125ms aprox
                    usleep(125000);
                    i++;
                }
            } else { // else, wait 4.5 seconds
                usleep(4500000);
            }
        }
    }
}

/*
 * // /////////////////////////////////////////////////////////////////////////////////
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


    // LED
    //    LedHandler ledH;
    bool ToolIsOpen = false;
    // COMMUNICATION TEST
    //    RtDataHandler dataHandler;
    //    RtCommunication com("192.168.238.142");
    condition_variable msg_cond;
    condition_variable msg_cond_rt;

    RtCommunication* com = new RtCommunication(msg_cond_rt, "158.42.206.10");
    Communication* comN = new Communication(msg_cond, "158.42.206.10");
    comN->start();
    usleep(2000000);
    double version = comN->robot_state->getVersion();
    cout << "Version: " << endl << version << endl;
    com->rt_data_handler->setVersion(version);
    com->start();
    string ip_addr = com->getLocalIp();
    cout << "IP = " << ip_addr << endl;

    // open an close tool until the sended cmd is ok
    bool ok = false;
    while (!ok) {
        if (com->set_digital_out(TOOL_DO, true)) {
            ToolIsOpen = true;
            ok = true;
        }
        usleep(1000000);
        if (com->set_digital_out(TOOL_DO, false)) {
            ToolIsOpen = false;
            ok = true;
        }
        usleep(1000000);
    }

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

    // Variables
    double totalTime = 0;
    int totalIterations = 0;

    double tTime = 0;
    int tIterations = 0;

    vector< Affine3d > poses2(8);
    Vec3d lastSPOVrvec;
    Affine3d poseRef;
    int remainId = -1;
    int actualPlane = -1;
    int wait = -1;

    CaptureFrame* capture_frame = new CaptureFrame(inputVideo, detectorParams, dictionary, STOP_ID);
    capture_frame->start();
    usleep(1000000);
    atomic<bool>* E_STOP = &capture_frame->E_STOP;
    bool keepalive = true;
    ofstream fileP;

    // ////////////////////////////////////////////////////////////////
    int loop_counter = 0;
    //    while (inputVideo.grab()) {
    while (keepalive) {
        //        while (!capture_frame->E_STOP) {
        while (!*E_STOP) {
            loop_counter = 0;
            double t = (double) getTickCount();
            // Get Robot parameters
            cout << "** isProgramRunning? =" << comN->robot_state->isProgramRunning() << endl << endl;
            cout << "** isEmergencyStopped? =" << comN->robot_state->isEmergencyStopped() << endl << endl;
            cout << "** isProtectiveStopped? =" << comN->robot_state->isProtectiveStopped() << endl << endl;

            //            cout << "Digital Outputs = " << com->rt_data_handler->getDigitalOutputs() << endl;
            //            cout << "Program state = " << com->rt_data_handler->getProgramState() << endl;
            vector< double > tool = com->rt_data_handler->getToolVectorActual();
            cout << "Tool vector actual = " << "[";
            for (int i = 0; i < tool.size(); i++)
                cout << tool[i] << " ";
            cout << "]" << endl;

            vector< double > tool1 = comN->robot_state->getToolInfo();
            cout << "Tool info = " << "[";
            for (int i = 0; i < tool1.size(); i++)
                cout << tool1[i] << " ";
            cout << "]" << endl;


            cout << "E_STOP: " << *E_STOP << endl;

            //////            cout << "WAIIIIT: " << wait << endl;

            //////            Mat image, imageCopy;
            //////        inputVideo.retrieve(image);
            //////            image = capture_frame->getImage();


            // START time measuring
            double tick = (double) getTickCount();

            vector< int > ids;
            vector< vector< Point2f > > corners, rejected;
            vector< Vec3d > rvecs, tvecs;

            // detect markers and estimate pose
            //////        aruco::detectMarkers(image, dictionary, corners, ids, detectorParams, rejected);
            Mat image, imageCopy;
            //            image = capture_frame->getImage();
            //            ids = capture_frame->getIds();
            //            corners = capture_frame->getCorners();
            //            rejected = capture_frame->getRegected();
            capture_frame->cfDetectMarkers(image, corners, ids, rejected);

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
            //        int x = 270;
            //        int y = 190;
            //        int width = 100;
            //        int height = width;
            //        Point pt1(x, y), pt2(x + width, y + height);
            //        // tvecs ~ 0.49
            //        rectangle(imageCopy, pt1, pt2, Scalar(255, 0, 0), 2);
            //
            //        line(imageCopy, Point(0, 360), Point(640, 360), Scalar(255, 0, 255), 2, 8);
            //        line(imageCopy, Point(320, 0), Point(320, 480), Scalar(255, 0, 255), 2, 8);

            // Draw marker vectors
            if (ids.size() > 0) {
                aruco::drawDetectedMarkers(imageCopy, corners, ids);
                //            cout << "%%%%%%%%%%%%%%%%%%%%%%%%%%" << endl << "CORNERS: " << endl << corners[0] << endl << "%%%%%%%%%%%%%%%%%%%%%%%%%%" << endl;

                cout << "----------------" << endl;
                // Process the distance btw markers and reference point
                //                if (!capture_frame->E_STOP)
                markerProcesor(com, comN, ids, rvecs, tvecs, E_STOP, ToolIsOpen, remainId, actualPlane, poseRef, poses2, lastSPOVrvec);

                for (unsigned int i = 0; i < ids.size(); i++) {
                    aruco::drawAxis(imageCopy, camMatrix, distCoeffs, rvecs[i], tvecs[i], markerLength * 0.5f);
                }
                // Actualize carrying
                cout << "idsCurrent = " << ids.size() << endl;
                cout << "remainId = " << remainId << endl;
                cout << "actualPlane = " << actualPlane << endl;

                for (unsigned int i = 0; i < ids.size(); i++) {
                    poses2[ids[i]] = Affine3d(rvecs[i], tvecs[i]);
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
                //            destroyWindow("RPi Camera");
                file.close();
                capture_frame->halt();
                com->halt();
                comN->halt();
                keepalive = false;
                break;
            }
            double cTime = ((double) getTickCount() - t) / getTickFrequency();
            tTime += cTime;
            tIterations++;
            if (tIterations % vidFps == 0) {
                fileP.open("robotParams.txt", fstream::in | fstream::out | fstream::app);
                fileP << "Detection Time TOTAL = " << cTime * 1000 << " ms " << "(Mean = " << 1000 * tTime / double(tIterations) << " ms)" << endl;
                fileP.close();
            }
        }
        bool ok = false;
        if (loop_counter == 0) {
            while (!ok)
                ok = com->setSpeed(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        }
        loop_counter++;
        // if E_STOP print warning
        if (*E_STOP) /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            print_warning("STOP marker detected!");
        // stop leds if exit
        if (!keepalive)
            com->ledH.stopLeds();

    }
    return 0;
}
