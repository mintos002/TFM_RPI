/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   capture_frame.h
 * Author: minto
 *
 * Created on 26 de junio de 2018, 16:08
 */

#ifndef CAPTURE_FRAME_H
#define CAPTURE_FRAME_H

#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <unistd.h>
#include <iostream>
#include <vector>
#include <thread>
#include <mutex>
#include <atomic>
#include <condition_variable>
#include <chrono>

#include "print_out.h"

//captureFrame(mutex val_lock, VideoCapture& inputVideo, Ptr<aruco::DetectorParameters> detectorParams, Ptr<aruco::Dictionary> dictionary ,  Mat& image, atomic<bool>& E_STOP)

class CaptureFrame {
public:
    CaptureFrame(cv::VideoCapture& inVideo, cv::Ptr<cv::aruco::DetectorParameters> detParams, cv::Ptr<cv::aruco::Dictionary> dic, int stopId);
    void start();
    void halt();
    cv::Mat getImage();
    std::vector<std::vector<cv::Point2f>> getCorners();
    std::vector<int> getIds();
    std::vector<std::vector<cv::Point2f>> getRegected();
    void cfDetectMarkers(cv::Mat& o_image, std::vector<std::vector<cv::Point2f>>& o_corners, std::vector<int>& o_ids, std::vector<std::vector<cv::Point2f>>& o_regected);
    std::atomic<bool> E_STOP;
    
private:
    cv::VideoCapture inputVideo;
    cv::Ptr<cv::aruco::DetectorParameters> detectorParams;
    cv::Ptr<cv::aruco::Dictionary> dictionary;
    cv::Mat image, imageCopy;
    std::vector<std::vector<cv::Point2f>> corners, rejected;
    std::vector<int> ids;
    int STOP_ID;
    bool isStopDetected;
    
    bool keepalive;
    
    void run();
    
    std::thread frameThread;
    std::mutex val_lock; // Locks the variables while saveing the data;
    std::condition_variable* pMsg_cond; //Signals that new vars are available
    
};

#endif /* CAPTURE_FRAME_H */

