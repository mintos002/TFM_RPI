/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   capture_frame.cpp
 * Author: minto
 * 
 * Created on 26 de junio de 2018, 16:09
 */

#include "capture_frame.h"

CaptureFrame::CaptureFrame(cv::VideoCapture& inVideo, cv::Ptr<cv::aruco::DetectorParameters> detParams, cv::Ptr<cv::aruco::Dictionary> dic, int stopId) {
    inputVideo = inVideo;
    detectorParams = detParams;
    dictionary = dic;
    STOP_ID = stopId;
    isStopDetected = false;
    E_STOP = true;
    keepalive = false;
}

void CaptureFrame::start() {
    E_STOP = false;
    keepalive = true;
    frameThread = std::thread(&CaptureFrame::run, this);
}

void CaptureFrame::halt() {
    keepalive = false;
    E_STOP = false;
    frameThread.join();
    print_info("Camera stopped takeing frames.");
}

cv::Mat CaptureFrame::getImage() {
    cv::Mat ret;
    val_lock.lock();
    ret = image;
    val_lock.unlock();
    return ret;
}

std::vector<std::vector<cv::Point2f>> CaptureFrame::getCorners() {
    std::vector<std::vector < cv::Point2f>> ret;
    val_lock.lock();
    ret = corners;
    val_lock.unlock();
    return ret;
}

std::vector<int> CaptureFrame::getIds() {
    std::vector<int> ret;
    val_lock.lock();
    ret = ids;
    val_lock.unlock();
    return ret;
}

std::vector<std::vector<cv::Point2f>> CaptureFrame::getRegected() {
    std::vector<std::vector < cv::Point2f>> ret;
    val_lock.lock();
    ret = rejected;
    val_lock.unlock();
    return ret;
}

// To prevent sync problems it needs to send all data in a mutex
void CaptureFrame::cfDetectMarkers(cv::Mat& o_image, std::vector<std::vector<cv::Point2f>>& o_corners, std::vector<int>& o_ids, std::vector<std::vector<cv::Point2f>>& o_rejected) {
    val_lock.lock();
    o_image = image;
    o_corners = corners;
    o_ids = ids;
    o_rejected = rejected;
    val_lock.unlock();
}

void CaptureFrame::run() {
    print_debug("Camera Frame: started");
    double totalTime = 0;
    int totalIterations = 0;
    while (keepalive && inputVideo.grab()) {
        isStopDetected = false;
        double tick = (double) cv::getTickCount();
        val_lock.lock();
        inputVideo.retrieve(image);
        //        imshow("RPi C", image);
        cv::aruco::detectMarkers(image, dictionary, corners, ids, detectorParams, rejected);
        if (ids.size() > 0) {
            // Check if there is STOP marker
            for (int i = 0; i < ids.size(); i++) {
                if (ids[i] == STOP_ID) {
                    isStopDetected = true;
                }
            }
            if(isStopDetected) {
                E_STOP = true;
            } else {
                E_STOP = false;
            }
        }
        val_lock.unlock();
        double currentTime = ((double) cv::getTickCount() - tick) / cv::getTickFrequency();
        totalTime += currentTime;
        totalIterations++;
        if (totalIterations % 30 == 0) {
            std::cout <<"Detection Time CAPTURE= " << currentTime * 1000 << " ms " << "(Mean = " << 1000 * totalTime / double(totalIterations) << " ms)" << std::endl;
        }
    }
//    val_lock.lock();
    E_STOP = true;
//    val_lock.unlock();
}




