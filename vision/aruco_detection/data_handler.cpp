/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   data_handler.cpp
 * Author: minto
 * 
 * Created on 8 de marzo de 2018, 16:30
 */

#include "data_handler.h"

DataHandler::DataHandler(std::condition_variable& msg_cond) {
    version_msg.majorVersion = 0;
    version_msg.minorVersion = 0;
    new_data_available = false;
    pMsg_cond = &msg_cond;
    DataHandler::setDisconnected();
    robot_mode_running = robotModeV33::ROBOT_MODE_RUNNING;
}

double DataHandler::ntohd(uint64_t nf) {
    double x;
    nf = be64toh(nf); //These functions convert the byte encoding of integer values from the byte order that the current CPU (the "host") uses, to and from little-endian and big-endian byte order.
    memcpy(&x, &nf, sizeof (x));
    return x;
}

void DataHandler::unpack(uint8_t* buffer, unsigned int buffer_length) {
    //printf("**START_unpack\n");

    /* Returns missing bytes to unpack a message, or 0 if all data was parsed */
    unsigned int offset = 0;
    while (buffer_length > offset) {
        int len;
        unsigned char message_type;
        memcpy(&len, &buffer[offset], sizeof (len));
        len = ntohl(len); // function converts the unsigned integer netlong from network byte order to host byte order.
        if (len + offset > buffer_length) {
            return;
        }
        memcpy(&message_type, &buffer[offset + sizeof (len)], sizeof (message_type));
        switch (message_type) {
            case messageType::ROBOT_MESSAGE:
                //printf("**...ROBOT_MESSAGE\n");
                DataHandler::unpackRobotMessage(buffer, offset, len); //'len' is inclusive the 5 bytes from messageSize and messageType
                break;
            case messageType::ROBOT_STATE:
                //printf("**...ROBOT_STATE\n");
                DataHandler::unpackRobotState(buffer, offset, len); //'len' is inclusive the 5 bytes from messageSize and messageType
                break;
            case messageType::PROGRAM_STATE_MESSAGE:
                //printf("**...ROBOT_STATE_MESSAGE\n");
                //Don't do anithing atm...
            default:
                break;
        }
        offset += len;
    }
    return;
    //printf("**END_unpack\n");

}

void DataHandler::unpackRobotMessage(uint8_t* buffer, unsigned int offset, uint32_t len) {
    //printf("**START_unpackRobotMessage\n");

    offset += 5;
    uint64_t timestamp;
    int8_t source, robot_message_type;
    memcpy(&timestamp, &buffer[offset], sizeof (timestamp));
    offset += sizeof (timestamp);
    memcpy(&source, &buffer[offset], sizeof (source));
    offset += sizeof (source);
    memcpy(&robot_message_type, &buffer[offset], sizeof (robot_message_type));
    offset += sizeof (robot_message_type);
    switch (robot_message_type) {
        case robotMessageType::ROBOT_MESSAGE_VERSION:
            val_lock.lock();
            version_msg.timestamp = timestamp;
            version_msg.source = source;
            version_msg.robotMessageType = robot_message_type;
            DataHandler::unpackRobotMessageVersion(buffer, offset, len);
            val_lock.unlock();
            //printf("**...ROBOT_MESSAGE_VERSION\n");
            break;
        default:
            break;
    }
    //printf("**END_unpackRobotMessage\n");
}

void DataHandler::unpackRobotState(uint8_t* buffer, unsigned int offset, uint32_t len) {
    //printf("**START_unpackRobotState\n");

    offset += 5;
    while (offset < len) {
        int32_t length;
        uint8_t package_type;
        memcpy(&length, &buffer[offset], sizeof (length));
        length = ntohl(length);
        memcpy(&package_type, &buffer[offset] + sizeof (length), sizeof (package_type));
        switch (package_type) {
            case packageType::ROBOT_MODE_DATA:
                val_lock.lock();
                DataHandler::unpackRobotMode(buffer, offset + 5);
                //printf("**...ROBOT_MODE_DATA\n");
                val_lock.unlock();
                break;
            case packageType::MASTERBOARD_DATA:
                val_lock.lock();
                DataHandler::unpackRobotStateMasterboard(buffer, offset + 5);
                //printf("**...MASTERBOARD_DATA\n");
                val_lock.unlock();
                break;
            case packageType::CARTESIAN_INFO:
                val_lock.lock();
                DataHandler::unpackCartesianInfo(buffer, offset + 5);
                val_lock.unlock();
                break;
            default:
                break;
        }
        offset += length;
    }
    new_data_available = true;
    pMsg_cond ->notify_all();
    //printf("**END_unpackRobotState\n");
}

void DataHandler::unpackRobotMessageVersion(uint8_t* buffer, unsigned int offset, uint32_t len) {
    //printf("**START_unpackRobotMessageVertion\n");

    memcpy(&version_msg.projectNameSize, &buffer[offset], sizeof (version_msg.projectNameSize));
    offset += sizeof (version_msg.projectNameSize);
    memcpy(&version_msg.projectName, &buffer[offset], sizeof (char) * version_msg.projectNameSize);
    offset += version_msg.projectNameSize;
    version_msg.projectName[version_msg.projectNameSize] = '\0';
    memcpy(&version_msg.majorVersion, &buffer[offset], sizeof (version_msg.majorVersion));
    offset += sizeof (version_msg.majorVersion);
    memcpy(&version_msg.minorVersion, &buffer[offset], sizeof (version_msg.minorVersion));
    offset += sizeof (version_msg.minorVersion);
    // vertion 3.3
    memcpy(&version_msg.bugfixVersion, &buffer[offset], sizeof (version_msg.bugfixVersion));
    offset += sizeof (version_msg.bugfixVersion);
    version_msg.bugfixVersion = ntohl(version_msg.bugfixVersion);
    memcpy(&version_msg.buildNumber, &buffer[offset], sizeof (version_msg.buildNumber));
    offset += sizeof (version_msg.buildNumber);
    version_msg.buildNumber = ntohl(version_msg.buildNumber);
    // END_vertion 3.3
    memcpy(&version_msg.buildDate, &buffer[offset], sizeof (char) * len - offset);
    version_msg.buildDate[len - offset] = '\0';
    if (version_msg.majorVersion < 2) {
        robot_mode_running = robotModeV18::ROBOT_RUNNING_MODE;
    }
    //printf("**END_unpackRobotMessageVertion\n");

}

void DataHandler::unpackRobotMode(uint8_t * buffer, unsigned int offset) {
    //printf("**START_unpackRobotMode\n");

    memcpy(&robot_mode.timestamp, &buffer[offset], sizeof (robot_mode.timestamp));
    offset += sizeof (robot_mode.timestamp);
    uint8_t tmp;
    memcpy(&tmp, &buffer[offset], sizeof (tmp));
    if (tmp > 0) {
        robot_mode.isRobotConnected = true;
    } else {
        robot_mode.isRobotConnected = false;
    }
    offset += sizeof (tmp);
    memcpy(&tmp, &buffer[offset], sizeof (tmp));
    if (tmp > 0) {
        robot_mode.isRealRobotEnabled = true;
    } else {
        robot_mode.isRealRobotEnabled = false;
    }
    offset += sizeof (tmp);
    memcpy(&tmp, &buffer[offset], sizeof (tmp));
    if (tmp > 0) {
        robot_mode.isPowerOnRobot = true;
    } else {
        robot_mode.isPowerOnRobot = false;
    }
    offset += sizeof (tmp);
    memcpy(&tmp, &buffer[offset], sizeof (tmp));
    if (tmp > 0) {
        robot_mode.isEmergencyStopped = true;
    } else {
        robot_mode.isEmergencyStopped = false;
    }
    offset += sizeof (tmp);
    memcpy(&tmp, &buffer[offset], sizeof (tmp));
    if (tmp > 0) {
        robot_mode.isProtectiveStopped = true;
    } else {
        robot_mode.isProtectiveStopped = false;
    }
    offset += sizeof (tmp);
    memcpy(&tmp, &buffer[offset], sizeof (tmp));
    if (tmp > 0) {
        robot_mode.isProgramRunning = true;
    } else {
        robot_mode.isProgramRunning = false;
    }
    offset += sizeof (tmp);
    memcpy(&tmp, &buffer[offset], sizeof (tmp));
    if (tmp > 0)
        robot_mode.isProgramPaused = true;
    else
        robot_mode.isProgramPaused = false;
    offset += sizeof (tmp);
    memcpy(&robot_mode.robotMode, &buffer[offset], sizeof (robot_mode.robotMode));
    offset += sizeof (robot_mode.robotMode);
    uint64_t temp;
    if (DataHandler::getVersion() > 2.) {
        memcpy(&robot_mode.controlMode, &buffer[offset], sizeof (robot_mode.controlMode));
        offset += sizeof (robot_mode.controlMode);
        memcpy(&temp, &buffer[offset], sizeof (temp));
        offset += sizeof (temp);
        robot_mode.targetSpeedFraction = DataHandler::ntohd(temp);
    }
    memcpy(&temp, &buffer[offset], sizeof (temp));
    offset += sizeof (temp);
    robot_mode.speedScaling = DataHandler::ntohd(temp);
    //printf("**END_unpackRobotMode\n");

    //if (robot_mode.isProgramRunning == true) {
//        printf("** isProgramRunning = %s\n", robot_mode.isProgramRunning ? "true" : "false");
    //}
}

void DataHandler::unpackRobotStateMasterboard(uint8_t* buffer, unsigned int offset) {
    //printf("**START_unpackRobotStateMasterboard\n");

    if (DataHandler::getVersion() < 3.0) {
        int16_t digital_input_bits, digital_output_bits;
        memcpy(&digital_input_bits, &buffer[offset], sizeof (digital_input_bits));
        offset += sizeof (digital_input_bits);
        memcpy(&digital_output_bits, &buffer[offset], sizeof (digital_output_bits));
        offset += sizeof (digital_output_bits);
        mb_data.digitalInputBits = ntohs(digital_input_bits);
        mb_data.digitalOutputBits = ntohs(digital_output_bits);
    } else {
        memcpy(&mb_data.digitalInputBits, &buffer[offset], sizeof (mb_data.digitalInputBits));
        offset += sizeof (mb_data.digitalInputBits);
        memcpy(&mb_data.digitalOutputBits, &buffer[offset], sizeof (mb_data.digitalOutputBits));
        offset += sizeof (mb_data.digitalOutputBits);
        mb_data.digitalInputBits = ntohs(mb_data.digitalInputBits);
        mb_data.digitalOutputBits = ntohs(mb_data.digitalOutputBits);
    }

    memcpy(&mb_data.analogInputRange0, &buffer[offset], sizeof (mb_data.analogInputRange0));
    offset += sizeof (mb_data.analogInputRange0);
    memcpy(&mb_data.analogInputRange1, &buffer[offset], sizeof (mb_data.analogInputRange1));
    offset += sizeof (mb_data.analogInputRange1);
    uint64_t temp;
    memcpy(&temp, &buffer[offset], sizeof (temp));
    offset += sizeof (temp);
    mb_data.analogInput0 = DataHandler::ntohd(temp);
    memcpy(&temp, &buffer[offset], sizeof (temp));
    offset += sizeof (temp);
    mb_data.analogInput1 = DataHandler::ntohd(temp);
    memcpy(&mb_data.analogOutputDomain0, &buffer[offset], sizeof (mb_data.analogOutputDomain0));
    offset += sizeof (mb_data.analogOutputDomain0);
    memcpy(&mb_data.analogOutputDomain1, &buffer[offset], sizeof (mb_data.analogOutputDomain1));
    offset += sizeof (mb_data.analogOutputDomain1);
    memcpy(&temp, &buffer[offset], sizeof (temp));
    offset += sizeof (temp);
    mb_data.analogOutput0 = DataHandler::ntohd(temp);
    memcpy(&temp, &buffer[offset], sizeof (temp));
    offset += sizeof (temp);
    mb_data.analogOutput1 = DataHandler::ntohd(temp);

    memcpy(&mb_data.masterBoardTemperature, &buffer[offset], sizeof (mb_data.masterBoardTemperature));
    offset += sizeof (mb_data.masterBoardTemperature);
    mb_data.masterBoardTemperature = ntohl(mb_data.masterBoardTemperature);
    memcpy(&mb_data.robotVoltage48V, &buffer[offset], sizeof (mb_data.robotVoltage48V));
    offset += sizeof (mb_data.robotVoltage48V);
    mb_data.robotVoltage48V = ntohl(mb_data.robotVoltage48V);
    memcpy(&mb_data.robotCurrent, &buffer[offset], sizeof (mb_data.robotCurrent));
    offset += sizeof (mb_data.robotCurrent);
    mb_data.robotCurrent = ntohl(mb_data.robotCurrent);
    memcpy(&mb_data.masterIOCurrent, &buffer[offset], sizeof (mb_data.masterIOCurrent));
    offset += sizeof (mb_data.masterIOCurrent);
    mb_data.masterIOCurrent = ntohl(mb_data.masterIOCurrent);

    memcpy(&mb_data.safetyMode, &buffer[offset], sizeof (mb_data.safetyMode));
    offset += sizeof (mb_data.safetyMode);

    // Ojo Versiones ................
    memcpy(&mb_data.masterOnOffState, &buffer[offset], sizeof (mb_data.masterOnOffState));
    offset += sizeof (mb_data.masterOnOffState);

    memcpy(&mb_data.euromap67InterfaceInstalled, &buffer[offset], sizeof (mb_data.euromap67InterfaceInstalled));
    offset += sizeof (mb_data.euromap67InterfaceInstalled);
    if (mb_data.euromap67InterfaceInstalled != 0) {
        memcpy(&mb_data.euromapInputBits, &buffer[offset], sizeof (mb_data.euromapInputBits));
        offset += sizeof (mb_data.euromapInputBits);
        mb_data.euromapInputBits = ntohl(mb_data.euromapInputBits);
        memcpy(&mb_data.euromapOutputBits, &buffer[offset], sizeof (mb_data.euromapOutputBits));
        offset += sizeof (mb_data.euromapOutputBits);
        mb_data.euromapOutputBits = ntohl(mb_data.euromapOutputBits);
        if (DataHandler::getVersion() < 3.0) {
            int16_t euromap_voltage, euromap_current;
            memcpy(&euromap_voltage, &buffer[offset], sizeof (euromap_voltage));
            offset += sizeof (euromap_voltage);
            memcpy(&euromap_current, &buffer[offset], sizeof (euromap_current));
            offset += sizeof (euromap_current);
            mb_data.euromapVoltage = ntohs(euromap_voltage);
            mb_data.euromapCurrent = ntohs(euromap_current);
        } else {
            memcpy(&mb_data.euromapVoltage, &buffer[offset], sizeof (mb_data.euromapVoltage));
            offset += sizeof (mb_data.euromapVoltage);
            mb_data.euromapVoltage = ntohl(mb_data.euromapVoltage);
            memcpy(&mb_data.euromapCurrent, &buffer[offset], sizeof (mb_data.euromapCurrent));
            offset += sizeof (mb_data.euromapCurrent);
            mb_data.euromapCurrent = ntohl(mb_data.euromapCurrent);
        }

    }
        //printf("**END_unpackRobotStateMasterboard\n");

}

void DataHandler::unpackCartesianInfo(uint8_t * buffer, unsigned int offset) {
    uint64_t temp;
    memcpy(&temp, &buffer[offset], sizeof (temp));
    offset += sizeof (temp);
    
    cartesian_i.toolX = DataHandler::ntohd(temp);
    memcpy(&temp, &buffer[offset], sizeof (temp));
    offset += sizeof (temp);
    cartesian_i.toolY = DataHandler::ntohd(temp);
    memcpy(&temp, &buffer[offset], sizeof (temp));
    offset += sizeof (temp);
    cartesian_i.toolZ = DataHandler::ntohd(temp);
    memcpy(&temp, &buffer[offset], sizeof (temp));
    offset += sizeof (temp);
    cartesian_i.toolRx = DataHandler::ntohd(temp);
    memcpy(&temp, &buffer[offset], sizeof (temp));
    offset += sizeof (temp);
    cartesian_i.toolRy = DataHandler::ntohd(temp);
    memcpy(&temp, &buffer[offset], sizeof (temp));
    offset += sizeof (temp);
    cartesian_i.toolRz = DataHandler::ntohd(temp);
    memcpy(&temp, &buffer[offset], sizeof (temp));
    offset += sizeof (temp);
    
    if(DataHandler::getVersion() > 3.0) {
        cartesian_i.TCPOffsetX = DataHandler::ntohd(temp);
        memcpy(&temp, &buffer[offset], sizeof (temp));
        offset += sizeof (temp);
        cartesian_i.TCPOffsetY = DataHandler::ntohd(temp);
        memcpy(&temp, &buffer[offset], sizeof (temp));
        offset += sizeof (temp);
        cartesian_i.TCPOffsetZ = DataHandler::ntohd(temp);
        memcpy(&temp, &buffer[offset], sizeof (temp));
        offset += sizeof (temp);
        cartesian_i.TCPOffsetRx = DataHandler::ntohd(temp);
        memcpy(&temp, &buffer[offset], sizeof (temp));
        offset += sizeof (temp);
        cartesian_i.TCPOffsetRy = DataHandler::ntohd(temp);
        memcpy(&temp, &buffer[offset], sizeof (temp));
        offset += sizeof (temp);
        cartesian_i.TCPOffsetRz = DataHandler::ntohd(temp);
        memcpy(&temp, &buffer[offset], sizeof (temp));
        offset += sizeof (temp);
    }
}

std::vector<double> DataHandler::getToolInfo() {
    std::vector<double> vec;
    double a[6];
    val_lock.lock();
    a[0] = cartesian_i.toolX;
    a[1] = cartesian_i.toolY;
    a[2] = cartesian_i.toolZ;
    a[3] = cartesian_i.toolRx;
    a[4] = cartesian_i.toolRy;
    a[5] = cartesian_i.toolRz;
    vec.insert(vec.end(), std::begin(a), std::end(a));
    val_lock.unlock();
    return vec;
}

std::vector<double> DataHandler::getTCPInfo() {
    std::vector<double> vec;
    double a[6];
    val_lock.lock();
    a[0] = cartesian_i.TCPOffsetX;
    a[1] = cartesian_i.TCPOffsetY;
    a[2] = cartesian_i.TCPOffsetZ;
    a[3] = cartesian_i.TCPOffsetRx;
    a[4] = cartesian_i.TCPOffsetRy;
    a[5] = cartesian_i.TCPOffsetRz;
    vec.insert(vec.end(), std::begin(a), std::end(a));
    val_lock.unlock();
    return vec;
}

double DataHandler::getVersion() {
    //printf("**START_getVersion\n");
    double ver;
    val_lock.lock();
    ver = version_msg.majorVersion + 0.1 * version_msg.minorVersion + .01 * version_msg.bugfixVersion + 0.00001 * version_msg.buildNumber;
    val_lock.unlock();
    //printf("**END_getVersion\n");

    return ver;
}

void DataHandler::finishedReading() {
    new_data_available = false;
}

bool DataHandler::getNewDataAvailable() {
    return new_data_available;
}

int DataHandler::getDigitalInputBits() {
    return mb_data.digitalInputBits;
}

int DataHandler::getDigitalOutputBits() {
    return mb_data.digitalOutputBits;
}

double DataHandler::getAnalogInput0() {
    return mb_data.analogInput0;
}

double DataHandler::getAnalogInput1() {
    return mb_data.analogInput1;
}

double DataHandler::getAnalogOutput0() {
    return mb_data.analogOutput0;

}

double DataHandler::getAnalogOutput1() {
    return mb_data.analogOutput1;
}

bool DataHandler::isRobotConnected() {
    return robot_mode.isRobotConnected;
}

bool DataHandler::isRealRobotEnabled() {
    return robot_mode.isRealRobotEnabled;
}

bool DataHandler::isPowerOnRobot() {
    return robot_mode.isPowerOnRobot;
}

bool DataHandler::isEmergencyStopped() {
    return robot_mode.isEmergencyStopped;
}

bool DataHandler::isProtectiveStopped() {
    return robot_mode.isProtectiveStopped;
}

bool DataHandler::isProgramRunning() {
    return robot_mode.isProgramRunning;
}

bool DataHandler::isProgramPaused() {
    return robot_mode.isProgramPaused;
}

unsigned char DataHandler::getRobotMode() {
    return robot_mode.robotMode;
}

bool DataHandler::isReady() {
    if (robot_mode.robotMode == robot_mode_running) {
        return true;
    }
    return false;
}

void DataHandler::setDisconnected() {
    robot_mode.isRobotConnected = false;
    robot_mode.isRealRobotEnabled = false;
    robot_mode.isPowerOnRobot = false;
}

