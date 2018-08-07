/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   rt_data_handler.cpp
 * Author: minto
 * 
 * Created on 23 de abril de 2018, 21:15
 */

#include "rt_data_handler.h"

RtDataHandler::RtDataHandler(std::condition_variable& msg_cond) {
    version = 3.3;
    time = 0.0;
    q_target.assign(6, 0.0);
    qd_target.assign(6, 0.0);
    qdd_target.assign(6, 0.0);
    i_target.assign(6, 0.0);
    m_target.assign(6, 0.0);
    q_actual.assign(6, 0.0);
    qd_actual.assign(6, 0.0);
    i_actual.assign(6, 0.0);
    i_control.assign(6, 0.0);
    tool_vector_actual.assign(6, 0.0);
    tcp_speed_actual.assign(6, 0.0);
    tcp_force.assign(6, 0.0);
    tool_vector_target.assign(6, 0.0);
    tcp_speed_target.assign(6, 0.0);
    digital_input_bits.assign(64, false);
    motor_temperatures.assign(6, 0.0);
    controller_timer = 0.0;
    robot_mode = 0.0;
    joint_modes.assign(6, 0.0);
    safety_mode = 0.0;
    tool_accelerometer_values.assign(3, 0.0);
    speed_scaling = 0.0;
    linear_momentum_norm = 0.0;
    v_main = 0.0;
    v_robot = 0.0;
    i_robot = 0.0;
    v_actual.assign(6, 0.0);
    digital_outputs = 0.0;
    program_state = 0.0;
    
    data_published = false;
    controller_updated = false;
    pMsg_cond= &msg_cond;
}

RtDataHandler::~RtDataHandler() {
    data_published = true;
    controller_updated = true;
    pMsg_cond->notify_all();
}

void RtDataHandler::setDataPublished() {
    data_published = false;
}

bool RtDataHandler::getDataPublished() {
    return data_published;
}

void RtDataHandler::setControllerUpdated() {
    controller_updated = false;
}

bool RtDataHandler::getControllerUpdated() {
    return controller_updated;
}

double RtDataHandler::ntohd(uint64_t nf) {
    double x;
    nf = be64toh(nf);
    memcpy(&x, &nf, sizeof (x));
    return x;
}

std::vector<double> RtDataHandler::unpackVector(uint8_t * buf, int start_index, int nr_of_vals) {
    uint64_t q;
    std::vector<double> ret;
    for (int i = 0; i < nr_of_vals; i++) {
        memcpy(&q, &buf[start_index + i * sizeof (q)], sizeof (q));
        ret.push_back(ntohd(q));
    }
    return ret;
}

std::vector<bool> RtDataHandler::unpackDigitalInputBits(int64_t data) {
    std::vector<bool> ret;
    for (int i = 0; i < 64; i++) {
        ret.push_back((data & (1 << i)) >> i);
    }
    return ret;
}

void RtDataHandler::setVersion(double ver) {
    val_lock.lock();
    version = ver;
    val_lock.unlock();
}

double RtDataHandler::getVersion() {
    double ret;
    val_lock.lock();
    ret = version;
    val_lock.unlock();
    return ret;
}

double RtDataHandler::getTime() {
    double ret;
    val_lock.lock();
    ret = time;
    val_lock.unlock();
    return ret;
}

std::vector<double> RtDataHandler::getQTarget() {
    std::vector<double> ret;
    val_lock.lock();
    ret = q_target;
    val_lock.unlock();
    return ret;
}

std::vector<double> RtDataHandler::getQdTarget() {
    std::vector<double> ret;
    val_lock.lock();
    ret = qd_target;
    val_lock.unlock();
    return ret;
}

std::vector<double> RtDataHandler::getQddTarget() {
    std::vector<double> ret;
    val_lock.lock();
    ret = qdd_target;
    val_lock.unlock();
    return ret;
}

std::vector<double> RtDataHandler::getITarget() {
    std::vector<double> ret;
    val_lock.lock();
    ret = i_target;
    val_lock.unlock();
    return ret;
}

std::vector<double> RtDataHandler::getMTarget() {
    std::vector<double> ret;
    val_lock.lock();
    ret = m_target;
    val_lock.unlock();
    return ret;
}

std::vector<double> RtDataHandler::getQActual() {
    std::vector<double> ret;
    val_lock.lock();
    ret = q_actual;
    val_lock.unlock();
    return ret;
}

std::vector<double> RtDataHandler::getQdActual() {
    std::vector<double> ret;
    val_lock.lock();
    ret = qd_actual;
    val_lock.unlock();
    return ret;
}

std::vector<double> RtDataHandler::getIActual() {
    std::vector<double> ret;
    val_lock.lock();
    ret = i_actual;
    val_lock.unlock();
    return ret;
}

std::vector<double> RtDataHandler::getIControl() {
    std::vector<double> ret;
    val_lock.lock();
    ret = i_control;
    val_lock.unlock();
    return ret;
}

std::vector<double> RtDataHandler::getToolVectorActual() {
    std::vector<double> ret;
    val_lock.lock();
    ret = tool_vector_actual;
    val_lock.unlock();
    return ret;
}

std::vector<double> RtDataHandler::getTcpSpeedActual() {
    std::vector<double> ret;
    val_lock.lock();
    ret = tcp_speed_actual;
    val_lock.unlock();
    return ret;
}

std::vector<double> RtDataHandler::getTcpForce() {
    std::vector<double> ret;
    val_lock.lock();
    ret = tcp_force;
    val_lock.unlock();
    return ret;
}

std::vector<double> RtDataHandler::getToolVectorTarget() {
    std::vector<double> ret;
    val_lock.lock();
    ret = tool_vector_target;
    val_lock.unlock();
    return ret;
}

std::vector<double> RtDataHandler::getTcpSpeedTarget() {
    std::vector<double> ret;
    val_lock.lock();
    ret = tcp_speed_target;
    val_lock.unlock();
    return ret;
}

std::vector<bool> RtDataHandler::getDigitalInputBits() {
    std::vector<bool> ret;
    val_lock.lock();
    ret = digital_input_bits;
    val_lock.unlock();
    return ret;
}

std::vector<double> RtDataHandler::getMotorTemperatures() {
    std::vector<double> ret;
    val_lock.lock();
    ret = motor_temperatures;
    val_lock.unlock();
    return ret;
}

double RtDataHandler::getControllerTimer() {
    double ret;
    val_lock.lock();
    ret = controller_timer;
    val_lock.unlock();
    return ret;
}

double RtDataHandler::getRobotMode() {
    double ret;
    val_lock.lock();
    ret = robot_mode;
    val_lock.unlock();
    return ret;
}

std::vector<double> RtDataHandler::getJointModes() {
    std::vector<double> ret;
    val_lock.lock();
    ret = joint_modes;
    val_lock.unlock();
    return ret;
}

double RtDataHandler::getSafety_mode() {
    double ret;
    val_lock.lock();
    ret = safety_mode;
    val_lock.unlock();
    return ret;
}

std::vector<double> RtDataHandler::getToolAccelerometerValues() {
    std::vector<double> ret;
    val_lock.lock();
    ret = tool_accelerometer_values;
    val_lock.unlock();
    return ret;
}

double RtDataHandler::getSpeedScaling() {
    double ret;
    val_lock.lock();
    ret = speed_scaling;
    val_lock.unlock();
    return ret;
}

double RtDataHandler::getLinearMomentumNorm() {
    double ret;
    val_lock.lock();
    ret = linear_momentum_norm;
    val_lock.unlock();
    return ret;
}

double RtDataHandler::getVMain() {
    double ret;
    val_lock.lock();
    ret = v_main;
    val_lock.unlock();
    return ret;
}

double RtDataHandler::getVRobot() {
    double ret;
    val_lock.lock();
    ret = v_robot;
    val_lock.unlock();
    return ret;
}

double RtDataHandler::getIRobot() {
    double ret;
    val_lock.lock();
    ret = i_robot;
    val_lock.unlock();
    return ret;
}

std::vector<double> RtDataHandler::getVActual() {
    std::vector<double> ret;
    val_lock.lock();
    ret = v_actual;
    val_lock.unlock();
    return ret;
}

double RtDataHandler::getDigitalOutputs() {
    double ret;
    val_lock.lock();
    ret = digital_outputs;
    val_lock.unlock();
    return ret;
}

double RtDataHandler::getProgramState() {
    double ret;
    val_lock.lock();
    ret = program_state;
    val_lock.unlock();
    return ret;
}

double RtDataHandler::getElbowPosition() {
    double ret;
    val_lock.lock();
    ret = elbow_position;
    val_lock.unlock();
    return ret;
}

double RtDataHandler::getElbowVelocity() {
    double ret;
    val_lock.lock();
    ret = elbow_velocity;
    val_lock.unlock();
    return ret;
}

int y=0;
int n=0;
void RtDataHandler::unpack(uint8_t * buf) {
    int64_t digital_input_bits_;
    uint64_t unpack_to;
    uint16_t offset = 0;
    val_lock.lock();
    int len;
    memcpy(&len, &buf[offset], sizeof (len));
    // Message Size
    offset += sizeof (len);
//    printf("/n LEN= %i\n", len);

    len = ntohl(len); // function converts the unsigned integer netlong from network byte order to host byte order.

    //Check the correct message length is received
    bool len_good = true;
    if (version >= 1.6 && version < 1.7) { //v1.6
        if (len != 756)
            len_good = false;
    } else if (version >= 1.7 && version < 1.8) { //v1.7
        if (len != 764)
            len_good = false;
    } else if (version >= 1.8 && version < 1.9) { //v1.8
        if (len != 812)
            len_good = false;
    } else if (version >= 3.0 && version < 3.2) { //v3.0 & v3.1
        if (len != 1044)
            len_good = false;
    } else if (version >= 3.2 && version <= 3.4) { //v3.2
        if (len != 1060)
            len_good = false;
    } else if (version >= 3.5) { // v.3.5
        if (len != 1108)
            len_good = false;
    }
    
    if(len == 1060){
        y++;
    } else {
        n++;
    }
//////    printf("\n Y | N = ( %i, %i ) -> Y(%f) \n", y, n, (double)y/(y+n));

    if (!len_good) {
        val_lock.unlock();
        return;
    }

    memcpy(&unpack_to, &buf[offset], sizeof (unpack_to));
    time = RtDataHandler::ntohd(unpack_to);
//    printf("time = %f\n", time);
    offset += sizeof (double);
    q_target = unpackVector(buf, offset, 6);
    offset += sizeof (double) * 6;
    qd_target = unpackVector(buf, offset, 6);
    offset += sizeof (double) * 6;
    qdd_target = unpackVector(buf, offset, 6);
    offset += sizeof (double) * 6;
    i_target = unpackVector(buf, offset, 6);
    offset += sizeof (double) * 6;
    m_target = unpackVector(buf, offset, 6);
    offset += sizeof (double) * 6;
    q_actual = unpackVector(buf, offset, 6);
    offset += sizeof (double) * 6;
    qd_actual = unpackVector(buf, offset, 6);
    offset += sizeof (double) * 6;
    i_actual = unpackVector(buf, offset, 6);
    offset += sizeof (double) * 6;
    if (version <= 1.9) {
        if (version > 1.6)
            tool_accelerometer_values = unpackVector(buf, offset, 3);
        offset += sizeof (double) * (3 + 15);
        tcp_force = unpackVector(buf, offset, 6);
        offset += sizeof (double) * 6;
        tool_vector_actual = unpackVector(buf, offset, 6);
        offset += sizeof (double) * 6;
        tcp_speed_actual = unpackVector(buf, offset, 6);
    } else {
        i_control = unpackVector(buf, offset, 6);
        offset += sizeof (double) * 6;
        tool_vector_actual = unpackVector(buf, offset, 6);
//        printf("\n TCP= [%f, %f, %f, %f, %f, %f] \n", tool_vector_actual[0], tool_vector_actual[1], tool_vector_actual[2], tool_vector_actual[3], tool_vector_actual[4], tool_vector_actual[5]);
        offset += sizeof (double) * 6;
        tcp_speed_actual = unpackVector(buf, offset, 6);
        offset += sizeof (double) * 6;
        tcp_force = unpackVector(buf, offset, 6);
        offset += sizeof (double) * 6;
        tool_vector_target = unpackVector(buf, offset, 6);
        offset += sizeof (double) * 6;
        tcp_speed_target = unpackVector(buf, offset, 6);
    }
    offset += sizeof (double) * 6;

    memcpy(&digital_input_bits_, &buf[offset], sizeof (digital_input_bits_));
    digital_input_bits = unpackDigitalInputBits(be64toh(digital_input_bits_));
    offset += sizeof (double);
    motor_temperatures = unpackVector(buf, offset, 6);
    offset += sizeof (double) * 6;
    memcpy(&unpack_to, &buf[offset], sizeof (unpack_to));
    controller_timer = ntohd(unpack_to);
    if (version > 1.6) {
        offset += sizeof (double) * 2;
        memcpy(&unpack_to, &buf[offset], sizeof (unpack_to));
        robot_mode = ntohd(unpack_to);
        if (version > 1.7) {
            offset += sizeof (double);
            joint_modes = unpackVector(buf, offset, 6);
        }
    }
    if (version > 1.8) {
        offset += sizeof (double) * 6;
        memcpy(&unpack_to, &buf[offset], sizeof (unpack_to));
        safety_mode = ntohd(unpack_to);
        offset += sizeof (double);
        tool_accelerometer_values = unpackVector(buf, offset, 3);
        offset += sizeof (double) * 3;
        memcpy(&unpack_to, &buf[offset], sizeof (unpack_to));
        speed_scaling = ntohd(unpack_to);
        offset += sizeof (double);
        memcpy(&unpack_to, &buf[offset], sizeof (unpack_to));
        linear_momentum_norm = ntohd(unpack_to);
        offset += sizeof (double);
        memcpy(&unpack_to, &buf[offset], sizeof (unpack_to));
        v_main = ntohd(unpack_to);
        offset += sizeof (double);
        memcpy(&unpack_to, &buf[offset], sizeof (unpack_to));
        v_robot = ntohd(unpack_to);
        offset += sizeof (double);
        memcpy(&unpack_to, &buf[offset], sizeof (unpack_to));
        i_robot = ntohd(unpack_to);
        offset += sizeof (double);
        v_actual = unpackVector(buf, offset, 6);
    }
    if (version >= 3.2) {
        offset += sizeof (double) * 6;
        memcpy(&unpack_to, &buf[offset], sizeof (unpack_to));
        digital_outputs = ntohd(unpack_to);
        offset += sizeof (double);
        memcpy(&unpack_to, &buf[offset], sizeof (unpack_to));
        program_state = ntohd(unpack_to);
        if (version >= 3.5) {
            offset += sizeof (double);
            memcpy(&unpack_to, &buf[offset], sizeof (unpack_to));
            elbow_position = ntohd(unpack_to);
            offset += sizeof (double);
            memcpy(&unpack_to, &buf[offset], sizeof (unpack_to));
            elbow_velocity = ntohd(unpack_to);
        }
    }
    val_lock.unlock();
    controller_updated = true;
    data_published = true;
    pMsg_cond->notify_all();
}

