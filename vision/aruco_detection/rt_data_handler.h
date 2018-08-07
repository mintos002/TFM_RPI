/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   rt_data_handler.h
 * Author: minto
 *
 * Created on 23 de abril de 2018, 21:15
 */

#ifndef RT_DATA_HANDLER_H
#define RT_DATA_HANDLER_H

#include <inttypes.h>
#include <vector>
#include <stdlib.h>
#include <string.h>
#include <mutex>
#include <netinet/in.h>
#include <condition_variable>


class RtDataHandler {
public:
    RtDataHandler(std::condition_variable& msg_cond);
    ~RtDataHandler();
    double getVersion();
    double getTime();
    std::vector<double> getQTarget();
    std::vector<double> getQdTarget();
    std::vector<double> getQddTarget();
    std::vector<double> getITarget();
    std::vector<double> getMTarget();
    std::vector<double> getQActual();
    std::vector<double> getQdActual();
    std::vector<double> getIActual();
    std::vector<double> getIControl();
    std::vector<double> getToolVectorActual();
    std::vector<double> getTcpSpeedActual();
    std::vector<double> getTcpForce();
    std::vector<double> getToolVectorTarget();
    std::vector<double> getTcpSpeedTarget();
    std::vector<bool> getDigitalInputBits();
    std::vector<double> getMotorTemperatures();
    double getControllerTimer();
    double getRobotMode();
    std::vector<double> getJointModes();
    double getSafety_mode();
    
    std::vector<double> getToolAccelerometerValues();
    
    double getSpeedScaling();
    double getLinearMomentumNorm();
    
    double getVMain();
    double getVRobot();
    double getIRobot();
    std::vector<double> getVActual();
    
    double getDigitalOutputs();
    double getProgramState();
    
    double getElbowPosition();
    double getElbowVelocity();
    
    void setVersion(double ver);
    void setDataPublished();
    bool getDataPublished();
    bool getControllerUpdated();
    void setControllerUpdated();
    void unpack(uint8_t * buf);
private:
    double version; //protocol version

    double time; //Time elapsed since the controller was started
    std::vector<double> q_target; //Target joint positions
    std::vector<double> qd_target; //Target joint velocities
    std::vector<double> qdd_target; //Target joint accelerations
    std::vector<double> i_target; //Target joint currents
    std::vector<double> m_target; //Target joint moments (torques)
    std::vector<double> q_actual; //Actual joint positions
    std::vector<double> qd_actual; //Actual joint velocities
    std::vector<double> i_actual; //Actual joint currents
    std::vector<double> i_control; //Joint control currents
    std::vector<double> tool_vector_actual; //Actual Cartesian coordinates of the tool: (x,y,z,rx,ry,rz), where rx, ry and rz is a rotation vector representation of the tool orientation
    std::vector<double> tcp_speed_actual; //Actual speed of the tool given in Cartesian coordinates
    std::vector<double> tcp_force; //Generalised forces in the TC
    std::vector<double> tool_vector_target; //Target Cartesian coordinates of the tool: (x,y,z,rx,ry,rz), where rx, ry and rz is a rotation vector representation of the tool orientation
    std::vector<double> tcp_speed_target; //Target speed of the tool given in Cartesian coordinates
    std::vector<bool> digital_input_bits; //Current state of the digital inputs. NOTE: these are bits encoded as int64_t, e.g. a value of 5 corresponds to bit 0 and bit 2 set high
    std::vector<double> motor_temperatures; //Temperature of each joint in degrees celsius
    double controller_timer; //Controller realtime thread execution time
    double robot_mode; //Robot mode
    std::vector<double> joint_modes; //Joint control modes
    double safety_mode; //Safety mode
    std::vector<double> tool_accelerometer_values; //Tool x,y and z accelerometer values (software version 1.7)
    double speed_scaling; //Speed scaling of the trajectory limiter
    double linear_momentum_norm; //Norm of Cartesian linear momentum
    double v_main; //Masterboard: Main voltage
    double v_robot; //Matorborad: Robot voltage (48V)
    double i_robot; //Masterboard: Robot current
    std::vector<double> v_actual; //Actual joint voltages
    double digital_outputs;
    double program_state;
    double elbow_position;
    double elbow_velocity;

    std::mutex val_lock; // Locks the variables while unpack parses data;

    std::condition_variable* pMsg_cond; //Signals that new vars are available
    bool data_published; //to avoid spurious wakes
    bool controller_updated; //to avoid spurious wakes

    std::vector<double> unpackVector(uint8_t * buf, int start_index, int nr_of_vals);
    std::vector<bool> unpackDigitalInputBits(int64_t data);
    double ntohd(uint64_t nf);
};

#endif /* RT_DATA_HANDLER_H */
