// Copyright[2017] <Walking Machine> [copyright]

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <Kinova.API.CommLayerUbuntu.h>
#include <KinovaTypes.h>
#include <unistd.h>
#include <stdlib.h>
#include <string>
#include <fstream>
#include <iostream>
#include <cmath>
#include <dlfcn.h>
#include <stdio.h>

#include "Lib/sara_arm_hardware_interface.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8MultiArray.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Bool.h"

#include <vector>
#include <stdio.h>


int main(int argc, char **argv) {

    bool Succes = false;
    while (!Succes) {
        // We load the library
        commandLayer_handle = dlopen("Kinova.API.USBCommandLayerUbuntu.so", RTLD_NOW|RTLD_GLOBAL);

        // We load the functions from the library
        MyInitAPI = (int (*)()) dlsym(commandLayer_handle, "InitAPI");
        MyCloseAPI = (int (*)()) dlsym(commandLayer_handle, "CloseAPI");
        MyMoveHome = (int (*)()) dlsym(commandLayer_handle, "MoveHome");
        MyEraseAllTrajectories = (int (*)()) dlsym(commandLayer_handle, "EraseAllTrajectories");
        MySetJointZero = (int (*)(int ActuatorAdress)) dlsym(commandLayer_handle, "SetJointZero");
        MyInitFingers = (int (*)()) dlsym(commandLayer_handle, "InitFingers");
        MyGetDevices = (int (*)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result)) dlsym(commandLayer_handle, "GetDevices");
        MySetActiveDevice = (int (*)(KinovaDevice devices)) dlsym(commandLayer_handle, "SetActiveDevice");
        MySendBasicTrajectory = (int (*)(TrajectoryPoint)) dlsym(commandLayer_handle, "SendBasicTrajectory");
        MySendAdvanceTrajectory = (int (*)(TrajectoryPoint)) dlsym(commandLayer_handle, "SendAdvanceTrajectory");
        MyGetAngularCommand = (int (*)(AngularPosition &)) dlsym(commandLayer_handle, "GetAngularCommand");
        MySetActuatorMaxVelocity = (int (*)(float &)) dlsym(commandLayer_handle, "SetActuatorMaxVelocity");
        MyGetSensorsInfo = (int (*)(SensorsInfo &)) dlsym(commandLayer_handle, "GetSensorsInfo");

        MyGetActuatorsPosition = (int (*)(float *)) dlsym(commandLayer_handle, "GetActuatorsPosition");
        MyGetAngularVelocity = (int (*)(float *)) dlsym(commandLayer_handle, "GetAngularVelocity");

        // << ----   I N I T I A L I S A T I O N   ---- >>
        if ((MyInitAPI == NULL) || (MyCloseAPI == NULL) || (MySendBasicTrajectory == NULL) ||
            (MySendAdvanceTrajectory == NULL) || (MyMoveHome == NULL) || (MyInitFingers == NULL)) {
            //      cout << "* * *  E R R O R   D U R I N G   I N I T I A L I Z A T I O N  * * *" << endl;
            //     cout << "* * *          T R Y I N G   A G A I N   I N   A  S E C      * * *" << endl;
            sleep(1);
        } else {
            Succes = true;
        }
    }
    Succes = false;
    int nb_attemps = 1;
    while (!Succes) {
        result = (*MyInitAPI)();
        devicesCount = MyGetDevices(devices, result);
        if (result != 1) {
            //       cout << "* * *                  N O   A R M   F O U N D               * * *" << endl;
            //       cout << "* * *                   A T T E M P   N B   " << nb_attemps << "              * * *" << endl;
            //      cout << "* * *              S E A R C H I N G   A G A I N . . .       * * *" << endl;
            nb_attemps++;
            sleep(1);
        } else {
            Succes = true;
            //       cout << "* * *                    A R M   F O U N D                 * * *" << endl;
        }
    }
//    cout << "I N I T I A L I Z A T I O N   C O M P L E T E D" << endl << endl;
    //   cout << "Initialization's result :" << result << endl;
//    cout << "Number of attemps :" << nb_attemps << endl;
    // ROS
    // Initialisation
    ros::init(argc, argv, "sara_arm");
    // Obtention du nodehandle
    ros::NodeHandle n;
    // ros::spin();
    ros::MultiThreadedSpinner spinner(4);  // Use 4 threads
    spinner.spin();
}

MyRobot::MyRobot() {
    // connect and register the joint state interface
    hardware_interface::JointStateHandle state_handle_Motor1("Motor1", &pos[0], &vel[0], &eff[0]);
    jnt_state_interface.registerHandle(state_handle_Motor1);
    hardware_interface::JointStateHandle state_handle_Motor2("Motor2", &pos[1], &vel[1], &eff[1]);
    jnt_state_interface.registerHandle(state_handle_Motor2);
    hardware_interface::JointStateHandle state_handle_Motor3("Motor3", &pos[2], &vel[2], &eff[2]);
    jnt_state_interface.registerHandle(state_handle_Motor3);
    hardware_interface::JointStateHandle state_handle_Motor4("Motor4", &pos[3], &vel[3], &eff[3]);
    jnt_state_interface.registerHandle(state_handle_Motor4);
    hardware_interface::JointStateHandle state_handle_Motor5("Motor5", &pos[4], &vel[4], &eff[4]);
    jnt_state_interface.registerHandle(state_handle_Motor5);
    /*
    hardware_interface::JointStateHandle state_handle_Motor6("Motor6", &pos[5], &vel[5], &eff[5]);
    jnt_state_interface.registerHandle(state_handle_Motor6);
    hardware_interface::JointStateHandle state_handle_Motor7("Motor7", &pos[6], &vel[6], &eff[6]);
    jnt_state_interface.registerHandle(state_handle_Motor7);
    */
    registerInterface(&jnt_state_interface);

    // connect and register the joint velocity interface
    hardware_interface::JointHandle vel_handle_Motor1(VelocityJointInterface.getHandle("Motor1"), &cmd[0]);
    VelocityJointInterface.registerHandle(vel_handle_Motor1);
    hardware_interface::JointHandle vel_handle_Motor2(VelocityJointInterface.getHandle("Motor2"), &cmd[1]);
    VelocityJointInterface.registerHandle(vel_handle_Motor2);
    hardware_interface::JointHandle vel_handle_Motor3(VelocityJointInterface.getHandle("Motor3"), &cmd[2]);
    VelocityJointInterface.registerHandle(vel_handle_Motor3);
    hardware_interface::JointHandle vel_handle_Motor4(VelocityJointInterface.getHandle("Motor4"), &cmd[3]);
    VelocityJointInterface.registerHandle(vel_handle_Motor4);
    hardware_interface::JointHandle vel_handle_Motor5(VelocityJointInterface.getHandle("Motor5"), &cmd[4]);
    VelocityJointInterface.registerHandle(vel_handle_Motor5);
    /*
    hardware_interface::JointHandle vel_handle_Motor6(VelocityJointInterface.getHandle("Motor6"), &cmd[3]);
    VelocityJointInterface.registerHandle(vel_handle_Motor6);
    hardware_interface::JointHandle vel_handle_Motor7(VelocityJointInterface.getHandle("Motor7"), &cmd[4]);
    VelocityJointInterface.registerHandle(vel_handle_Motor7);
    */
    registerInterface(&VelocityJointInterface);

}



void MyRobot::Update() {
    float PositionList[NOMBRE_DE_MOTEURS_KINOVA];
    float VelocityList[NOMBRE_DE_MOTEURS_KINOVA];
    TrajectoryPoint pointToSend;

    MyGetActuatorsPosition(PositionList);
    MyGetActuatorsPosition(VelocityList);
    for (int i=0; i < NOMBRE_DE_MOTEURS_KINOVA; i++) {
        // << ----  U P D A T E   S T A T U S  ---- >>
        pos[i] = PositionList[i];
        vel[i] = VelocityList[i];
        // eff[i] = 0.0F;

    }
    //  << ---- E X E C U T E   O R D E R S ---- >>
    pointToSend.SynchroType = 0;
    pointToSend.LimitationsActive = 0;
    pointToSend.Limitations.speedParameter1 = 100;
    pointToSend.Limitations.speedParameter2 = 100;
    pointToSend.Limitations.speedParameter3 = 100;
    pointToSend.Position.Type = ANGULAR_VELOCITY;
    MySendAdvanceTrajectory(pointToSend);
}




