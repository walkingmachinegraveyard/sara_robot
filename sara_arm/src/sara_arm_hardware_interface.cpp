// Copyright[2017] <Walking Machine> [copyright]

#include <hardware_interface/joint_command_interface.h>
#include <controller_manager/controller_manager.h>
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











MyRobot::MyRobot() {
    // connect and register the joint state interface
    ROS_INFO("--1--");
    int i = 0;
    std::string Name;
    Name = "Motor"+boost::lexical_cast<std::string>(i+1);
    joint_state_interface_.registerHandle(hardware_interface::JointStateHandle( Name, &pos[i], &vel[i], &eff[i++]));
    ROS_INFO("--2--");
    registerInterface(&joint_state_interface_);
    ROS_INFO("--3--");
    i = 0;
    // connect and register the joint velocity interface
    Name = "Motor"+boost::lexical_cast<std::string>(i+1);
    joint_velocity_interface_.registerHandle(hardware_interface::JointHandle(joint_state_interface_.getHandle(Name), &cmd[i++]));

    ROS_INFO("--4--");

    registerInterface(&joint_velocity_interface_);

}



void MyRobot::Read() {

    //ROS_INFO("--1--");
    /*
    float PositionList[NOMBRE_DE_MOTEURS_KINOVA];
    float VelocityList[NOMBRE_DE_MOTEURS_KINOVA];
    MyGetActuatorsPosition(PositionList);
    MyGetActuatorsPosition(VelocityList);
    */
   // ROS_INFO("--2--");
    for (int i=0; i < NOMBRE_DE_MOTEURS_KINOVA; i++) {
        // << ----  U P D A T E   S T A T U S  ---- >>
/*
        ROS_INFO("--2--");

        pos[i] = PositionList[i];
        vel[i] = VelocityList[i];
        // eff[i] = 0.0F;
*/
    }
}



void MyRobot::Write() {
    TrajectoryPoint pointToSend;
    //  << ---- E X E C U T E   O R D E R S ---- >>
    /*
    pointToSend.SynchroType = 0;
    pointToSend.LimitationsActive = 0;
    pointToSend.Limitations.speedParameter1 = 100;
    pointToSend.Limitations.speedParameter2 = 100;
    pointToSend.Limitations.speedParameter3 = 100;
    pointToSend.Position.Type = ANGULAR_VELOCITY;
    MySendAdvanceTrajectory(pointToSend);
     */
}
















int main(int argc, char **argv) {

    ROS_INFO("--e--");
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
                  //cout << "* * *  E R R O R   D U R I N G   I N I T I A L I Z A T I O N  * * *" << endl;
            //     cout << "* * *          T R Y I N G   A G A I N   I N   A  S E C      * * *" << endl;
            sleep(1);
        } else {
            Succes = true;
        }
    }


    Succes = false;
    int nb_attemps = 1;
    while (!Succes) {

        ROS_INFO("--d--");
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
        Succes = true;

    }
//    cout << "I N I T I A L I Z A T I O N   C O M P L E T E D" << endl << endl;
    //   cout << "Initialization's result :" << result << endl;
//    cout << "Number of attemps :" << nb_attemps << endl;

    // ROS
    // Initialisation

    ROS_INFO("--c--");
    ros::init(argc, argv, "sara_arm");
    ROS_INFO("--c1--");
    // Obtention du nodehandle
    ros::NodeHandle n;
    ROS_INFO("--c2--");
    // ros::spin();
    ros::MultiThreadedSpinner spinner(4);  // Use 4 threads
    ROS_INFO("--c3--");
    ros::Duration period(0.02);
    ROS_INFO("--c4--");
    // Création de l'instance de Sara
    MyRobot Sara;
    ROS_INFO("--c5--");
    ROS_INFO("--b--");

    controller_manager::ControllerManager cm(&Sara, n);
    while (ros::ok())
    {
      //  ROS_INFO("--a1--");
        Sara.Read();
     //   ROS_INFO("--a2--");
        cm.update(ros::Time::now(), period);
      //  ROS_INFO("--a3--");
        Sara.Write();
    }
    spinner.spin();

}
