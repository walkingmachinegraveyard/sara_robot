// Copyright[2017] <Walking Machine> [copyright]

#include <list>
#include "ARM_H_I"


class MyRobot : public hardware_interface::RobotHW {

    init() {




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

            if ((MyInitAPI == NULL) || (MyCloseAPI == NULL) || (MySendBasicTrajectory == NULL) ||
                (MySendAdvanceTrajectory == NULL) || (MyMoveHome == NULL) || (MyInitFingers == NULL)) {
                cout << "* * *  E R R O R   D U R I N G   I N I T I A L I Z A T I O N  * * *" << endl;
                cout << "* * *          T R Y I N G   A G A I N   I N   A  S E C      * * *" << endl;
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
                cout << "* * *                  N O   A R M   F O U N D               * * *" << endl;
                cout << "* * *                   A T T E M P   N B   " << nb_attemps << "              * * *" << endl;
                cout << "* * *              S E A R C H I N G   A G A I N . . .       * * *" << endl;

                nb_attemps++;
                sleep(1);

            } else {
                Succes = true;
                    cout << "* * *                    A R M   F O U N D                 * * *" << endl;
            }
        }
        cout << "I N I T I A L I Z A T I O N   C O M P L E T E D" << endl << endl;

        cout << "Initialization's result :" << result << endl;
        cout << "Number of attemps :" << nb_attemps << endl;

        // ROS
        // Initialisation
        ros::init(argc, argv, "sara_arm");

        // Obtention du nodehandle
        ros::NodeHandle n;

        // ros::spin();
        ros::MultiThreadedSpinner spinner(4);  // Use 4 threads
        spinner.spin();

        return 0;
    }
















 public:
    MyRobot() {
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


       registerInterface(&jnt_state_interface);

       // connect and register the joint position interface
       hardware_interface::JointHandle pos_handle_Motor1(jnt_state_interface.getHandle("Motor1"), &cmd[0]);
       jnt_pos_interface.registerHandle(pos_handle_Motor1);

       hardware_interface::JointHandle pos_handle_Motor2(jnt_state_interface.getHandle("Motor2"), &cmd[1]);
       jnt_pos_interface.registerHandle(pos_handle_Motor2);

       hardware_interface::JointHandle pos_handle_Motor3(jnt_state_interface.getHandle("Motor3"), &cmd[2]);
       jnt_pos_interface.registerHandle(pos_handle_Motor3);

       hardware_interface::JointHandle pos_handle_Motor4(jnt_state_interface.getHandle("Motor4"), &cmd[3]);
       jnt_pos_interface.registerHandle(pos_handle_Motor4);

       hardware_interface::JointHandle pos_handle_Motor5(jnt_state_interface.getHandle("Motor5"), &cmd[4]);
       jnt_pos_interface.registerHandle(pos_handle_Motor5);


       registerInterface(&jnt_pos_interface);
    }


 private:
    hardware_interface::JointStateInterface jnt_state_interface;
    hardware_interface::PositionJointInterface jnt_pos_interface;
    double cmd[NOMBRE_DE_MOTEURS];
    double pos[NOMBRE_DE_MOTEURS];
    double vel[NOMBRE_DE_MOTEURS];
    double eff[NOMBRE_DE_MOTEURS];


    void RunRobot() {
        while (1) {
            UpdateState();
            ExecutePosition();
        }
    }

    UpdateState() {
        int i;
        for (i=0; i < NOMBRE_DE_MOTEURS; i++) {
        //    &pos[i] =



        }


    }




}


main() {
  MyRobot robot;
  controller_manager::ControllerManager cm(&robot);

  while (true) {
     robot.read();
     cm.update(robot.get_time(), robot.get_period());
     robot.write();
     sleep();
  }
}
