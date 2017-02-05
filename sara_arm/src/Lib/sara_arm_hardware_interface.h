#ifndef ARM_H_I
#define ARM_H_I

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>

const int NOMBRE_DE_MOTEURS_KINOVA = 5;
//Handle for the library's command layer.
void * commandLayer_handle;
int result;  // Tampon de réception des résultats de certaines fonctions
int devicesCount;
KinovaDevice devices[MAX_KINOVA_DEVICE];

// Function pointers to the functions we need
int (*MyInitAPI)();
int (*MyCloseAPI)();
int (*MySendBasicTrajectory)(TrajectoryPoint command);
int (*MySendAdvanceTrajectory)(TrajectoryPoint command);
int (*MyGetDevices)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result);
int (*MySetActiveDevice)(KinovaDevice device);
int (*MyMoveHome)();
int (*MySetJointZero)(int ActuatorAdress);
int (*MyInitFingers)();
int (*MyGetAngularCommand)(AngularPosition &);
int (*MyEraseAllTrajectories)();
int (*MyGetSensorsInfo)(SensorsInfo &);
int (*MySetActuatorMaxVelocity)(float &);
int (*MyGetActuatorsPosition)(float *);
int (*MyGetAngularVelocity)(float *);






class MyRobot : public hardware_interface::RobotHW {
    public:
        MyRobot();
        void init();

    private:
        hardware_interface::JointStateInterface jnt_state_interface;
        hardware_interface::VelocityJointInterface VelocityJointInterface;
        double cmd[NOMBRE_DE_MOTEURS_KINOVA];
        double pos[NOMBRE_DE_MOTEURS_KINOVA];
        double vel[NOMBRE_DE_MOTEURS_KINOVA];
        double eff[NOMBRE_DE_MOTEURS_KINOVA];
        void Update();
};

#endif