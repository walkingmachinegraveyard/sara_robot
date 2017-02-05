#ifndef ARM_H_I
#define ARM_H_I

#include "Lib/sara_arm_hardware_interface.h"
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>


const int NOMBRE_DE_MOTEURS = 5;

class MyRobot : public hardware_interface::RobotHW {
    public:
        MyRobot();
        void init();

    private:
        hardware_interface::JointStateInterface jnt_state_interface;
        hardware_interface::PositionJointInterface jnt_pos_interface;
        double cmd[NOMBRE_DE_MOTEURS];
        double pos[NOMBRE_DE_MOTEURS];
        double vel[NOMBRE_DE_MOTEURS];
        double eff[NOMBRE_DE_MOTEURS];

        void RunRobot();

        void UpdateState();
};

#endif
