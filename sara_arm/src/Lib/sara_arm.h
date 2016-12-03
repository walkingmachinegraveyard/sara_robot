#ifndef Animation_H_
#define Animation_H_

	#include <iostream>
	#include <dlfcn.h>
	#include <vector>
	#include <Kinova.API.CommLayerUbuntu.h>
	#include <KinovaTypes.h>
	#include <stdio.h>
	#include <unistd.h>
	#include <stdlib.h>
	#include <cmath>
	#include <string>





	int result;

	AngularPosition currentCommand;
	TrajectoryPoint pointToSend;

	//Handle for the library's command layer.
	void * commandLayer_handle;

	//Function pointers to the functions we need
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



	void Stop(  );

	int main();

	void MyGoToStart( );

	void WaitForReach(  );

	void ApplyPoint( int Speed );

	void SetJointRelPoint( int Joint, int Angle);

	void SetJointGlobPoint( int Joint, int Angle );

	// Function to go to the hardcoded starting position
	void MyGoToStart( );

	void ApplyVelocities( );


#endif
