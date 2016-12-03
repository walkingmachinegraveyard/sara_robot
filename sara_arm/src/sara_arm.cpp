
#include "Lib/sara_arm.h"


KinovaDevice list[MAX_KINOVA_DEVICE];
int devicesCount;




int MyStartPosition[] = { 	180,
							90,
							34,
							270,
							325,
							0	};







using namespace std;

int main()
{

	string Animation = "RobotDance";


	//We load the library
	commandLayer_handle = dlopen("Kinova.API.USBCommandLayerUbuntu.so",RTLD_NOW|RTLD_GLOBAL);

	//We load the functions from the library
	MyInitAPI = (int (*)()) dlsym(commandLayer_handle,"InitAPI");
	MyCloseAPI = (int (*)()) dlsym(commandLayer_handle,"CloseAPI");
	MyMoveHome = (int (*)()) dlsym(commandLayer_handle,"MoveHome");
	MyEraseAllTrajectories = (int (*)()) dlsym(commandLayer_handle,"EraseAllTrajectories");
	MySetJointZero = (int (*)(int ActuatorAdress)) dlsym(commandLayer_handle,"SetJointZero");
	MyInitFingers = (int (*)()) dlsym(commandLayer_handle,"InitFingers");
	MyGetDevices = (int (*)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result)) dlsym(commandLayer_handle,"GetDevices");
	MySetActiveDevice = (int (*)(KinovaDevice devices)) dlsym(commandLayer_handle,"SetActiveDevice");
	MySendBasicTrajectory = (int (*)(TrajectoryPoint)) dlsym(commandLayer_handle,"SendBasicTrajectory");
	MySendAdvanceTrajectory = (int (*)(TrajectoryPoint)) dlsym(commandLayer_handle,"SendAdvanceTrajectory");
	MyGetAngularCommand = (int (*)(AngularPosition &)) dlsym(commandLayer_handle,"GetAngularCommand");



	pointToSend.InitStruct();




	if((MyInitAPI == NULL) || (MyCloseAPI == NULL) || (MySendBasicTrajectory == NULL) ||
	   (MySendAdvanceTrajectory == NULL) || (MyMoveHome == NULL) || (MyInitFingers == NULL))
	{
		cout << "* * *  E R R O R   D U R I N G   I N I T I A L I Z A T I O N  * * *" << endl;
	}
	else
	{
		cout << "I N I T I A L I Z A T I O N   C O M P L E T E D" << endl << endl;

		result = (*MyInitAPI)();
		devicesCount = MyGetDevices(list, result);

		cout << "Initialization's result :" << result << endl;



		for(int i = 0; i < devicesCount; i++)
		{



			cout << "Found a robot on the USB bus (" << list[i].SerialNumber << ")" << endl;

			//Setting the current device as the active device.
			MySetActiveDevice(list[i]);

			cout << "Go to start position" << endl;
			MyGoToStart( );
			WaitForReach( );



			cout << "*********************************" << endl;
			cout << "Finding animation"<< Animation << ")" << endl;


			if ( Animation == "RobotDance" ) {

				SetJointGlobPoint( 1, 260 );
				SetJointGlobPoint( 2, 90 );
				SetJointGlobPoint( 3, 102 );
				SetJointGlobPoint( 4, 270 );
				SetJointGlobPoint( 5, 300 );
				ApplyPoint( 20 );
				WaitForReach( );

				Stop(  );
				float Speed;
				float Period;

				Period = 5000;

				for( float i=0; i<(5.0/Period*1000); i++ ){

					Speed = sin(i/Period*100.0f)*40.0f;

					cout << "Speed = " << Speed << endl;

					SetJointGlobPoint( 4, Speed );
					ApplyVelocities( );
					usleep(Period);
				}
			} else if ( Animation == "HeilSara!" ) {

				SetJointGlobPoint( 1, 180 );
				SetJointGlobPoint( 2, 130 );
				SetJointGlobPoint( 3, 30 );
				SetJointGlobPoint( 4, 270 );
				SetJointGlobPoint( 5, 300 );
				ApplyPoint( 20 );
				WaitForReach( );


				SetJointRelPoint( 3, 40 );
				SetJointRelPoint( 5, 40 );
				ApplyPoint( 60 );
				WaitForReach( );


				SetJointRelPoint( 3, -40 );
				SetJointRelPoint( 5, -40 );
				ApplyPoint( 60 );
				WaitForReach( );


				MyGoToStart( );
				WaitForReach(  );

				//sleep(1);

			}




		}
		cout << endl << "C L O S I N G   A P I" << endl;
		result = (*MyCloseAPI)();
	}

	dlclose(commandLayer_handle);

	return 0;
}











void WaitForReach(  ){

	int MyTolerence = 2;
	bool ok = false;


	cout << "Waiting for reach" << endl;
	while( !ok ){

		usleep(50000);

		///cout << "Still waiting..." << endl;
		MyGetAngularCommand(currentCommand);
		ok = true;
		ok = ok && abs(currentCommand.Actuators.Actuator1-pointToSend.Position.Actuators.Actuator1)<MyTolerence;
		ok = ok && abs(currentCommand.Actuators.Actuator2-pointToSend.Position.Actuators.Actuator2)<MyTolerence;
		ok = ok && abs(currentCommand.Actuators.Actuator3-pointToSend.Position.Actuators.Actuator3)<MyTolerence;
		ok = ok && abs(currentCommand.Actuators.Actuator4-pointToSend.Position.Actuators.Actuator4)<MyTolerence;
		ok = ok && abs(currentCommand.Actuators.Actuator5-pointToSend.Position.Actuators.Actuator5)<MyTolerence;
		//ok = ok && abs(currentCommand.Actuators.Actuator6-pointToSend.Position.Actuators.Actuator6)<MyTolerence;
	}

}





void ApplyPoint( int Speed ){

	pointToSend.SynchroType = 1;
	pointToSend.LimitationsActive = 1;
	pointToSend.Limitations.speedParameter1 = Speed;
	pointToSend.Limitations.speedParameter2 = Speed;
	//pointToSend.Limitations.speedParameter3 = Speed;
	pointToSend.Position.Type = ANGULAR_POSITION;
	MySendAdvanceTrajectory(pointToSend);
}



void ApplyVelocities(  ){

	pointToSend.SynchroType = 0;
	pointToSend.LimitationsActive = 0;
	pointToSend.Limitations.speedParameter1 = 60;
	pointToSend.Limitations.speedParameter2 = 60;
	//pointToSend.Limitations.speedParameter3 = Speed;
	pointToSend.Position.Type = ANGULAR_VELOCITY;
	MySendAdvanceTrajectory(pointToSend);
}





void Stop(  ){

	MyEraseAllTrajectories();
	SetJointGlobPoint( 1, 0 );
	SetJointGlobPoint( 2, 0 );
	SetJointGlobPoint( 3, 0 );
	SetJointGlobPoint( 4, 0 );
	SetJointGlobPoint( 5, 0 );
	SetJointGlobPoint( 6, 0 );


}



void SetJointRelPoint( int Joint, int Angle ){
	MyGetAngularCommand(currentCommand);
	switch (Joint) {
		case 1:
			pointToSend.Position.Actuators.Actuator1 = currentCommand.Actuators.Actuator1+Angle;
		break;
		case 2:
			pointToSend.Position.Actuators.Actuator2 = currentCommand.Actuators.Actuator2+Angle;
		break;
		case 3:
			pointToSend.Position.Actuators.Actuator3 = currentCommand.Actuators.Actuator3+Angle;
		break;
		case 4:
			pointToSend.Position.Actuators.Actuator4 = currentCommand.Actuators.Actuator4+Angle;
		break;
		case 5:
			pointToSend.Position.Actuators.Actuator5 = currentCommand.Actuators.Actuator5+Angle;
		break;
		case 6:
			pointToSend.Position.Actuators.Actuator6 = currentCommand.Actuators.Actuator6+Angle;
		break;
	}
}




void SetJointGlobPoint( int Joint, int Angle ){
	MyGetAngularCommand(currentCommand);
	switch (Joint) {
		case 1:
			pointToSend.Position.Actuators.Actuator1 = Angle;
		break;
		case 2:
			pointToSend.Position.Actuators.Actuator2 = Angle;
		break;
		case 3:
			pointToSend.Position.Actuators.Actuator3 = Angle;
		break;
		case 4:
			pointToSend.Position.Actuators.Actuator4 = Angle;
		break;
		case 5:
			pointToSend.Position.Actuators.Actuator5 = Angle;
		break;
		case 6:
			pointToSend.Position.Actuators.Actuator6 = Angle;
		break;
	}
}






// Function to go to the hardcoded starting position
void MyGoToStart( ){

	SetJointGlobPoint( 1, MyStartPosition[0] );
	SetJointGlobPoint( 2, MyStartPosition[1] );
	SetJointGlobPoint( 3, MyStartPosition[2] );
	SetJointGlobPoint( 4, MyStartPosition[3] );
	SetJointGlobPoint( 5, MyStartPosition[4] );
	SetJointGlobPoint( 6, MyStartPosition[5] );

	ApplyPoint( 30 );

}
