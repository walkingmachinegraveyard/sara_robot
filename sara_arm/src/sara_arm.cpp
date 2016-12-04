
#include "Lib/sara_arm.h"


KinovaDevice devices[MAX_KINOVA_DEVICE];
int devicesCount;




int MyStartPosition[] = { 	180,
							90,
							34,
							270,
							325,
							0	};







using namespace std;







void teleop( const std_msgs::Int8MultiArray& msg )
{
	int Vel[6];
	Vel[0] = msg.data[0];
	Vel[1] = msg.data[1];
	Vel[2] = msg.data[2];
	Vel[3] = msg.data[3];
	Vel[4] = msg.data[4];
	Vel[5] = msg.data[5];


	int i;
	for( i=0; i<200; i++ ){

		cout << "Go!" << i << endl;

		cout << "Joint1 :" << Vel[0] << endl;
		cout << "Joint2 :" << Vel[1] << endl;
		cout << "Joint3 :" << Vel[2] << endl;
		cout << "Joint4 :" << Vel[3] << endl;
		cout << "Joint5 :" << Vel[4] << endl;
		cout << "Joint6 :" << Vel[5] << endl;

	 	SetJointGlobPoint( 1, Vel[0] );
	 	SetJointGlobPoint( 2, Vel[1] );
	 	SetJointGlobPoint( 3, Vel[2] );
	 	SetJointGlobPoint( 4, Vel[3] );
	 	SetJointGlobPoint( 5, Vel[4] );
	 	SetJointGlobPoint( 6, Vel[5] );
		ApplyVelocities();
		//usleep(1000);
	}
	cout << "Fini!" << endl;
}









int main(int argc, char **argv)
{

  	string Animation = "CrazySara";


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

  	MyGetSensorsInfo = (int (*)(SensorsInfo &)) dlsym(commandLayer_handle,"GetSensorsInfo");




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
  		devicesCount = MyGetDevices(devices, result);

  		cout << "Initialization's result :" << result << endl;



		ros::init(argc, argv, "sara_arm");

		ros::NodeHandle n;




		ros::Subscriber sub = n.subscribe("teleop_arm", 10, teleop );

		ros::spin();


	//	cout << endl << "C L O S I N G   A P I" << endl;
	//	result = (*MyCloseAPI)();
	}






	// dlclose(commandLayer_handle);

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
		case 1: pointToSend.Position.Actuators.Actuator1 = currentCommand.Actuators.Actuator1+Angle;
		break;
		case 2: pointToSend.Position.Actuators.Actuator2 = currentCommand.Actuators.Actuator2+Angle;
		break;
		case 3: pointToSend.Position.Actuators.Actuator3 = currentCommand.Actuators.Actuator3+Angle;
		break;
		case 4: pointToSend.Position.Actuators.Actuator4 = currentCommand.Actuators.Actuator4+Angle;
		break;
		case 5: pointToSend.Position.Actuators.Actuator5 = currentCommand.Actuators.Actuator5+Angle;
		break;
		case 6: pointToSend.Position.Actuators.Actuator6 = currentCommand.Actuators.Actuator6+Angle;
		break;
	}
}




void SetJointGlobPoint( int Joint, int Angle ){
	MyGetAngularCommand(currentCommand);
	switch (Joint) {
		case 1: pointToSend.Position.Actuators.Actuator1 = Angle;
		break;
		case 2: pointToSend.Position.Actuators.Actuator2 = Angle;
		break;
		case 3: pointToSend.Position.Actuators.Actuator3 = Angle;
		break;
		case 4: pointToSend.Position.Actuators.Actuator4 = Angle;
		break;
		case 5: pointToSend.Position.Actuators.Actuator5 = Angle;
		break;
		case 6: pointToSend.Position.Actuators.Actuator6 = Angle;
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



void PrintInfo(){

	MyGetSensorsInfo( Response );
	cout << "Courant = " << Response.Current << endl;
	cout << "Tension = " << Response.Voltage << endl;

}
