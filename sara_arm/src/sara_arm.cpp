/*
 SARA arm
 input: std_msgs::Int8MultiArray&
 input: std_msgs::String&
 output: mouvements du bras

*/


#include "Lib/sara_arm.h"

// Déclaration des variables utiles
int result;  // Tampon de réception des résultats de certaines fonctions
bool Can_teleop;  // Flag qui détermine si le node teleop est actif ou non
Sequence Anim;
AngularPosition AngularCommand;
TrajectoryPoint pointToSend;
SensorsInfo Response;




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
	if ( Can_teleop ){
		SetJointGlobPoint( 1, msg.data[0] );
	 	SetJointGlobPoint( 2, msg.data[1] );
	 	SetJointGlobPoint( 3, msg.data[2] );
	 	SetJointGlobPoint( 4, msg.data[3] );
	 	SetJointGlobPoint( 5, msg.data[4] );
	 	SetJointGlobPoint( 6, msg.data[5] );
		ApplyVelocities();
	}

}


void animation( const std_msgs::String& msg )
{
	if ( msg.data == "clear_sequence" ){
		Anim.Lenght = 0;
		Stop();

	} else if ( msg.data == "jouer_sequence" ){
		Execute_sequence( Anim );

	} else if ( msg.data == "ajoute_point" ){
		MyGetAngularCommand( AngularCommand );
		Anim.Points[Anim.Lenght].Joints[0] = AngularCommand.Actuators.Actuator1;
		Anim.Points[Anim.Lenght].Joints[1] = AngularCommand.Actuators.Actuator2;
		Anim.Points[Anim.Lenght].Joints[2] = AngularCommand.Actuators.Actuator3;
		Anim.Points[Anim.Lenght].Joints[3] = AngularCommand.Actuators.Actuator4;
		Anim.Points[Anim.Lenght].Joints[4] = AngularCommand.Actuators.Actuator5;
		Anim.Points[Anim.Lenght].Joints[5] = AngularCommand.Actuators.Actuator6;
		Anim.Points[Anim.Lenght].Speed = 100;

	}





}















int main(int argc, char **argv)
{

  	//string Animation = "CrazySara";



	Anim.Lenght = 0;

  	pointToSend.InitStruct();

	bool Succes = false;

	while( !Succes ) {

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
		MySetActuatorMaxVelocity = (int (*)(float &)) dlsym(commandLayer_handle,"SetActuatorMaxVelocity");
	  	MyGetSensorsInfo = (int (*)(SensorsInfo &)) dlsym(commandLayer_handle,"GetSensorsInfo");

		if ((MyInitAPI == NULL) || (MyCloseAPI == NULL) || (MySendBasicTrajectory == NULL) ||
   		(MySendAdvanceTrajectory == NULL) || (MyMoveHome == NULL) || (MyInitFingers == NULL) ){
	  		cout << "* * *  E R R O R   D U R I N G   I N I T I A L I Z A T I O N  * * *" << endl;
			sleep(1);

	  	} else {
			Succes = true;

		}
	}



	cout << "I N I T I A L I Z A T I O N   C O M P L E T E D" << endl << endl;


	Succes = false;
	int nb_attemps = 1;
	while( !Succes ) {

		result = (*MyInitAPI)();
		devicesCount = MyGetDevices(devices, result);

		if ( result > 10 ){
	  		cout << "* * *                  N O   A R M   F O U N D               * * *" << endl;
	  		cout << "* * *                   A T T E M P   N B   " << nb_attemps << "              * * *" << endl;

	  		cout << "* * *              S E A R C H I N G   A G A I N . . .       * * *" << endl;
			nb_attemps ++;
			sleep(1);

	  	} else {
			Succes = true;

		}
	}

	cout << "Initialization's result :" << result << endl;
	cout << "Number of attemps :" << nb_attemps << endl;


	// ROS
	// Initialisation
	ros::init(argc, argv, "sara_arm");

	// Obtention du nodehandle
	ros::NodeHandle n;

	ros::Subscriber sub;

	// inscription du node "teleop" au topic "teleop_arm"
	sub = n.subscribe("teleop_arm", 10, teleop );

	// inscription du node "animation" au topic "animation_arm"
	sub = n.subscribe("animation_arm", 10, animation );


	ros::spin();




	// dlclose(commandLayer_handle);

	//	cout << endl << "C L O S I N G   A P I" << endl;
	//	result = (*MyCloseAPI)();

	return 0;
}

























void WaitForReach(  ){

	int MyTolerence = 2;  // Tolérence en degré
	bool ok = false;


	cout << "Waiting for reach" << endl;
	while( !ok ){

		usleep(50000);

		///cout << "Still waiting..." << endl;
		MyGetAngularCommand(AngularCommand);
		ok = true;
		ok = ok && abs(AngularCommand.Actuators.Actuator1-pointToSend.Position.Actuators.Actuator1)<MyTolerence;
		ok = ok && abs(AngularCommand.Actuators.Actuator2-pointToSend.Position.Actuators.Actuator2)<MyTolerence;
		ok = ok && abs(AngularCommand.Actuators.Actuator3-pointToSend.Position.Actuators.Actuator3)<MyTolerence;
		ok = ok && abs(AngularCommand.Actuators.Actuator4-pointToSend.Position.Actuators.Actuator4)<MyTolerence;
		ok = ok && abs(AngularCommand.Actuators.Actuator5-pointToSend.Position.Actuators.Actuator5)<MyTolerence;
		//ok = ok && abs(AngularCommand.Actuators.Actuator6-pointToSend.Position.Actuators.Actuator6)<MyTolerence;
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
	pointToSend.Limitations.speedParameter1 = 100;
	pointToSend.Limitations.speedParameter2 = 100;
	//pointToSend.Limitations.speedParameter3 = 100;
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
	MyGetAngularCommand(AngularCommand);
	switch (Joint) {
		case 1: pointToSend.Position.Actuators.Actuator1 = AngularCommand.Actuators.Actuator1+Angle;
		break;
		case 2: pointToSend.Position.Actuators.Actuator2 = AngularCommand.Actuators.Actuator2+Angle;
		break;
		case 3: pointToSend.Position.Actuators.Actuator3 = AngularCommand.Actuators.Actuator3+Angle;
		break;
		case 4: pointToSend.Position.Actuators.Actuator4 = AngularCommand.Actuators.Actuator4+Angle;
		break;
		case 5: pointToSend.Position.Actuators.Actuator5 = AngularCommand.Actuators.Actuator5+Angle;
		break;
		case 6: pointToSend.Position.Actuators.Actuator6 = AngularCommand.Actuators.Actuator6+Angle;
		break;
	}
}




void SetJointGlobPoint( int Joint, int Angle ){
	MyGetAngularCommand(AngularCommand);
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



void Execute_sequence( Sequence Anim ){

/*	int a, b;
	while (infile >> a >> b)
	{
	    // process pair (a,b)
	}*/

	Can_teleop = false;
	int i;
	for ( i = 0; i<Anim.Lenght; i++ ){

		SetJointGlobPoint( 1, Anim.Points[i].Joints[0] );
		SetJointGlobPoint( 2, Anim.Points[i].Joints[1] );
		SetJointGlobPoint( 3, Anim.Points[i].Joints[2] );
		SetJointGlobPoint( 4, Anim.Points[i].Joints[3] );
		SetJointGlobPoint( 5, Anim.Points[i].Joints[4] );
		SetJointGlobPoint( 6, Anim.Points[i].Joints[5] );
		ApplyPoint( Anim.Points[i].Speed );
		WaitForReach( );

	}

	Can_teleop = true;
}
