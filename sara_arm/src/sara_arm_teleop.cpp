
#include "Lib/sara_arm.h"









void animation(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());







  	string Animation = "CrazySara";



	for(int i = 0; i < devicesCount; i++)
	{

		cout << "Found a robot on the USB bus (" << devices[i].SerialNumber << ")" << endl;

		//Setting the current device as the active device.
		MySetActiveDevice(devices[i]);

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
			for( float i=0; i<(100000); i++ ){

				Speed = sin(i/Period*100.0f)*40.0f;


				MyGetSensorsInfo( Response );

				cout << "Courant = " << Response.Current << endl;

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

		} else if ( Animation == "CrazySara" ) {


			SetJointGlobPoint( 1, 170 );
			SetJointGlobPoint( 2, 0 );
			SetJointGlobPoint( 3, 16 );
			SetJointGlobPoint( 4, 280 );
			SetJointGlobPoint( 5, 345 );
			ApplyPoint( 20 );
			WaitForReach( );


			SetJointGlobPoint( 1, 161 );
			SetJointGlobPoint( 2, -80 );
			SetJointGlobPoint( 3, 16 );
			SetJointGlobPoint( 4, 330 );
			SetJointGlobPoint( 5, 345 );
			ApplyPoint( 30 );
			WaitForReach( );


			Stop();
			float Speed;
			float Period;
			Period = 5000;
			for( float i=0; i<(100000); i++ ){

				Speed = sin(i/Period*300.0f)*80.0f;
				SetJointGlobPoint( 3, Speed );

				Speed = sin(i/Period*250.0f)*80.0f;
				SetJointGlobPoint( 5, Speed );






				ApplyVelocities( );
				usleep(Period);
			}



			MyGoToStart( );
			WaitForReach(  );

			//sleep(1);

		}
	}
}
