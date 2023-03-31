//******************************************************************
// 
//  Generated by RoboCompDSL
//  
//  File name: Ultrasound.ice
//  Source: Ultrasound.idsl
//
//******************************************************************
#ifndef ROBOCOMPULTRASOUND_ICE
#define ROBOCOMPULTRASOUND_ICE
module RoboCompUltrasound
{
	exception HardwareFailedException{ string what; };
	exception UnknownSensorException{ string what; };
	struct BusParams
	{
		int numSensors;
		int baudRate;
		int basicPeriod;
	};
	struct SensorParams
	{
		string device;
		int busId;
		string name;
	};
	struct SonarPose
	{
		double x;
		double y;
		float angle;
	};
	sequence <SensorParams> SensorParamsList;
	sequence <int> SensorsState;
	sequence <SonarPose> SonarPoseList;
	interface Ultrasound
	{
		SensorsState getAllSensorDistances () throws UnknownSensorException,HardwareFailedException;
		SensorParamsList getAllSensorParams ();
		SonarPoseList getAllSonarPose ();
		BusParams getBusParams ();
		int getSensorDistance (string sensor) throws UnknownSensorException,HardwareFailedException;
		SensorParams getSensorParams (string sensor);
		int getSonarsNumber ();
	};
};

#endif
