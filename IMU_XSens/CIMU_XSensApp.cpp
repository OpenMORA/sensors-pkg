/* +---------------------------------------------------------------------------+
   |                 Open MORA (MObile Robot Arquitecture)                     |
   |                  University of Almeria ARM-eCar module                    |
   |                                                                           |
   |   Copyright (C) 2014  University of Almeria                               |
   +---------------------------------------------------------------------------+ */

/**  @moos_module Interface to XSens IMU devices (MT4 generation protocol).
  *  The config file must be include a configuration block as expected by mrpt::hwdrivers::CIMUXSens_MT4 (similar to rawlog-grabber)
  */

#include "CIMU_XSensApp.h"
#include <mrpt/slam/CObservationIMU.h>

#include <sstream>
#include <iomanip>
#include <iostream>

using namespace std;

using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::utils;
using namespace mrpt::hwdrivers;
using namespace mrpt::slam;


CIMU_XSensApp::CIMU_XSensApp() 
{
}

CIMU_XSensApp::~CIMU_XSensApp()
{
}

bool CIMU_XSensApp::OnStartUp()
{
	try 
	{
		m_imu.loadConfig(m_ini, "");
		m_imu.initialize();	
	}
	catch (std::exception &e)
	{
		MOOSTrace("*ERROR*:\n%s\n", e.what());
		return false;
	}

	// There is also a MRPT-like object (this->m_ini) that is a wrapper
	//  to read from the module config block of the current MOOS mission file.
	// m_ini.read_int(...);
	return DoRegistrations();
}

bool CIMU_XSensApp::OnCommandMsg( CMOOSMsg Msg )
{
	if(Msg.IsSkewed(MOOSTime())) return true;
	if(!Msg.IsString()) return MOOSFail("This module only accepts string command messages\n");
	const std::string sCmd = Msg.GetString();
	//MOOSTrace("COMMAND RECEIVED: %s\n",sCmd.c_str());
	// Process the command "sCmd".

	return true;
}

// Main module loop code. 
bool CIMU_XSensApp::Iterate()
{
	try
	{
		m_imu.doProcess();
	}
	catch(std::exception &e)
	{
		MOOSTrace("*ERROR*:\n%s\n", e.what());
		return false;
	}
		
	mrpt::hwdrivers::CGenericSensor::TListObservations lstObjs;
	m_imu.getObservations(lstObjs);

	if (lstObjs.empty())
		return true;

	// Send just the most recent observation to the DB:
	CSerializablePtr obj= lstObjs.rbegin()->second;
	ASSERT_(IS_DERIVED(obj,CObservationIMU));

	CObservationIMUPtr obs = CObservationIMUPtr(obj);
	mrpt::vector_byte bObs;
	mrpt::utils::ObjectToOctetVector(obs.pointer(), bObs);

	//!  @moos_var     IMU_OBS  IMU readings as binary serializations of "CObservationIMU" passed to std::vector<uint8_t> through ObjectToOctetVector()
	m_Comms.Notify("IMU_OBS", bObs );
	

	// Also publish numeric values of the IMU for the convenience of users:

	//!  @moos_var     IMU_ACC_X  Acceleration in X (m/s^2)
	if (obs->dataIsPresent[IMU_X_ACC]) m_Comms.Notify("IMU_ACC_X", obs->rawMeasurements[IMU_X_ACC] );
	//!  @moos_var     IMU_ACC_Y  Acceleration in Y (m/s^2)
	if (obs->dataIsPresent[IMU_Y_ACC]) m_Comms.Notify("IMU_ACC_Y", obs->rawMeasurements[IMU_Y_ACC] );
	//!  @moos_var     IMU_ACC_Z  Acceleration in Z (m/s^2)
	if (obs->dataIsPresent[IMU_Z_ACC]) m_Comms.Notify("IMU_ACC_Z", obs->rawMeasurements[IMU_Z_ACC] );

	//!  @moos_var     IMU_YAW_VEL Yaw (Z) angular velocity (rad/sec)
	if (obs->dataIsPresent[IMU_YAW_VEL]) m_Comms.Notify("IMU_YAW_VEL", obs->rawMeasurements[IMU_YAW_VEL] );
	//!  @moos_var     IMU_PITCH_VEL Pitch (Y) angular velocity (rad/sec)
	if (obs->dataIsPresent[IMU_PITCH_VEL]) m_Comms.Notify("IMU_PITCH_VEL", obs->rawMeasurements[IMU_PITCH_VEL] );
	//!  @moos_var     IMU_ROLL_VEL Roll (X) angular velocity (rad/sec)
	if (obs->dataIsPresent[IMU_ROLL_VEL]) m_Comms.Notify("IMU_ROLL_VEL", obs->rawMeasurements[IMU_ROLL_VEL] );

	//!  @moos_var     IMU_YAW Yaw (Z) absolute value (rad)
	if (obs->dataIsPresent[IMU_YAW]) m_Comms.Notify("IMU_YAW", obs->rawMeasurements[IMU_YAW] );
	//!  @moos_var     IMU_PITCH Pitch (Y) absolute value (rad)
	if (obs->dataIsPresent[IMU_PITCH]) m_Comms.Notify("IMU_PITCH", obs->rawMeasurements[IMU_PITCH] );
	//!  @moos_var     IMU_ROLL Roll (X) absolute value (rad)
	if (obs->dataIsPresent[IMU_ROLL]) m_Comms.Notify("IMU_ROLL", obs->rawMeasurements[IMU_ROLL] );

	//!  @moos_var     IMU_MAG_X  X magnetic field value (gauss)
	if (obs->dataIsPresent[IMU_MAG_X]) m_Comms.Notify("IMU_MAG_X", obs->rawMeasurements[IMU_MAG_X] );
	//!  @moos_var     IMU_MAG_Y  Y magnetic field value (gauss)
	if (obs->dataIsPresent[IMU_MAG_Y]) m_Comms.Notify("IMU_MAG_Y", obs->rawMeasurements[IMU_MAG_Y] );
	//!  @moos_var     IMU_MAG_Z  Z magnetic field value (gauss)
	if (obs->dataIsPresent[IMU_MAG_Z]) m_Comms.Notify("IMU_MAG_Z", obs->rawMeasurements[IMU_MAG_Z] );

	return true;
}

bool CIMU_XSensApp::OnConnectToServer()
{
	DoRegistrations();
	return true;
}


bool CIMU_XSensApp::DoRegistrations()
{
	RegisterMOOSVariables();
	RegisterMOOSVariables_OpenMORA();
	return true;
}


bool CIMU_XSensApp::OnNewMail(MOOSMSG_LIST &NewMail)
{
	UpdateMOOSVariables(NewMail);
	UpdateMOOSVariables_OpenMORA(NewMail);
	return true;
}
