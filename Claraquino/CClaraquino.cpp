/* +---------------------------------------------------------------------------+
   |                 Open MORA (MObile Robot Arquitecture)                     |
   |                  University of Almeria ARM-eCar module                    |
   |                                                                           |
   |   Copyright (C) 2014  University of Almeria                               |
   +---------------------------------------------------------------------------+ */

/**  @moos_module MOOS interface to a Claraquino USB board.
  *
  *  More info on this board in ...
  */

#include "CClaraquino.h"
#include <mrpt/system/threads.h> // for sleep()

using namespace std;
using namespace mrpt;
using namespace mrpt::utils;


CClaraquino::CClaraquino() :
	m_serial_port_name("COM9"),
	m_serial_port_baudrate(115200)
{
}

CClaraquino::~CClaraquino()
{
}

bool CClaraquino::OnStartUp()
{
	// If want a different mode than standard:
	// - REGULAR_ITERATE_AND_MAIL
	// - COMMS_DRIVEN_ITERATE_AND_MAIL
	// - REGULAR_ITERATE_AND_COMMS_DRIVEN_MAIL
	SetIterateMode(COMMS_DRIVEN_ITERATE_AND_MAIL);

	// Read parameters (if any) from the mission configuration file.

	//! @moos_param SERIAL_PORT (Default="COM11") The serial port to connect to.
	m_MissionReader.GetConfigurationParam("SERIAL_PORT",m_serial_port_name);

	//! @moos_param SERIAL_PORT_BAUDRATE (Default=115200) The serial port baud rate
	m_MissionReader.GetConfigurationParam("SERIAL_PORT_BAUDRATE",m_serial_port_baudrate);

	//! @moos_var	CLARAQUINO_T1_FREQ  Generates a 50% duty square signal in T1 pin (PD4) with the given frequency in Hz (set to 0 to disable).
	//! @moos_subscribe	CLARAQUINO_T1_FREQ
	AddMOOSVariable_OpenMORA("CLARAQUINO_T1_FREQ", 0.01);

	// Try to connect...
	this->AttemptConnection();


	// There is also a MRPT-like object (this->m_ini) that is a wrapper
	//  to read from the module config block of the current MOOS mission file.
	// m_ini.read_int(...);
	return DoRegistrations();
}

bool CClaraquino::OnCommandMsg( CMOOSMsg Msg )
{
	if(Msg.IsSkewed(MOOSTime())) return true;
	if(!Msg.IsString()) return MOOSFail("This module only accepts string command messages\n");
	const std::string sCmd = Msg.GetString();
	//MOOSTrace("COMMAND RECEIVED: %s\n",sCmd.c_str());
	// Process the command "sCmd".

	return true;
}

bool CClaraquino::Iterate()
{
	// Main module loop code.

	//!  @moos_var CLARAQUINO_LAST_CMD_STATUS The last command sent out to the board. Can be used to monitor whether the module is up and responding.

	// Process updates of: CLARAQUINO_T1_FREQ
	{
		CMOOSVariable *pVar = GetMOOSVar_OpenMORA("CLARAQUINO_T1_FREQ");
		if(pVar && pVar->IsFresh())
		{
			pVar->SetFresh(false);

			const double freq = pVar->GetDoubleVal();
			MOOSTrace("Setting T1_FREQ=%f\n", freq);

			try
			{
				std::string s = mrpt::format("T1_FREQ %f\n",freq);
				m_serial.WriteBuffer(&s[0],s.size());
				m_Comms.Notify("CLARAQUINO_LAST_CMD_STATUS",s);
			}
			catch (std::exception &e)
			{
				MOOSTrace("*** Error sending command:\n%s",e.what());
			}
		}
	}



	return true;
}

bool CClaraquino::OnConnectToServer()
{
	DoRegistrations();
	return true;
}


bool CClaraquino::DoRegistrations()
{
	RegisterMOOSVariables();
	RegisterMOOSVariables_OpenMORA();
	return true;
}


bool CClaraquino::OnNewMail(MOOSMSG_LIST &NewMail)
{
	UpdateMOOSVariables(NewMail);
	UpdateMOOSVariables_OpenMORA(NewMail);
	return true;
}


 //!< Returns true if the serial comms are working
bool CClaraquino::IsConnected() const
{
	return m_serial.isOpen();
}

bool CClaraquino::AttemptConnection()
{
	if (m_serial.isOpen()) return true; // Already open.

	try {
		m_serial.open(m_serial_port_name);

		// Set basic params:
		m_serial.setConfig(m_serial_port_baudrate);
		m_serial.setTimeouts(100,0,10,0,50);

		return true;
	}
	catch (exception &e)
	{
		cerr << "[CClaraquino::AttemptConnection] COMMS error: " << e.what() << endl;
		return false;
	}
}

