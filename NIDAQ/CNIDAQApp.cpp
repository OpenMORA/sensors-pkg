/* +---------------------------------------------------------------------------+
   |                 Open MORA (MObile Robot Arquitecture)                     |
   |                  University of Almeria ARM-eCar module                    |
   |                                                                           |
   |   Copyright (C) 2014  University of Almeria                               |
   +---------------------------------------------------------------------------+ */

/**  @moos_module NationalInstruments DAQ module. 
  *  The config file must be include a configuration block as expected by mrpt::hwdrivers::CIMUXSens_MT4 (similar to rawlog-grabber)
  */

#include "CNIDAQApp.h"

#include <sstream>
#include <iomanip>
#include <iostream>

using namespace std;

using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::utils;


CNIDAQApp::CNIDAQApp() 
{
}

CNIDAQApp::~CNIDAQApp()
{
}

bool CNIDAQApp::OnStartUp()
{
	try 
	{
		m_daq.loadConfig(m_ini, "");
		m_daq.initialize();	
	}
	catch (std::exception &e)
	{
		MOOSTrace("*ERROR*:\n%s\n", e.what());
		return false;
	}

	return DoRegistrations();
}

bool CNIDAQApp::OnCommandMsg( CMOOSMsg Msg )
{
	if(Msg.IsSkewed(MOOSTime())) return true;
	if(!Msg.IsString()) return MOOSFail("This module only accepts string command messages\n");
	const std::string sCmd = Msg.GetString();
	//MOOSTrace("COMMAND RECEIVED: %s\n",sCmd.c_str());
	// Process the command "sCmd".
	return true;
}

bool CNIDAQApp::Iterate()
{
	try
	{
		m_daq.doProcess();
	}
	catch(std::exception &e)
	{
		MOOSTrace("*ERROR*:\n%s\n", e.what());
		return false;
	}
		
	mrpt::hwdrivers::CGenericSensor::TListObservations lstObjs;
	m_daq.getObservations(lstObjs);

	if (lstObjs.empty())
		return true;

	// Send out all observations to the DB:
	for (mrpt::hwdrivers::CGenericSensor::TListObservations::iterator it=lstObjs.begin();it!=lstObjs.end();++it)
	{
		CObservationRawDAQPtr obs = CObservationRawDAQPtr(it->second);
		mrpt::vector_byte bObs;
		mrpt::utils::ObjectToOctetVector(obs.pointer(), bObs);

		//!  @moos_var DAQ_OBS  NI DAQ readings as binary serializations of "CObservationRawDAQ" passed to std::vector<uint8_t> through ObjectToOctetVector()
		m_Comms.Notify("DAQ_OBS", bObs );
	}

	try
	{

	CSerializablePtr obj= lstObjs.rbegin()->second;
	ASSERT_(IS_DERIVED(obj,CObservationRawDAQ));

	CObservationRawDAQPtr obs = CObservationRawDAQPtr(obj);
	
	// Also publish numeric values for the convenience of users:
	const size_t nAIN = obs->AIN_double.size();
	const size_t nCh  = obs->AIN_channel_count;
	if (nAIN>0  // There are readings
		&& nCh>0   // There are >=1 channels read
		&& (nAIN % nCh)==0  // The number of samples MUST BE an exact multiple of channel count
		)
	{
		const size_t nSamplesPerCh = nAIN/nCh;

		// Get the last values of each channel:
		std::map<unsigned int,double> ch_vals;
		for (unsigned int i=0;i<nCh;i++)
		{
			size_t idx;
			if (obs->AIN_interleaved)  
				// A0 A1 A2   A0 A1 A2 ...
				idx = (nSamplesPerCh-1)*nCh + i;
			else 
				// A0 A0... A1 A1 ... A2 A2 ...
				idx = nSamplesPerCh*i + (nSamplesPerCh-1);
			ch_vals[i]=obs->AIN_double[idx];
		}

		// Publish values:
		for (std::map<unsigned int,double>::const_iterator it=ch_vals.begin();it!=ch_vals.end();++it)
		{
			//!  @moos_var DAQ_AIN{i}  NI DAQ Analog Input (AIN) voltage readings for the i'th channel (0-based, in the order specified in the .moos file)
			//!  @moos_publish DAQ_AIN{i}
			m_Comms.Notify(mrpt::format("DAQ_AIN%u",it->first), it->second );
		}

	}

	}
	catch(std::exception &e) {
		std::cerr << "[NIDAQ] Exception: " << e.what() << std::endl;
	}
	catch(...) {
		std::cerr << "[NIDAQ] Untyped exception!\n";
	}

	return true;
}

bool CNIDAQApp::OnConnectToServer()
{
	DoRegistrations();
	return true;
}


bool CNIDAQApp::DoRegistrations()
{
	RegisterMOOSVariables();
	RegisterMOOSVariables_OpenMORA();
	return true;
}


bool CNIDAQApp::OnNewMail(MOOSMSG_LIST &NewMail)
{
	UpdateMOOSVariables(NewMail);
	UpdateMOOSVariables_OpenMORA(NewMail);
	return true;
}
