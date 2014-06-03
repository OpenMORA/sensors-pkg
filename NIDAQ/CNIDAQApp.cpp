/* +---------------------------------------------------------------------------+
   |                 Open MORA (MObile Robot Arquitecture)                     |
   |                  University of Almeria ARM-eCar module                    |
   |                                                                           |
   |   Copyright (C) 2014  University of Almeria                               |
   +---------------------------------------------------------------------------+ */

/**  @moos_module NationalInstruments DAQ module. 
  *  The config file must be include a configuration block as expected by mrpt::hwdrivers::CNationalInstrumentsDAQ (similar to rawlog-grabber)
  *  Analog inputs are gathered synchronously and published as binary observations and also (decimated) as individual MOOS double variables. 
  *  Analog and digital asynchronous outputs are supported via individual MOOS double variables.
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
	// If want a different mode than standard:
	// - REGULAR_ITERATE_AND_MAIL
	// - COMMS_DRIVEN_ITERATE_AND_MAIL
	// - REGULAR_ITERATE_AND_COMMS_DRIVEN_MAIL
	SetIterateMode(REGULAR_ITERATE_AND_COMMS_DRIVEN_MAIL);

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

	m_varnames_ao.clear();
	m_varnames_do.clear();

	for (unsigned int i=0;i<m_daq.task_definitions.size();i++)
	{
		const mrpt::hwdrivers::CNationalInstrumentsDAQ::TaskDescription &t = m_daq.task_definitions[i];

		// Subscribe to analog outputs:
		//!  @moos_var DAQ_AO_TASK{i}  NI DAQ Analog outputs (AOUT) voltage for the i'th *task* of this kind (0-based, in the order specified in the .moos file)
		//!  @moos_subscribe DAQ_AO_TASK{i}
		if (t.has_ao) 
		{
			const unsigned int new_idx = m_varnames_ao.size();
			const string sVar = mrpt::format("DAQ_AO_TASK%u",new_idx);
			m_Comms.Register( sVar );
			m_varnames_ao[sVar] = i;
		}

		// Subscribe to digital outputs:
		//!  @moos_var DAQ_DO_TASK{i}  NI DAQ digital outputs (DOUT) for the i'th *task* of this kind (0-based, in the order specified in the .moos file). 0 => false, !=0 => true
		//!  @moos_subscribe DAQ_DO_TASK{i}
		if (t.has_do)
		{
			const unsigned int new_idx = m_varnames_do.size();
			const string sVar = mrpt::format("DAQ_DO_TASK%u",new_idx);
			m_Comms.Register( sVar );
			m_varnames_do[sVar] = i;
		}
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
	try
	{
		// Process asynchronous outputs:
		for (MOOSMSG_LIST::const_iterator it=NewMail.begin();it!=NewMail.end();++it)
		{
			const string sVar = it->GetName();

			// Analog outputs:
			for (map<string,size_t>::const_iterator itM=m_varnames_ao.begin();itM!=m_varnames_ao.end();++itM)
			{
				if (itM->first==sVar)
				{
					const size_t taskIdx = itM->second;
					const double val = it->GetDouble();

					m_daq.writeAnalogOutputTask(taskIdx,1, &val ,0.1, true);
					break;
				}
			}
			
			// Analog outputs:
			for (map<string,size_t>::const_iterator itM=m_varnames_do.begin();itM!=m_varnames_do.end();++itM)
			{
				if (itM->first==sVar)
				{
					const size_t taskIdx = itM->second;
					const bool val = it->GetDouble() != 0.0;

					m_daq.writeDigitalOutputTask(taskIdx,val, 0.1);
					break;
				}
			}

		} // end for each message

	}
	catch(std::exception &e) {
		std::cerr << "[NIDAQ] Exception: " << e.what() << std::endl;
	}
	catch(...) {
		std::cerr << "[NIDAQ] Untyped exception!\n";
	}


	UpdateMOOSVariables(NewMail);
	UpdateMOOSVariables_OpenMORA(NewMail);
	return true;
}
