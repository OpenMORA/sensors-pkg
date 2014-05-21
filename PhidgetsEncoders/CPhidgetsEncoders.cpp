/* +---------------------------------------------------------------------------+
   |                 Open MORA (MObile Robot Arquitecture)                     |
   |                  University of Almeria ARM-eCar module                    |
   |                                                                           |
   |   Copyright (C) 2014  University of Almeria                               |
   +---------------------------------------------------------------------------+ */

/**  @moos_module Interface to Phidgets high-speed encoder USB board  */

#include "CPhidgetsEncoders.h"

#include <sstream>
#include <iomanip>
#include <iostream>
#include <numeric>

using namespace std;

using namespace mrpt;
using namespace mrpt::utils;


int CCONV AttachHandler(CPhidgetHandle ENC, void *userptr)
{
	CPhidgetsEncoders *obj = (CPhidgetsEncoders*)userptr;

	int serialNo;
	CPhidget_DeviceID deviceID;
	int inputcount;

	CPhidget_getSerialNumber(ENC, &serialNo);

	//Retrieve the device ID and number of encoders so that we can set the enables if needed
	CPhidget_getDeviceID(ENC, &deviceID);
	CPhidgetEncoder_getEncoderCount((CPhidgetEncoderHandle)ENC, &inputcount);
	printf("[PhidgetsEncoders] Encoder %10d attached! \n", serialNo);

	// Resize:
	obj->m_encoder_states_cs.enter();
	obj->m_encoder_states.resize(inputcount);
	obj->m_encoder_states_cs.leave();

	//the 1047 requires enabling of the encoder inputs, so enable them if this is a 1047
	if (deviceID == PHIDID_ENCODER_HS_4ENCODER_4INPUT)
	{
			for (int i = 0 ; i < inputcount ; i++)
					CPhidgetEncoder_setEnabled((CPhidgetEncoderHandle)ENC, i, 1);
	}
	return 0;
}


int CCONV DetachHandler(CPhidgetHandle ENC, void *userptr)
{
	CPhidgetsEncoders *obj = (CPhidgetsEncoders*)userptr;

	int serialNo;
	CPhidget_getSerialNumber(ENC, &serialNo);
	printf("[PhidgetsEncoders] Encoder %10d detached! \n", serialNo);

	return 0;
}

int CCONV ErrorHandler(CPhidgetHandle ENC, void *userptr, int ErrorCode, const char *Description)
{
	CPhidgetsEncoders *obj = (CPhidgetsEncoders*)userptr;

	printf("[PhidgetsEncoders] Error handled. %d - %s \n", ErrorCode, Description);

	return 0;
}

int CCONV InputChangeHandler(CPhidgetEncoderHandle ENC, void *usrptr, int Index, int State)
{
	CPhidgetsEncoders *obj = (CPhidgetsEncoders*)usrptr;
	//printf("[PhidgetsEncoders] Input #%i - State: %i \n", Index, State);
	return 0;
}

int CCONV PositionChangeHandler(CPhidgetEncoderHandle ENC, void *usrptr, int Index, int Time, int RelativePosition)
{
	CPhidgetsEncoders *obj = (CPhidgetsEncoders*)usrptr;

	int Position;
	CPhidgetEncoder_getPosition(ENC, Index, &Position);

	obj->m_encoder_states_cs.enter();

	if (Index<obj->m_encoder_states.size())
	{
		CPhidgetsEncoders::TStatePerChannel &spc = obj->m_encoder_states[Index];
		spc.tickPos = Position;
		spc.instantaneousSpeed = RelativePosition / (Time*1e-6);
		spc.speeds_buffer.push_back(spc.instantaneousSpeed);
		spc.speed_buffer_updated = true;
		spc.loops_without_update_speed_buffer = 0;
	}

	obj->m_encoder_states_cs.leave();

	//printf("Encoder #%i - Pos: %5d -- RelChange %2d -- Elapsed Time: %5d spd=%f \n", Index, Position, RelativePosition, Time,obj->m_encoder_states[Index].instantaneousSpeed);

	return 0;
}


CPhidgetsEncoders::CPhidgetsEncoders() :
	m_encoderHandle(NULL),
	m_encoderSerial(-1),
	m_encoderPublishPrefix("ENC1"),
	m_speed_filter_length(10),
	m_speed_filter_max_idle_loops(1)
{
	CPhidgetEncoder_create(&m_encoderHandle);

	//Set the handlers to be run when the device is plugged in or opened from software, unplugged or closed from software, or generates an error.
	CPhidget_set_OnAttach_Handler((CPhidgetHandle)m_encoderHandle, AttachHandler, this);
	CPhidget_set_OnDetach_Handler((CPhidgetHandle)m_encoderHandle, DetachHandler, this);
	CPhidget_set_OnError_Handler((CPhidgetHandle)m_encoderHandle, ErrorHandler, this);

	//Registers a callback that will run if an input changes.
	//Requires the handle for the Phidget, the function that will be called, and an arbitrary pointer that will be supplied to the callback function (may be NULL).
	CPhidgetEncoder_set_OnInputChange_Handler(m_encoderHandle, InputChangeHandler, this);

	//Registers a callback that will run if the encoder changes.
	//Requires the handle for the Encoder, the function that will be called, and an arbitrary pointer that will be supplied to the callback function (may be NULL).
	CPhidgetEncoder_set_OnPositionChange_Handler (m_encoderHandle, PositionChangeHandler, this);

	// Open: to be done when we know the serial number to be open!
}

CPhidgetsEncoders::~CPhidgetsEncoders()
{
	CPhidget_close((CPhidgetHandle)m_encoderHandle);
	CPhidget_delete((CPhidgetHandle)m_encoderHandle);
}

bool CPhidgetsEncoders::OnStartUp()
{
	// If want a different mode than standard:
	// - REGULAR_ITERATE_AND_MAIL
	// - COMMS_DRIVEN_ITERATE_AND_MAIL
	// - REGULAR_ITERATE_AND_COMMS_DRIVEN_MAIL
	//SetIterateMode(REGULAR_ITERATE_AND_MAIL);

	// Read parameters (if any) from the mission configuration file.

	//! @moos_param ENCODER_SERIAL (Default=-1) The serial number of the Phidgets encoder: the number in the "S/N" sticker (or "-1" to open the first board found).
	m_MissionReader.GetConfigurationParam("ENCODER_SERIAL",m_encoderSerial);

	//! @moos_param ENCODER_PUBLISH_PREFIX  (Default="ENC1") Encoder data from this board will be published as "<ENCODER_PUBLISH_PREFIX>_CH<i>_{COUNT,INSTANT_VEL}" with i=0,1,2,3 for each encoder channel.
	m_MissionReader.GetConfigurationParam("ENCODER_PUBLISH_PREFIX",m_encoderPublishPrefix);

	//! @moos_param SPEED_FILTER_SAMPLES_LEN  (Default=10) Number of samples for the sliding window average filter of speeds.
	m_MissionReader.GetConfigurationParam("SPEED_FILTER_SAMPLES_LEN",m_speed_filter_length);
	
	//! @moos_param SPEED_FILTER_IDLE_ITER_LOOPS_BEFORE_RESET  (Default=1) Number of "ITERATE" loops without any new encoder tick before resetting the filtered average velocities.
	m_MissionReader.GetConfigurationParam("SPEED_FILTER_IDLE_ITER_LOOPS_BEFORE_RESET",m_speed_filter_max_idle_loops);


	CPhidget_open((CPhidgetHandle)m_encoderHandle, m_encoderSerial);

	// There is also a MRPT-like object (this->m_ini) that is a wrapper
	//  to read from the module config block of the current MOOS mission file.
	// m_ini.read_int(...);
	return DoRegistrations();
}

bool CPhidgetsEncoders::OnCommandMsg( CMOOSMsg Msg )
{
	if(Msg.IsSkewed(MOOSTime())) return true;
	if(!Msg.IsString()) return MOOSFail("This module only accepts string command messages\n");
	const std::string sCmd = Msg.GetString();
	//MOOSTrace("COMMAND RECEIVED: %s\n",sCmd.c_str());
	// Process the command "sCmd".

	return true;
}

bool CPhidgetsEncoders::Iterate()
{
	// Main module loop code.

	//!  @moos_publish <ENCODER_PUBLISH_PREFIX>_CH<i>_COUNT for each channel (i=0,1,2,3), the current tick count (position) of the encoder.
	//!  @moos_var_aliases <ENCODER_PUBLISH_PREFIX>_CH<i>_COUNT ENC1_CH0_COUNT ENC1_CH1_COUNT ENC1_CH2_COUNT ENC1_CH3_COUNT

	//!  @moos_publish <ENCODER_PUBLISH_PREFIX>_CH<i>_INST_VEL for each channel (i=0,1,2,3), the instantaneous estimation of the velocity, in ticks/second.
	//!  @moos_var_aliases <ENCODER_PUBLISH_PREFIX>_CH<i>_INST_VEL ENC1_CH0_INST_VEL ENC1_CH1_INST_VEL ENC1_CH2_INST_VEL ENC1_CH3_INST_VEL 

	//!  @moos_publish <ENCODER_PUBLISH_PREFIX>_CH<i>_AVRG_VEL for each channel (i=0,1,2,3), the averaged velocity, in ticks/second, according to the given filter window length.
	//!  @moos_var_aliases <ENCODER_PUBLISH_PREFIX>_CH<i>_AVRG_VEL ENC1_CH0_AVRG_VEL ENC1_CH1_AVRG_VEL ENC1_CH2_AVRG_VEL ENC1_CH3_AVRG_VEL 

	// Publish:
	{
		mrpt::synch::CCriticalSectionLocker csl( &m_encoder_states_cs );

		for (unsigned int i=0;i<m_encoder_states.size();i++)
		{
			TStatePerChannel &spc = m_encoder_states[i];

			m_Comms.Notify( mrpt::format("%s_CH%u_COUNT",m_encoderPublishPrefix.c_str(),i), spc.tickPos );
			m_Comms.Notify( mrpt::format("%s_CH%u_INST_VEL",m_encoderPublishPrefix.c_str(),i), spc.instantaneousSpeed );

			if (m_speed_filter_length>0)
			{
				if (!spc.speed_buffer_updated)
				{
					if (++spc.loops_without_update_speed_buffer >= m_speed_filter_max_idle_loops)
					{
						double avrg = 0.0;
						m_Comms.Notify( mrpt::format("%s_CH%u_AVRG_VEL",m_encoderPublishPrefix.c_str(),i), avrg );
					}
				}
				else
				{
					spc.loops_without_update_speed_buffer = 0;

					if (spc.speeds_buffer.size()>=m_speed_filter_length)
					{
						const double avrg = std::accumulate(spc.speeds_buffer.begin(),spc.speeds_buffer.end(),0.0) / spc.speeds_buffer.size();
						spc.speeds_buffer.clear();
						m_Comms.Notify( mrpt::format("%s_CH%u_AVRG_VEL",m_encoderPublishPrefix.c_str(),i), avrg );
					}
				}
			}
		}

	} // end critical section


	return true;
}

bool CPhidgetsEncoders::OnConnectToServer()
{
	DoRegistrations();
	return true;
}


bool CPhidgetsEncoders::DoRegistrations()
{
	RegisterMOOSVariables();
	return true;
}


bool CPhidgetsEncoders::OnNewMail(MOOSMSG_LIST &NewMail)
{
	UpdateMOOSVariables(NewMail);
	return true;
}
