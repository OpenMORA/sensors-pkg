/* +---------------------------------------------------------------------------+
   |                 Open MORA (MObile Robot Arquitecture)                     |
   |                  University of Almeria ARM-eCar module                    |
   |                                                                           |
   |   Copyright (C) 2014  University of Almeria                               |
   +---------------------------------------------------------------------------+ */

#ifndef CPhidgetsEncoders_H
#define CPhidgetsEncoders_H

#include <COpenMORAMOOSApp.h>
#include <mrpt/utils.h>
#include <phidget21.h>

class CPhidgetsEncoders : public COpenMORAApp
{
public:
    CPhidgetsEncoders();
    virtual ~CPhidgetsEncoders();

protected:
	/** called at startup */
	virtual bool OnStartUp();
	/** called when new mail arrives */
	virtual bool OnNewMail(MOOSMSG_LIST & NewMail);
	/** called when work is to be done */
	virtual bool Iterate();
	/** called when app connects to DB */
	virtual bool OnConnectToServer();

	bool OnCommandMsg( CMOOSMsg Msg );

	bool DoRegistrations();


	// Local class members:
	CPhidgetEncoderHandle  m_encoderHandle;
	int m_encoderSerial;
	std::string   m_encoderPublishPrefix;
	unsigned int  m_speed_filter_length; //!< (Default:10) Number of samples for filtering speed
	unsigned int  m_speed_filter_max_idle_loops; 

public:
	struct TStatePerChannel
	{
		double tickPos;
		double instantaneousSpeed;
		std::vector<double> speeds_buffer;
		bool speed_buffer_updated;
		unsigned int  loops_without_update_speed_buffer;

		TStatePerChannel() : tickPos(.0),instantaneousSpeed(.0),speed_buffer_updated(false),loops_without_update_speed_buffer(0) {}
	};

	mrpt::synch::CCriticalSection  m_encoder_states_cs;
	std::vector<TStatePerChannel>  m_encoder_states; 

};
#endif
