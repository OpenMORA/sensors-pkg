/* +---------------------------------------------------------------------------+
   |                 Open MORA (MObile Robot Arquitecture)                     |
   |                  University of Almeria ARM-eCar module                    |
   |                                                                           |
   |   Copyright (C) 2014  University of Almeria                               |
   +---------------------------------------------------------------------------+ */

#ifndef CIMU_XSensApp_H
#define CIMU_XSensApp_H

#include <COpenMORAMOOSApp.h>
#include <mrpt/hwdrivers/CIMUXSens_MT4.h>


class CIMU_XSensApp : public COpenMORAApp
{
public:
    CIMU_XSensApp();
    virtual ~CIMU_XSensApp();

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

	// DATA. Your local variables here...

	mrpt::hwdrivers::CIMUXSens_MT4  m_imu;


};
#endif
