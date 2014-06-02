/* +---------------------------------------------------------------------------+
   |                 Open MORA (MObile Robot Arquitecture)                     |
   |                  University of Almeria ARM-eCar module                    |
   |                                                                           |
   |   Copyright (C) 2014  University of Almeria                               |
   +---------------------------------------------------------------------------+ */

#ifndef CNIDAQApp_H
#define CNIDAQApp_H

#include <COpenMORAMOOSApp.h>
#include <mrpt/hwdrivers/CNationalInstrumentsDAQ.h>

class CNIDAQApp : public COpenMORAApp
{
public:
    CNIDAQApp();
    virtual ~CNIDAQApp();

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

	// DATA:

	mrpt::hwdrivers::CNationalInstrumentsDAQ  m_daq;

	std::map<std::string,size_t> m_varnames_ao; //!< MOOS var names of taks for async analog output, mapped to their task index
	std::map<std::string,size_t> m_varnames_do; //!< MOOS var names of taks for async digital output, mapped to their task index

};
#endif
