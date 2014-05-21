/* +---------------------------------------------------------------------------+
   |                 Open MORA (MObile Robot Arquitecture)                     |
   |                                                                           |
   |                        http://babel.isa.uma.es/mora/                      |
   |                                                                           |
   |   Copyright (C) 2010  University of Malaga                                |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics (MAPIR) Lab, University of Malaga (Spain).                  |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MORA project.                                   |
   |                                                                           |
   |     MORA is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MORA is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MORA.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */

#ifndef CGenericSensorApp_H
#define CGenericSensorApp_H

#include <MOOS/libMOOS/App/MOOSApp.h>
#include <mrpt/hwdrivers/CGenericSensor.h>
#include <mrpt/utils/CTimeLogger.h>

#include <COpenMORAMOOSApp.h>


class CGenericSensorApp : public COpenMORAApp
{
public:
    CGenericSensorApp();
    virtual ~CGenericSensorApp();

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

    /** performs the registration for mail */
    bool DoRegistrations();

	// DATA
	mrpt::hwdrivers::CGenericSensorPtr  m_sensor;

	// Kinect
	std::string m_decimate_type;

	std::vector<size_t> m_rows_to_include; //!< Useful for external applications when you want take in account only some rows
	size_t				m_only_one_of_each;

	size_t	m_regions_decimate_height;
	size_t	m_regions_decimate_width;

	mrpt::utils::CTimeLogger	m_timeLog;

};

#endif
