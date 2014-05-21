/* +---------------------------------------------------------------------------+
   |                 Open MORA (MObile Robot Arquitecture)                     |
   |                  University of Almeria ARM-eCar module                    |
   |                                                                           |
   |   Copyright (C) 2014  University of Almeria                               |
   +---------------------------------------------------------------------------+ */

/**  @moos_module Stereo camera with two FlyCapture cameras plus a Claraquino board for sync trigger.
  *  <br>
  *  The camera configuration must be specified in the .moos config block, in the format expected
  *   by <a href="http://reference.mrpt.org/svn/classmrpt_1_1hwdrivers_1_1_c_camera_sensor.html">mrpt::hwdrivers::CCamera</a>.
  *
  */

#include "CStereoFlyCapApp.h"
#include <mrpt/utils/CTimeLogger.h>
#include <mrpt/utils/CConfigFile.h>
#include <mrpt/system/filesystem.h> // ASSERT_FILE_EXISTS_()

#include <sstream>
#include <iomanip>
#include <iostream>
#include <cstdio> // remove()

using namespace std;

using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::utils;
using namespace mrpt::hwdrivers;


void hook_rectify(const mrpt::slam::CObservationPtr &o, void* user_ptr)
{
	CStereoFlyCapApp *me = reinterpret_cast<CStereoFlyCapApp*>(user_ptr);

	using mrpt::slam::CObservationStereoImages;
	if (o->GetRuntimeClass()!=CLASS_ID(CObservationStereoImages))
		return; // WTF!? Just in case...

	mrpt::slam::CObservationStereoImagesPtr obs = mrpt::slam::CObservationStereoImagesPtr(o);

	if (!me->m_live_rectify)
		return;

	if (!me->m_rectifier.isSet())
	{
	    mrpt::synch::CCriticalSectionLocker csl(& me->m_rectifier_cs);

		me->m_rectifier.setAlpha(me->m_rectify_alpha);
		me->m_rectifier.enableBothCentersCoincide(me->m_rectify_align_centers);
		me->m_rectifier.enableResizeOutput(true, me->m_rectify_resize_new_size.x,me->m_rectify_resize_new_size.y);
		me->m_rectifier.setFromCamParams(me->m_stereo_params);
	}

	me->m_rectifier.rectify(*obs);
}


CStereoFlyCapApp::CStereoFlyCapApp() :
	m_grab_started(false),
	m_external_img_dirs("StereoFlyCap_Images"),
	m_imgs_queue_max_length(100),
	m_live_rectify(false),
	m_rectify_alpha(0.5)
{
}

CStereoFlyCapApp::~CStereoFlyCapApp()
{
	this->purgeStereoExternalImgs();
}

bool CStereoFlyCapApp::OnStartUp()
{
try
{

	// If want a different mode than standard:
	// - REGULAR_ITERATE_AND_MAIL
	// - COMMS_DRIVEN_ITERATE_AND_MAIL
	// - REGULAR_ITERATE_AND_COMMS_DRIVEN_MAIL
	//SetIterateMode(REGULAR_ITERATE_AND_MAIL);

	//! @moos_param IMAGES_DIR (Default:"StereoFlyCap_Images") Output directory for image (temporary) files in externally-stored mrpt::utils::CImage
	m_MissionReader.GetConfigurationParam("IMAGES_DIR",m_external_img_dirs);

	//! @moos_param IMAGES_DIR_APPEND_TIMEDATE (Default=true) If true, will append date & time to the images dir name
	const bool append_timemark = m_ini.read_bool("","IMAGES_DIR_APPEND_TIMEDATE",true);
	if (append_timemark)
	{
		mrpt::system::TTimeParts parts;
		mrpt::system::timestampToParts(mrpt::system::now(), parts, true);
		m_external_img_dirs += mrpt::format("%04u-%02u-%02u_%02uh%02um%02us",
			(unsigned int)parts.year,
			(unsigned int)parts.month,
			(unsigned int)parts.day,
			(unsigned int)parts.hour,
			(unsigned int)parts.minute,
			(unsigned int)parts.second );
	}

	MOOSTrace("Images dir: '%s'\n",m_external_img_dirs.c_str());
	CImage::IMAGES_PATH_BASE = m_external_img_dirs;
	mrpt::system::createDirectory(m_external_img_dirs);

	//!  @moos_var     MORA_IMAGES_DIR The directory where mrpt::utils::CImage store delay-load images (may be a shared memory directory)
	m_Comms.Notify("MORA_IMAGES_DIR", m_external_img_dirs);

	//! @moos_param IMAGES_SKIP_COMPRESS (default=false) If set to "true", image compression while serializing will be disabled (useful when saving to shared memory).
	bool skip_img_compress = false;
	m_MissionReader.GetConfigurationParam("IMAGES_SKIP_COMPRESS",skip_img_compress);
	if (skip_img_compress)
	{
		CImage::DISABLE_JPEG_COMPRESSION = true;
		CImage::DISABLE_ZIP_COMPRESSION  = true;
	}


	//! @moos_param IMAGES_RATE  Frequency of images synch signal (Hz)
	double desired_img_rate = 10.0;
	m_MissionReader.GetConfigurationParam("IMAGES_RATE",desired_img_rate);


	//! @moos_param CAM_CALIB_FILE  An external config filename with the stereo calib data, in the format expected by mrpt::utils::TStereoCamera
	string sCalibFile;
	m_MissionReader.GetConfigurationParam("CAM_CALIB_FILE",sCalibFile);
	if (!sCalibFile.empty())
	{
		MOOSTrace("Loading stereo camera params from: %s...\n",sCalibFile.c_str());
		ASSERT_FILE_EXISTS_(sCalibFile)

		mrpt::utils::CConfigFile cfg(sCalibFile);
		m_stereo_params.loadFromConfigFile( "CAMERA", cfg );
	}

	// (Default:false) Whether to emit rectified images (true) or raw (false)
	m_live_rectify = m_ini.read_bool("","LIVE_RECTIFY",false);
	m_rectify_alpha = m_ini.read_double("","RECTIFY_ALPHA",0.5);
	m_rectify_align_centers = m_ini.read_bool("","RECTIFY_ALIGN_CENTERS",true);

	Eigen::MatrixXd rectif_size;
	m_ini.read_matrix("","RECTIFY_RESIZE",rectif_size);
	if (rectif_size.cols()==2 && rectif_size.rows()==1)
	{
		m_rectify_resize = true;
		m_rectify_resize_new_size.x = rectif_size(0,0);
		m_rectify_resize_new_size.y = rectif_size(0,1);
	}
	else
	{
		m_rectify_resize = false;
	}

	//! @moos_param IMGS_QUEUE_MAX_LENGTH  (Default=100) Max number of images in the temporary directory / shared memory
	m_MissionReader.GetConfigurationParam("IMGS_QUEUE_MAX_LENGTH",m_imgs_queue_max_length);

	// At start up, make sure that the cameras sync signal is DISABLED:
	MOOSTrace("Disabling SYNCH signal...\n");
	m_Comms.Notify("CLARAQUINO_T1_FREQ", 0.0);

	MOOSTrace("Waiting for the SYNCH signal to be disabled...\n");
	mrpt::system::sleep(1000);

	// For imaging sensors, set external storage directory:
	m_camera.setPathForExternalImages(m_external_img_dirs);


	// Load the camera config block.
	MOOSTrace("Loading camera config block...\n");
	m_camera.loadConfig(m_ini,"");

	// Rectification hook:
	if (m_live_rectify) {
		m_camera.addPreSaveHook( & hook_rectify, this );
	}

	// Init device:
	MOOSTrace("Initializing camera...\n");
	m_camera.initialize();

	// At start up, make sure that the cameras sync signal is DISABLED:
	m_Comms.Notify("CLARAQUINO_T1_FREQ", desired_img_rate);

	m_grab_started = true;

}
catch(std::exception &e)
{
	cerr << e.what();
	return false;
}


	return DoRegistrations();
}

bool CStereoFlyCapApp::OnCommandMsg( CMOOSMsg Msg )
{
	if(Msg.IsSkewed(MOOSTime())) return true;
	if(!Msg.IsString()) return MOOSFail("This module only accepts string command messages\n");
	const std::string sCmd = Msg.GetString();
	//MOOSTrace("COMMAND RECEIVED: %s\n",sCmd.c_str());
	// Process the command "sCmd".

	return true;
}

bool CStereoFlyCapApp::Iterate()
{
	// Have I ever seen the Claraquino module alive?
	// ----------------------------------------------------------
	{
		CMOOSVariable *var = GetMOOSVar("CLARAQUINO_LAST_CMD_STATUS");
		if (var && var->IsFresh()) {
			var->SetFresh(false);
			// ...
		}
	}

	// Normal grabbing main loop:
	// ----------------------------------------------------------
	if (!m_grab_started)
		return true;

	m_camera.doProcess();

	mrpt::hwdrivers::CGenericSensor::TListObservations lstObjs;
	m_camera.getObservations(lstObjs);

	if (lstObjs.empty())
		return true;

	// Quick stats:
	static mrpt::utils::CTicTac tim;
	static unsigned int nImgs = 0;
	if (++nImgs % 30 == 0)
	{
		std::cout << "Image grab rate: " << nImgs/tim.Tac() << " Hz\n";
		nImgs = 0;
		tim.Tic();
	}

	// Send to the DB:
	for (mrpt::hwdrivers::CGenericSensor::TListObservations::iterator it=lstObjs.begin();it!=lstObjs.end();++it)
	{
		CSerializablePtr obj= it->second;
		ASSERT_(IS_DERIVED(obj,CObservationStereoImages));

		// Embed the camera params into the obs:
		CObservationStereoImagesPtr obs = CObservationStereoImagesPtr(obj);
		if (!m_live_rectify)
        {
            obs->leftCamera = m_stereo_params.leftCamera;
            obs->rightCamera = m_stereo_params.rightCamera;
            obs->rightCameraPose = m_stereo_params.rightCameraPose;
        }
		// Serialize:
		mrpt::vector_byte bObs;
		mrpt::utils::ObjectToOctetVector(obs.pointer(), bObs);

		//!  @moos_var     STEREO1_OBS  Stereo images as binary serializations of "CObservationStereoImage" passed to std::vector<uint8_t> through ObjectToOctetVector()
		//!  @moos_publish STEREO1_OBS
		m_Comms.Notify("STEREO1_OBS", bObs );

		// Append to "sent" queue:
		m_imgs_queue.push_back(obs);
	}


	// Purge old imgs and their files:
	ASSERT_(m_imgs_queue_max_length>1)
	purgeStereoExternalImgs(m_imgs_queue_max_length);

	return true;
}

bool CStereoFlyCapApp::OnConnectToServer()
{
	DoRegistrations();
	return true;
}


bool CStereoFlyCapApp::DoRegistrations()
{
	//! @moos_subscribe	CLARAQUINO_LAST_CMD_STATUS
	AddMOOSVariable_OpenMORA("CLARAQUINO_LAST_CMD_STATUS",0);

	RegisterMOOSVariables();
	RegisterMOOSVariables_OpenMORA();
	return true;
}


bool CStereoFlyCapApp::OnNewMail(MOOSMSG_LIST &NewMail)
{
	UpdateMOOSVariables(NewMail);
	UpdateMOOSVariables_OpenMORA(NewMail);
	return true;
}


// Clear all remaining imgs in m_imgs_queue and their external files.
void CStereoFlyCapApp::purgeStereoExternalImgs(size_t nImgsToLeaveInQueue)
{
	while ( m_imgs_queue.size() > nImgsToLeaveInQueue )
	{
		const CObservationStereoImagesPtr &pObs = *m_imgs_queue.begin();
		if (!pObs)
			continue;

		const CObservationStereoImages* obs = pObs.pointer();

		// Left img:
		if (obs->imageLeft.isExternallyStored())
		{
			const char *s = obs->imageLeft.getExternalStorageFileAbsolutePath().c_str();
			if( remove(s)!= 0)
    			fprintf( stderr, "** Error deleting left img file: %s\n",s);
		}
		// Right img:
		if (obs->imageRight.isExternallyStored())
		{
			const char *s = obs->imageRight.getExternalStorageFileAbsolutePath().c_str();
			if( remove(s)!= 0)
    			fprintf( stderr, "** Error deleting right img file: %s\n",s);
		}

		// Delete from list:
		m_imgs_queue.erase( m_imgs_queue.begin() );
	}
}
