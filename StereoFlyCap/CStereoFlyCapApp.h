/* +---------------------------------------------------------------------------+
   |                 Open MORA (MObile Robot Arquitecture)                     |
   |                  University of Almeria ARM-eCar module                    |
   |                                                                           |
   |   Copyright (C) 2014  University of Almeria                               |
   +---------------------------------------------------------------------------+ */

#ifndef CStereoFlyCapApp_H
#define CStereoFlyCapApp_H

#include <COpenMORAMOOSApp.h>

#include <mrpt/hwdrivers/CCameraSensor.h>
#include <mrpt/utils/TStereoCamera.h>
#include <mrpt/slam/CObservationStereoImages.h>
#include <mrpt/vision/CStereoRectifyMap.h>

void hook_rectify(const mrpt::slam::CObservationPtr &o, void* user_ptr);

class CStereoFlyCapApp : public COpenMORAApp
{
	friend void hook_rectify(const mrpt::slam::CObservationPtr &o, void* user_ptr);

public:
    CStereoFlyCapApp();
    virtual ~CStereoFlyCapApp();

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
	mrpt::hwdrivers::CCameraSensor  m_camera;  //!< The camera object. See MRPT docs.
	bool    m_grab_started;  //!< init=false, will be set to true when the sync signal is started and everything starts running.

	std::string m_external_img_dirs; //!< Default: "StereoFlyCap_Images"

	mrpt::utils::TStereoCamera  m_stereo_params; //!< To be embedded into every observation


	unsigned int m_imgs_queue_max_length; //!< Max number of images in the temporary directory / shared memory

	bool m_live_rectify; //!< (Default:false) Whether to emit rectified images (true) or raw (false)
	double m_rectify_alpha; //!< (Default=0.5) Like OpenCV's stereoRectify alpha parameter
	bool   m_rectify_align_centers; //!< (Default=true) Make optical centers to coincide after rectify
	bool   m_rectify_resize;
	mrpt::utils::TImageSize m_rectify_resize_new_size;

    mrpt::synch::CCriticalSection m_rectifier_cs;
	mrpt::vision::CStereoRectifyMap m_rectifier;

	/** The list of imgs already sent to the DB, to be deleted in a while, to give time to other modules to grab the imgs if needed */
	std::list<mrpt::slam::CObservationStereoImagesPtr> m_imgs_queue;

	void purgeStereoExternalImgs(size_t nImgsToLeaveInQueue = 0); //!< Clear all remaining imgs in m_imgs_queue and their external files.

};
#endif
