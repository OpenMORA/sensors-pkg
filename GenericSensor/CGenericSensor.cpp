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



/**  @moos_module Uses MRPT's CGenericSensor to open an arbitrary sensor and publish its data in a timely fashion.
  *    This module can access to several different sensors: lasers, cameras, sonars, etc...
  *    The configuration block of the MOOS mission file for this module must
  *    contain the same parameters than those expected by mrpt::hwdrivers::CGenericSensor and
  *    the MRPT program rawlog-grabber.
  */

#include "CGenericSensor.h"
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/math/ops_containers.h>
#include <sstream>
#include <iomanip>
#include <iostream>


using namespace std;
using namespace mrpt;
using namespace mrpt::math;
using namespace mrpt::obs;
using namespace mrpt::utils;
using namespace mrpt::hwdrivers;


CGenericSensorApp::CGenericSensorApp()
{
}

CGenericSensorApp::~CGenericSensorApp()
{
}

bool CGenericSensorApp::OnStartUp()
{
    DoRegistrations();

    try
    {
		// Load config from .moos mission file:
		const string driver_name = m_ini.read_string("","driver","",true);

		m_sensor = CGenericSensorPtr( CGenericSensor::createSensor(driver_name ) );
		if (!m_sensor)
			return MOOSFail("***ERROR***: Class name not recognized: %s", driver_name.c_str() );

		// Load common & sensor specific parameters:
		m_sensor->loadConfig( m_ini, "" );

		// Init device:
		m_sensor->initialize();

		// Load specific kinect conf if needed
		if ( driver_name == "CKinect" )
		{
			m_decimate_type = m_ini.read_string( "", "decimate_type", "regions_decimate" );
			cout << "[INFO] Type of decimation: " << m_decimate_type << endl;

			m_ini.read_vector( "", "rows_to_include", vector<size_t>(), m_rows_to_include );

			m_only_one_of_each = m_ini.read_int( "", "only_one_of_each", 1) ;
			//cout << "Only one of each " << m_only_one_of_each << endl;

			m_regions_decimate_height	= m_ini.read_int( "", "regions_decimate_height",640 );
			m_regions_decimate_width	= m_ini.read_int( "", "regions_decimate_width",1 );

			//ASSERT_ ( ( 480 % m_regions_decimate_height ) == 0 );
			//ASSERT_ ( ( 640 % m_regions_decimate_width ) == 0 );

		}

		return true;
    }
	catch (std::exception &e)
	{
		return MOOSFail( "**Startup ERROR** %s", e.what() );
	}
}

bool CGenericSensorApp::OnCommandMsg( CMOOSMsg Msg )
{
    if(Msg.IsSkewed(MOOSTime()))
        return true;

    if(!Msg.IsString())
        return MOOSFail("pGenericSensor only accepts string command messages\n");

    std::string sCmd = Msg.GetString();

    MOOSTrace("COMMAND RECEIVED: %s\n",sCmd.c_str());

    return true;
}

void simpleDecimate3DRangeScan( const CObservationPtr &obs, CObservation3DRangeScanPtr &newObs, const vector<size_t> &v_rows, const size_t & onlyOneOfEach )
{
	newObs = CObservation3DRangeScanPtr( obs );

	size_t N = v_rows.size();

	size_t cols = newObs->cameraParams.ncols;
	size_t rows = newObs->cameraParams.nrows;

	for ( size_t i = 0; i < N; i++ )
		ASSERT_( v_rows[i] < rows );

	vector<float> x, y, z;
	x.swap( newObs->points3D_x );
	y.swap( newObs->points3D_y );
	z.swap( newObs->points3D_z );

	//newObs->points3D_x.clear(); newObs->points3D_x.resize( floor( (float)( N*cols / onlyOneOfEach ) ) );
	//newObs->points3D_y.clear(); newObs->points3D_y.resize( floor( (float)( N*cols / onlyOneOfEach ) ) );
	//newObs->points3D_z.clear(); newObs->points3D_z.resize( floor( (float)( N*cols / onlyOneOfEach ) ) );

	for ( size_t i = 0; i < N; i++ )
	{

		size_t begin = v_rows[i]*cols;
		//size_t end = begin + cols - 1;

		//cout << "Begin: "<< begin << "End: " << end << endl;

		for ( size_t j = 0; j < cols; j = j + onlyOneOfEach )
		{
			//newObs->points3D_x[i*cols+j] = x[begin+j];
			//newObs->points3D_y[i*cols+j] = y[begin+j];
			//newObs->points3D_z[i*cols+j] = z[begin+j];
			newObs->points3D_x.push_back(x[begin+j]);
			newObs->points3D_y.push_back(y[begin+j]);
			newObs->points3D_z.push_back(z[begin+j]);
			//newObs->points3D_x.assign( x.begin() + begin, x.begin() + end );
			//newObs->points3D_y.assign( y.begin() + begin, y.begin() + end );
			//newObs->points3D_z.assign( z.begin() + begin, z.begin() + end );
		}

	}

	//FILE *f = fopen("pepe.txt","a+");
	//for (size_t i = 0; i < 20; i++)
	//	fprintf(f,"%f",newObs->points3D_x[(cols/2)+i-10]);
	for (size_t i = 0; i < 5; i++)
		cout << newObs->points3D_x[(cols/2)+i-10] << " ";

	cout << endl;

	//fprintf(f,"\n");
	//	//fprintf(f,"%f\t%f\t%f\n",newObs->points3D_x[cols/2],newObs->points3D_y[cols/2],newObs->points3D_z[cols/2]);
	//fclose(f);

	//cout << "Size: " << newObs->points3D_x.size();


}

void regionsDecimate3DRangeScan( const CObservationPtr &obs, CObservation3DRangeScanPtr &newObs, const size_t &region_height, const size_t &region_width )
{
	newObs = CObservation3DRangeScanPtr( obs );

	size_t n_cols = newObs->cameraParams.ncols;
	size_t n_rows = newObs->cameraParams.nrows;

	vector<float> x, y, z;
	x.swap( newObs->points3D_x );
	y.swap( newObs->points3D_y );
	z.swap( newObs->points3D_z );

	// The number of points is the total number of regions
	size_t num_of_points = (n_cols/region_width)*(n_rows/region_height);
	cout << "Theorical num of points: " << num_of_points << endl;

	//cout << "Num of points= " << num_of_points << endl;
	//cout << "Num cols= " << n_cols/region_width << endl;
	//cout << "Num rows= " << n_rows/region_height << endl;


	// Clean vectors and resize
	newObs->points3D_x.clear(); newObs->points3D_x.resize( num_of_points );
	newObs->points3D_y.clear(); newObs->points3D_y.resize( num_of_points );
	newObs->points3D_z.clear(); newObs->points3D_z.resize( num_of_points );

	// Obtain the nearest point of each region
	for ( size_t region_col = 0; region_col < n_cols/region_width; region_col++ )
	{
		for ( size_t region_row = 0; region_row < n_rows/region_height; region_row++ )
		{
			double min_z = 200; // Any value is good if it's higher that the maximum Z distance value returned by kinect
			size_t index_min_z=std::string::npos;	// To store the index of the point with the minimun Z distance

			// Obtain the nearest point of the current region
			for ( size_t col = region_width*region_col; col < region_width*region_col + region_width; col++ )
			{
				for ( size_t row = region_height*region_row; row < region_height*region_row + region_height; row++ )
				{
					// Check if the actual point is the nearest (by now) to the kinect sensor
					size_t index_to_check = row*n_cols + col;
					if ( x[index_to_check] && ( x[index_to_check] < min_z ) )
					{
						min_z = x[index_to_check];
						index_min_z = index_to_check;
					}
				}
			}

			if (index_min_z!=std::string::npos)
			{

				// Insert the nearest point of the region
				size_t index_to_insert = region_col*( n_rows/region_height ) + region_row;
				//cout << "Index min z: " << index_min_z << endl;
				//cout << index_to_insert << " Inserted: " <<	x[index_min_z] << " " << y[index_min_z] << " " << z[index_min_z] << endl;

				newObs->points3D_x[ index_to_insert ] = x[index_min_z];
				newObs->points3D_y[ index_to_insert ] = y[index_min_z];
				newObs->points3D_z[ index_to_insert ] = z[index_min_z];
			}
		}
	}

	//cout << "Size: " << newObs->points3D_x.size() << endl;

}

bool checkIntensityNeighborhood( const CObservation3DRangeScanPtr &auxObs, const size_t &i_col, const size_t &i_row )
{
	int c1 = i_col-4, c2 = i_col + 4;
	int r1 = i_row-4, r2 = i_row + 4;

	if ( r1 < 0 ) r1 = 0;
	if ( r2 > 487 ) r2 = 487;
	if ( c1 < 0 ) c1 = 0;
	if ( c2 > 639 ) c2 = 639;

	//printf("row: %d, col: %d, r1: %d, r2: %d, c1: %d, c2: %d \n",i_row,i_col,r1,r2,c1,c2);
	//cout << "Size: " << ( r2 - r1 + 1 ) * ( c2 - c1 + 1 ) << endl;
	//if ( ( r2 - r1 + 1 ) * ( c2 - c1 + 1 ) > 81 )
	//	mrpt::system::pause();

	vector<size_t> v_intensity;
	v_intensity.resize( ( r2 - r1 + 1 ) * ( c2 - c1 + 1 ) );

	size_t count = 0;

	for ( int col = c1; col <= c2; col++ )
		for ( int row = r1; row <= r2; row++ )
		{
			unsigned char *c = auxObs->intensityImage(col,row);
			v_intensity[count] = (size_t)*c;
			//cout << v_intensity[count] << endl;
			count++;
		}

	//cout << mean( v_intensity ) << endl;

	double meanValue =mrpt::math::mean( v_intensity );

	if ( meanValue > 240 )
		return false;
	else
		return true;

	/*ofstream file("measures.txt",std::ios::app);
	file << i_col << " mean = " << mean( v_intensity ) << endl;
	file.close();*/
}

void colsDecimate3DRangeScan( const CObservationPtr &obs, CObservation2DRangeScanPtr &newObs )
{
	CObservation3DRangeScanPtr auxObs = CObservation3DRangeScanPtr( obs );

	size_t n_cols = auxObs->cameraParams.ncols;
	size_t n_rows = auxObs->cameraParams.nrows;

	newObs = CObservation2DRangeScan::Create();

	// Prepare newObs paramters
	newObs->scan.resize( n_cols+34, 0 );
	newObs->validRange.resize( n_cols+34, false );
	double FOV = 59.454383669258749758337144308389;
	newObs->aperture = DEG2RAD( FOV );
	newObs->rightToLeft = false;

	newObs->sensorLabel = "KINECT";

	//auxObs->intensityImage.saveToFile("1.jpg");

	/*if ( auxObs->hasPoints3D )
		cout << "OK" << endl;
	else
		cout << "FARSO" << endl;*/

	//cout << ">>" << auxObs->points3D_x.size() << "<<" << endl;

	vector<double> x_orig, y_orig, alfa_orig;

	//x_orig.resize(640,0); y_orig.resize(640,0); alfa_orig.resize(640,0);

	//printf("Received coordinates <x,y,z> = <%f,%f,%f>\n",auxObs->points3D_x[0],auxObs->points3D_y[0],auxObs->points3D_z[0]);

	vector<float> x, y, z;
	x.swap( auxObs->points3D_x );
	y.swap( auxObs->points3D_y );
	z.swap( auxObs->points3D_z );

	// Debug purposes
//	double x_orig, y_orig, alfa_orig;

	// Obtain the nearest point of each colum
	for ( size_t col = 0; col < n_cols; col++ )
	{
		bool anyFailMeasure = false;

		double min_x = 200; // Any value is good if it's higher that the maximum Z distance value returned by kinect
		//size_t index_min_z=std::string::npos;	// To store the index of the point with the minimun Z distance

		for ( size_t row = 0; row < n_rows; row++ )
		{
			if ( !anyFailMeasure )
			{
				// Check if the actual point is the nearest (by now) to the kinect sensor
				size_t index_to_check = row*n_cols + col;

				double depth_value = x[index_to_check];

				if ( !depth_value )
				{
					if ( !checkIntensityNeighborhood(auxObs,col,row) )
					{
						anyFailMeasure = true;
						newObs->validRange[ col ] = false;
					}
				}
				else if ( depth_value < min_x )
				{
					TPoint3D origin(0,0,0);
					newObs->scan[ col ] = origin.distanceTo(TPoint3D( x[index_to_check], y[index_to_check], 0 ) );
					newObs->validRange[ col ] = true;

					/*x_orig[col] = x[index_to_check];
					y_orig[col] = y[index_to_check];
					alfa_orig[col] = atan(y_orig[col]/x_orig[col]);					*/
				}
			}
			else
				break; // This column has at least one row with and invalid measure,so it's invalid too! (beastlike reasoning!)
		}
	}

	/*size_t col_ini = 0, col_final = 631;

	ofstream file;
	file.open("coordinates.txt",std::ios_base::app);

	for( size_t row = 0; row < n_rows; row++ )
	{
		file << x[row*n_cols + col_ini] << "\t";
		file << y[row*n_cols + col_ini]<< "\t";
		file << z[row*n_cols + col_ini]<< "\t";
		file << x[row*n_cols + col_final]<< "\t";
		file << y[row*n_cols + col_final]<< "\t";
		file << z[row*n_cols + col_final]<< "\t" << endl;
	}

	file.close();*/

	/*cout << "Insertion test: " << endl;
	cout << "Original x,y: " << x_orig << "," << y_orig << " alfa: " << RAD2DEG(alfa_orig) << endl;
	double alfa = DEG2RAD( FOV )/2;
	cout << "Processed x,y: " << newObs->scan[0]*cos( alfa ) << "," << newObs->scan[0]*sin( alfa ) << endl;*/
	//cout << "Size: " << newObs->points3D_x.size() << endl;

	/*cout << "ERRORS: " << endl;

	ofstream file("pepe.txt");
	for ( size_t index = 0; index < 640; index++ )
	{
		double alfa = (DEG2RAD( FOV )/2) - (DEG2RAD(FOV)/newObs->scan.size())*index;
		file << "Alfa = " << RAD2DEG(alfa) << endl;
		file << "Alfa orig = " << RAD2DEG(alfa_orig[index]) << endl;
		file << "Alfa error = " << RAD2DEG(alfa) - RAD2DEG(alfa_orig[index]) << endl;
		char s[1024];
		file << "Orig point <x,y> = <" << x_orig[index] << "," << y_orig[index] << ">" << endl;
		if ( !newObs->validRange[index] ) file << "INVALID!!!" << endl; else file << "VALID!" << endl;
		file << "Processed point <x,y> = <" << newObs->scan[index]*cos( alfa ) << "," << newObs->scan[index]*sin( alfa ) << "> Dist = " << newObs->scan[index] << endl;
		sprintf(s,"Error %d <x,y> = <%f,%f>\n", index, x_orig[index]-newObs->scan[index]*cos( alfa ), y_orig[index]-newObs->scan[index]*sin( alfa ));
		file<<s;

	}
	file.close();
	mrpt::system::pause();*/



}


void customizedDecimate3DRangeScan( const CObservationPtr &obs, CObservation3DRangeScanPtr &newObs )
{
	newObs = CObservation3DRangeScanPtr( obs );

	size_t n_cols = newObs->cameraParams.ncols;
	//size_t n_rows = newObs->cameraParams.nrows;

	size_t initial_col = 312;
	size_t final_col = 328;
	size_t initial_row = 240;
	size_t final_row = 460;

	vector<float> x, y, z;
	x.swap( newObs->points3D_x );
	y.swap( newObs->points3D_y );
	z.swap( newObs->points3D_z );

	// The number of points is the total number of regions
	size_t num_of_points = final_col  - initial_col;
	cout << "Theorical num of points: " << num_of_points << endl;

	//cout << "Num of points= " << num_of_points << endl;
	//cout << "Num cols= " << n_cols/region_width << endl;
	//cout << "Num rows= " << n_rows/region_height << endl;


	// Clean vectors and resize
	newObs->points3D_x.clear(); newObs->points3D_x.resize( num_of_points );
	newObs->points3D_y.clear(); newObs->points3D_y.resize( num_of_points );
	newObs->points3D_z.clear(); newObs->points3D_z.resize( num_of_points );



	// Obtain the nearest point of the current region
	for ( size_t col = initial_col; col < final_col; col++ )
	{
		double min_x = 200; // Any value is good if it's higher than the maximum X distance value returned by kinect
		size_t index_min_x=std::string::npos;	// To store the index of the point with the minimun Z distance

		for ( size_t row = initial_row; row < final_row; row++ )
		{
			// Check if the actual point is the nearest (by now) to the kinect sensor
			size_t index_to_check = row*n_cols + col;
			double high = z[index_to_check];
			if ( ( high > -0.7 ) && ( high < 2 ) ) // Take in account only points in this high range
			{
				if ( x[index_to_check] && ( x[index_to_check] < min_x ) )
				{
					min_x = x[index_to_check];
					index_min_x = index_to_check;
				}
			}
		}

		size_t index_to_insert = col - initial_col;
		if (index_min_x!=std::string::npos)
		{
			// Insert the nearest point of the region
			newObs->points3D_x[ index_to_insert ] = x[index_min_x];
			newObs->points3D_y[ index_to_insert ] = y[index_min_x];
			newObs->points3D_z[ index_to_insert ] = z[index_min_x];
		}
		else
		{
			newObs->points3D_x[ index_to_insert ] = newObs->points3D_y[ index_to_insert ] = newObs->points3D_z[ index_to_insert ] = 0;
		}
	}

	//cout << "Size: " << newObs->points3D_x.size() << endl;

}

bool CGenericSensorApp::Iterate()
{
	try
	{
		if (!m_sensor)
			return MOOSFail("There is no correctly initialized sensor!");

		// Process
		m_sensor->doProcess();

		// Get new observations
		CGenericSensor::TListObservations	lstObjs;
		m_sensor->getObservations( lstObjs );

		// Send just the most recent observation to the DB:
		if (lstObjs.empty())
			return true;

		CSerializablePtr obj= lstObjs.rbegin()->second;
		ASSERT_(IS_DERIVED(obj,CObservation));

		CObservationPtr obs = CObservationPtr(obj);

		{
			static int nSent = 0;
			static CTicTac tictac;
			if (!nSent) tictac.Tic();
			nSent++;
			const double rate = double(nSent)/(1e-5+tictac.Tac());

			static CTicTac tictacShow;

			if (tictacShow.Tac()>3.0)
			{
				tictacShow.Tic();
				printf("[pGenericSensor] SENSOR RATE %s: %f\n", obs->sensorLabel.c_str(), rate );

				//!  @moos_var   <SENSOR_LABEL>_RATE   The approximate observation gathering rate (in Hz) of the given sensor.
				//!  @moos_publish   <SENSOR_LABEL>_RATE
				m_Comms.Notify(obs->sensorLabel+string("_RATE"), rate );
			}
		}

		if (obs->sensorLabel.empty())
			throw std::logic_error("The 'sensorLabel' must be non-empty.");

		//!  @moos_var   <SENSOR_LABEL>   The MRPT observation parsed as a std::vector<uint8_t> through ObjectToOctetVector
		//!  @moos_publish   <SENSOR_LABEL>

		if ( obs->sensorLabel == "KINECT1" )
		{
			CObservation3DRangeScanPtr testObs3D = CObservation3DRangeScanPtr( obs );

			if ( testObs3D->hasPoints3D )
			{
				CObservation3DRangeScanPtr obs3D; // = CObservation3DRangeScanPtr( obs );
				CObservation2DRangeScanPtr obs2D;
				string type;

				if ( m_decimate_type == "simple_decimate" )
				{
					simpleDecimate3DRangeScan( obs, obs3D, m_rows_to_include, m_only_one_of_each );
					type = "3D";
				}
				else if ( m_decimate_type == "regions_decimate" )
				{
					m_timeLog.enter("Regions decimate 3D range scan");
					regionsDecimate3DRangeScan( obs, obs3D, m_regions_decimate_height, m_regions_decimate_width );
					m_timeLog.leave("Regions decimate 3D range scan");
					type = "3D";
				}
				else if ( m_decimate_type == "customized_decimate" )
				{
					m_timeLog.enter("Customized decimate 3D range scan");
					customizedDecimate3DRangeScan( obs, obs3D );
					m_timeLog.leave("Customized decimate 3D range scan");
					type = "3D";
				}
				else if ( m_decimate_type == "cols_decimate" )
				{
					m_timeLog.enter("Columns decimate 3D range scan");
					colsDecimate3DRangeScan( obs, obs2D );
					m_timeLog.leave("Columns decimate 3D range scan");
					type = "2D";
				}

				//size_t N = obs3D->points3D_x.size();
				//cout << "Final Size: " << N << endl;

				//cout << "Range image: " << obs3D->hasRangeImage << endl;
				//cout << "Intensity image: " << obs3D->hasIntensityImage << endl;

				mrpt::vector_byte bObs;

				if ( type == "3D" )
				      mrpt::utils::ObjectToOctetVector(obs3D.pointer(), bObs);
				else  mrpt::utils::ObjectToOctetVector(obs3D.pointer(), bObs);

				m_Comms.Notify(obs->sensorLabel, bObs );
			}

		}
		else
		{
			mrpt::vector_byte bObs;
			mrpt::utils::ObjectToOctetVector(obs.pointer(), bObs);
		    m_Comms.Notify(obs->sensorLabel, bObs );
		}

		return true;
    }
	catch (std::exception &e)
	{
		return MOOSFail( "**Iterate ERROR** %s", e.what() );
	}
}

bool CGenericSensorApp::OnConnectToServer()
{
    DoRegistrations();
    return true;
}


bool CGenericSensorApp::DoRegistrations()
{
	//! @moos_subscribe SHUTDOWN
	AddMOOSVariable( "SHUTDOWN", "SHUTDOWN", "SHUTDOWN", 0 );
    RegisterMOOSVariables();
    return true;
}

bool CGenericSensorApp::OnNewMail(MOOSMSG_LIST &NewMail)
{
	std::string cad;
	for(MOOSMSG_LIST::iterator i=NewMail.begin();i!=NewMail.end();++i)
	{
		if( (i->GetName()=="SHUTDOWN") && (MOOSStrCmp(i->GetString(),"true")) )
		{
			// Disconnect comms:
			MOOSTrace("Closing Module \n");
			this->RequestQuit();
		}

	}


    UpdateMOOSVariables(NewMail);
    return true;
}
