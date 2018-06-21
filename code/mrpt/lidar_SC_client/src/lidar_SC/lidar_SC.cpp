/*************************************************************************
  > File Name: icp-slam-live_main.cpp
  > Author: tangbf
  > Mail: 
  > Created Time: 2018年06月15日 星期五 09时49分32秒
 ************************************************************************/
#include <mrpt/hwdrivers/CGenericSensor.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/slam/CMetricMapBuilderICP.h>
#include <mrpt/utils/CConfigFile.h>
#include <mrpt/utils/CFileGZInputStream.h>
#include <mrpt/utils/CFileGZOutputStream.h>
#include <mrpt/utils/CFileOutputStream.h>
#include <mrpt/system/os.h>
#include <mrpt/system/threads.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/stock_objects.h>
#include <mrpt/opengl/CPlanarLaserScan.h>
#include <mrpt/gui/CDisplayWindow3D.h>

#include <mrpt/gui/CDisplayWindowPlots.h>



#define HAVE_LGLIDAR_HARDRIVERS 1
#if HAVE_LGLIDAR_HARDRIVERS
#include <mrpt/hwdrivers/CSerialPort.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/system/string_utils.h>
#include "LGlidar.h"
#endif

#include <iostream>

using namespace mrpt::hwdrivers;
using namespace mrpt::system;
using namespace std;
using namespace mrpt::obs;
using namespace mrpt::poses;
using namespace mrpt::maps;
using namespace mrpt::gui;
using namespace mrpt::utils;

using namespace mrpt::gui;
int main(int argc, char** argv)
{
	LGlidar laser;
	string serName = "ttyUSB0";
	string intensity = "n";
	bool allThreadsMustExit = false;

	laser.setSerialPort(serName);
	laser.setIntensityMode(lowerCase(intensity) == "y");

	if(laser.turnOn()){
		cout<<"initialization OK!"<<endl;
	}else{
		cout<<"initialization failed!"<<endl;
	}

	//CTicTac tictac;
	//tictac.Tic();
	
	CDisplayWindowPlots win("Lidar scans");


	while(true){

		bool thereIsObservation, hardError;
		CObservation2DRangeScan obs;

		laser.doProcessSimple(thereIsObservation, obs, hardError);

		if(hardError){
			cout<<"hardware error!"<<endl;
		}
		// cout<<"thereIsObservation:" << thereIsObservation<<endl;
		if(thereIsObservation){
	//		double FPS = 1.0 / tictiac.Tac();

			printf("Scan received:%u ranges, FOV:%.02fdeg, mid rang=%f\n",(unsigned int)obs.scan.size(), obs.aperture, obs.scan[obs.scan.size()/2]);


			obs.sensorPose = CPose3D(0, 0, 0);
			mrpt::maps::CSimplePointsMap theMap;
			theMap.insertionOptions.minDistBetweenLaserPoints = 0;
			theMap.insertObservation(&obs);

			std::vector<float> xs, ys, zs;
			theMap.getAllPoints(xs, ys, zs);
			win.plot(xs, ys, ".b3");
			win.axis_equal();
		}
	}

	cout<<"123"<<endl;
	return 0;
}
