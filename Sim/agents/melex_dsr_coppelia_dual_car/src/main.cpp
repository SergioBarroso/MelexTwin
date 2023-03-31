/*
 *    Copyright (C) 2022 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */


/** \mainpage RoboComp::pioneer_dsr
 *
 * \section intro_sec Introduction
 *
 * The pioneer_dsr component...
 *
 * \section interface_sec Interface
 *
 * interface...
 *
 * \section install_sec Installation
 *
 * \subsection install1_ssec Software depencences
 * ...
 *
 * \subsection install2_ssec Compile and install
 * cd pioneer_dsr
 * <br>
 * cmake . && make
 * <br>
 * To install:
 * <br>
 * sudo make install
 *
 * \section guide_sec User guide
 *
 * \subsection config_ssec Configuration file
 *
 * <p>
 * The configuration file etc/config...
 * </p>
 *
 * \subsection execution_ssec Execution
 *
 * Just: "${PATH_TO_BINARY}/pioneer_dsr --Ice.Config=${PATH_TO_CONFIG_FILE}"
 *
 * \subsection running_ssec Once running
 *
 * ...
 *
 */
#include <signal.h>

// QT includes
#include <QtCore>
#include <QtWidgets>

// ICE includes
#include <Ice/Ice.h>
#include <IceStorm/IceStorm.h>
#include <Ice/Application.h>

#include <rapplication/rapplication.h>
#include <sigwatch/sigwatch.h>
#include <qlog/qlog.h>

#include "config.h"
#include "genericmonitor.h"
#include "genericworker.h"
#include "specificworker.h"
#include "specificmonitor.h"
#include "commonbehaviorI.h"


#include <GenericBase.h>



class pioneer_dsr : public RoboComp::Application
{
public:
	pioneer_dsr (QString prfx, bool startup_check) { prefix = prfx.toStdString(); this->startup_check_flag=startup_check; }
private:
	void initialize();
	std::string prefix;
	TuplePrx tprx;
	bool startup_check_flag = false;

public:
	virtual int run(int, char*[]);
};

void ::pioneer_dsr::initialize()
{
	// Config file properties read example
	// configGetString( PROPERTY_NAME_1, property1_holder, PROPERTY_1_DEFAULT_VALUE );
	// configGetInt( PROPERTY_NAME_2, property1_holder, PROPERTY_2_DEFAULT_VALUE );
}

int ::pioneer_dsr::run(int argc, char* argv[])
{
#ifdef USE_QTGUI
	QApplication a(argc, argv);  // GUI application
#else
	QCoreApplication a(argc, argv);  // NON-GUI application
#endif


	sigset_t sigs;
	sigemptyset(&sigs);
	sigaddset(&sigs, SIGHUP);
	sigaddset(&sigs, SIGINT);
	sigaddset(&sigs, SIGTERM);
	sigprocmask(SIG_UNBLOCK, &sigs, 0);

	UnixSignalWatcher sigwatch;
	sigwatch.watchForSignal(SIGINT);
	sigwatch.watchForSignal(SIGTERM);
	QObject::connect(&sigwatch, SIGNAL(unixSignal(int)), &a, SLOT(quit()));

	int status=EXIT_SUCCESS;

	RoboCompCameraRGBDSimpleMulti::CameraRGBDSimpleMultiPrxPtr camerargbdsimplemulti_proxy;
	RoboCompDifferentialRobotMulti::DifferentialRobotMultiPrxPtr differentialrobotmulti_proxy;
	RoboCompFullPoseEstimationMulti::FullPoseEstimationMultiPrxPtr fullposeestimationmulti_proxy;
	RoboCompGpsUbloxMulti::GpsUbloxMultiPrxPtr gpsubloxmulti_proxy;
	RoboCompLaserMulti::LaserMultiPrxPtr lasermulti_proxy;

	string proxy, tmp;
	initialize();

	try
	{
		if (not GenericMonitor::configGetString(communicator(), prefix, "CameraRGBDSimpleMultiProxy", proxy, ""))
		{
			cout << "[" << PROGRAM_NAME << "]: Can't read configuration for proxy CameraRGBDSimpleMultiProxy\n";
		}
		camerargbdsimplemulti_proxy = Ice::uncheckedCast<RoboCompCameraRGBDSimpleMulti::CameraRGBDSimpleMultiPrx>( communicator()->stringToProxy( proxy ) );
	}
	catch(const Ice::Exception& ex)
	{
		cout << "[" << PROGRAM_NAME << "]: Exception creating proxy CameraRGBDSimpleMulti: " << ex;
		return EXIT_FAILURE;
	}
	rInfo("CameraRGBDSimpleMultiProxy initialized Ok!");


	try
	{
		if (not GenericMonitor::configGetString(communicator(), prefix, "DifferentialRobotMultiProxy", proxy, ""))
		{
			cout << "[" << PROGRAM_NAME << "]: Can't read configuration for proxy DifferentialRobotMultiProxy\n";
		}
		differentialrobotmulti_proxy = Ice::uncheckedCast<RoboCompDifferentialRobotMulti::DifferentialRobotMultiPrx>( communicator()->stringToProxy( proxy ) );
	}
	catch(const Ice::Exception& ex)
	{
		cout << "[" << PROGRAM_NAME << "]: Exception creating proxy DifferentialRobotMulti: " << ex;
		return EXIT_FAILURE;
	}
	rInfo("DifferentialRobotMultiProxy initialized Ok!");


	try
	{
		if (not GenericMonitor::configGetString(communicator(), prefix, "FullPoseEstimationMultiProxy", proxy, ""))
		{
			cout << "[" << PROGRAM_NAME << "]: Can't read configuration for proxy FullPoseEstimationMultiProxy\n";
		}
		fullposeestimationmulti_proxy = Ice::uncheckedCast<RoboCompFullPoseEstimationMulti::FullPoseEstimationMultiPrx>( communicator()->stringToProxy( proxy ) );
	}
	catch(const Ice::Exception& ex)
	{
		cout << "[" << PROGRAM_NAME << "]: Exception creating proxy FullPoseEstimationMulti: " << ex;
		return EXIT_FAILURE;
	}
	rInfo("FullPoseEstimationMultiProxy initialized Ok!");


	try
	{
		if (not GenericMonitor::configGetString(communicator(), prefix, "GpsUbloxMultiProxy", proxy, ""))
		{
			cout << "[" << PROGRAM_NAME << "]: Can't read configuration for proxy GpsUbloxMultiProxy\n";
		}
		gpsubloxmulti_proxy = Ice::uncheckedCast<RoboCompGpsUbloxMulti::GpsUbloxMultiPrx>( communicator()->stringToProxy( proxy ) );
	}
	catch(const Ice::Exception& ex)
	{
		cout << "[" << PROGRAM_NAME << "]: Exception creating proxy GpsUbloxMulti: " << ex;
		return EXIT_FAILURE;
	}
	rInfo("GpsUbloxMultiProxy initialized Ok!");


	try
	{
		if (not GenericMonitor::configGetString(communicator(), prefix, "LaserMultiProxy", proxy, ""))
		{
			cout << "[" << PROGRAM_NAME << "]: Can't read configuration for proxy LaserMultiProxy\n";
		}
		lasermulti_proxy = Ice::uncheckedCast<RoboCompLaserMulti::LaserMultiPrx>( communicator()->stringToProxy( proxy ) );
	}
	catch(const Ice::Exception& ex)
	{
		cout << "[" << PROGRAM_NAME << "]: Exception creating proxy LaserMulti: " << ex;
		return EXIT_FAILURE;
	}
	rInfo("LaserMultiProxy initialized Ok!");


	tprx = std::make_tuple(camerargbdsimplemulti_proxy,differentialrobotmulti_proxy,fullposeestimationmulti_proxy,gpsubloxmulti_proxy,lasermulti_proxy);
	SpecificWorker *worker = new SpecificWorker(tprx, startup_check_flag);
	//Monitor thread
	SpecificMonitor *monitor = new SpecificMonitor(worker,communicator());
	QObject::connect(monitor, SIGNAL(kill()), &a, SLOT(quit()));
	QObject::connect(worker, SIGNAL(kill()), &a, SLOT(quit()));
	monitor->start();

	if ( !monitor->isRunning() )
		return status;

	while (!monitor->ready)
	{
		usleep(10000);
	}

	try
	{
		try {
			// Server adapter creation and publication
			if (not GenericMonitor::configGetString(communicator(), prefix, "CommonBehavior.Endpoints", tmp, "")) {
				cout << "[" << PROGRAM_NAME << "]: Can't read configuration for proxy CommonBehavior\n";
			}
			Ice::ObjectAdapterPtr adapterCommonBehavior = communicator()->createObjectAdapterWithEndpoints("commonbehavior", tmp);
			auto commonbehaviorI = std::make_shared<CommonBehaviorI>(monitor);
			adapterCommonBehavior->add(commonbehaviorI, Ice::stringToIdentity("commonbehavior"));
			adapterCommonBehavior->activate();
		}
		catch(const Ice::Exception& ex)
		{
			status = EXIT_FAILURE;

			cout << "[" << PROGRAM_NAME << "]: Exception raised while creating CommonBehavior adapter: " << endl;
			cout << ex;

		}



		// Server adapter creation and publication
		cout << SERVER_FULL_NAME " started" << endl;

		// User defined QtGui elements ( main window, dialogs, etc )

		#ifdef USE_QTGUI
			//ignoreInterrupt(); // Uncomment if you want the component to ignore console SIGINT signal (ctrl+c).
			a.setQuitOnLastWindowClosed( true );
		#endif
		// Run QT Application Event Loop
		a.exec();


		status = EXIT_SUCCESS;
	}
	catch(const Ice::Exception& ex)
	{
		status = EXIT_FAILURE;

		cout << "[" << PROGRAM_NAME << "]: Exception raised on main thread: " << endl;
		cout << ex;

	}
	#ifdef USE_QTGUI
		a.quit();
	#endif

	status = EXIT_SUCCESS;
	monitor->terminate();
	monitor->wait();
	delete worker;
	delete monitor;
	return status;
}

int main(int argc, char* argv[])
{
	string arg;

	// Set config file
	QString configFile("etc/config");
	bool startup_check_flag = false;
	QString prefix("");
	if (argc > 1)
	{

		// Search in argument list for arguments
		QString startup = QString("--startup-check");
		QString initIC = QString("--Ice.Config=");
		QString prfx = QString("--prefix=");
		for (int i = 0; i < argc; ++i)
		{
			arg = argv[i];
			if (arg.find(startup.toStdString(), 0) != std::string::npos)
			{
				startup_check_flag = true;
				cout << "Startup check = True"<< endl;
			}
			else if (arg.find(prfx.toStdString(), 0) != std::string::npos)
			{
				prefix = QString::fromStdString(arg).remove(0, prfx.size());
				if (prefix.size()>0)
					prefix += QString(".");
				printf("Configuration prefix: <%s>\n", prefix.toStdString().c_str());
			}
			else if (arg.find(initIC.toStdString(), 0) != std::string::npos)
			{
				configFile = QString::fromStdString(arg).remove(0, initIC.size());
				qDebug()<<__LINE__<<"Starting with config file:"<<configFile;
			}
			else if (i==1 and argc==2 and arg.find("--", 0) == std::string::npos)
			{
				configFile = QString::fromStdString(arg);
				qDebug()<<__LINE__<<QString::fromStdString(arg)<<argc<<arg.find("--", 0)<<"Starting with config file:"<<configFile;
			}
		}

	}
	::pioneer_dsr app(prefix, startup_check_flag);

	return app.main(argc, argv, configFile.toLocal8Bit().data());
}
