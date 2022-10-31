#ifndef _QSLAMANDNAV_WIDGET_H
#define _QSLAMANDNAV_WIDGET_H

#include "gmapping/gui/qmappainter.h"
#include "gmapping/gui/qpixmapdumper.h"
#include <gmapping/utils/point.h>
#include <list>

class QSLAMandNavWidget :  public QMapPainter{
	public:
		QSLAMandNavWidget( QWidget * parent = 0, const char * name = 0, WFlags f = 0);
		virtual ~QSLAMandNavWidget();
		std::list<GMapping::IntPoint > trajectoryPoints;
		GMapping::IntPoint robotPose;
		double robotHeading;
		
		bool slamRestart;
		bool slamFinished;
		bool enableMotion;
		bool startWalker;
		bool trajectorySent;
		bool goHome;
		bool wantsQuit;
		bool printHelp;
		bool saveGoalPoints;
		bool writeImages;
		bool drawRobot;
		QPixmapDumper dumper;


	protected:
		virtual void paintEvent ( QPaintEvent *paintevent );
		virtual void mousePressEvent ( QMouseEvent * e );
		virtual void keyPressEvent ( QKeyEvent * e );
};

#endif

