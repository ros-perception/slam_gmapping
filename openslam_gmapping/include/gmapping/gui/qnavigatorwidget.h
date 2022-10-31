#ifndef _QNAVIGATOR_WIDGET_H
#define _QNAVIGATOR_WIDGET_H

#include "gmapping/gui/qmappainter.h"
#include "gmapping/gui/qpixmapdumper.h"
#include <gmapping/utils/point.h>
#include <list>

class QNavigatorWidget :  public QMapPainter{
	public:
		QNavigatorWidget( QWidget * parent = 0, const char * name = 0, WFlags f = 0);
		virtual ~QNavigatorWidget();
		std::list<GMapping::IntPoint > trajectoryPoints;
		bool repositionRobot;
		GMapping::IntPoint robotPose;
		double robotHeading;
		bool confirmLocalization;
		bool enableMotion;
		bool startWalker;
		bool startGlobalLocalization;
		bool trajectorySent;
		bool goHome;
		bool wantsQuit;
		bool writeImages;
		QPixmapDumper dumper;
		bool drawRobot;

	protected:
		virtual void paintEvent ( QPaintEvent *paintevent );
		virtual void mousePressEvent ( QMouseEvent * e );
		virtual void keyPressEvent ( QKeyEvent * e );
};

#endif

