#include "gmapping/gui/qslamandnavwidget.h"
#include <stdio.h>
using namespace GMapping;


QSLAMandNavWidget::QSLAMandNavWidget( QWidget * parent, const char * name, WFlags f)
: QMapPainter(parent, name, f), dumper("slamandnav", 1){
	robotPose=IntPoint(0,0);
	robotHeading=0;
	slamRestart=false;
	slamFinished=false;
	enableMotion=false;
	startWalker=false;
	trajectorySent=false;
	goHome=false;
	wantsQuit=false;
	printHelp=false;
	saveGoalPoints=false;
	writeImages=false;
	drawRobot=true;
}

QSLAMandNavWidget::~QSLAMandNavWidget(){}


void QSLAMandNavWidget::mousePressEvent ( QMouseEvent * e ){
	QPoint p=e->pos();
	int mx=p.x();
	int my=height()-p.y();
	if ( e->state()&Qt::ShiftButton && e->button()==Qt::LeftButton) {
		if (trajectorySent)
			trajectoryPoints.clear();
		e->accept();
		IntPoint p=IntPoint(mx, my);
		trajectoryPoints.push_back(p);
		trajectorySent=false;
	} 
}

void QSLAMandNavWidget::keyPressEvent ( QKeyEvent * e ){
	if (e->key()==Qt::Key_Delete){
		e->accept();
		if (!trajectoryPoints.empty())
			trajectoryPoints.pop_back();
	}
	if (e->key()==Qt::Key_S){
		e->accept();
		enableMotion=!enableMotion;
	}
	if (e->key()==Qt::Key_W){
		e->accept();
		startWalker=!startWalker;
	}
	if (e->key()==Qt::Key_G){
		e->accept();
		slamRestart=true;
	}
	if (e->key()==Qt::Key_T){
		e->accept();
		trajectorySent=true;
	}
	if (e->key()==Qt::Key_R){
		e->accept();
		goHome=true;
	}	
	if (e->key()==Qt::Key_C){
		e->accept();
		slamFinished=true;
		
	}
	if (e->key()==Qt::Key_Q){
		e->accept();
		wantsQuit=true;
		
	}
	if (e->key()==Qt::Key_H){
		e->accept();
		printHelp=true;
		//BABSI
		//insert the help here
	}
	if (e->key()==Qt::Key_Y){
		e->accept();
		saveGoalPoints=true;
		//BABSI
		//insert the help here
	}
	if (e->key()==Qt::Key_D){
		e->accept();
		drawRobot=!drawRobot;
		//BABSI
		//insert the help here
	}
}

void QSLAMandNavWidget::paintEvent ( QPaintEvent * ){
	QPixmap pixmap(*m_pixmap);
	QPainter painter(&pixmap);
	if (trajectorySent)
	painter.setPen(Qt::red);
	bool first=true;
	int oldx=0, oldy=0;
	//paint the path
	for (std::list<IntPoint>::const_iterator it=trajectoryPoints.begin(); it!=trajectoryPoints.end(); it++){
		int x=it->x;
		int y=height()-it->y;
		if (! first)
			painter.drawLine(oldx, oldy, x,y);
		oldx=x;
		oldy=y;
		first=false;
	}

	//paint the robot
	if (drawRobot){
		painter.setPen(Qt::black);
		int rx=robotPose.x;
		int ry=height()-robotPose.y;
		int robotSize=6;
		painter.drawLine(rx, ry, 
				rx+(int)(robotSize*cos(robotHeading)), ry-(int)(robotSize*sin(robotHeading)));
		painter.drawEllipse(rx-robotSize, ry-robotSize, 2*robotSize, 2*robotSize);
	}
	if (writeImages){
		dumper.dump(pixmap);
	}
	bitBlt(this,0,0,&pixmap,0,0,pixmap.width(),pixmap.height(),CopyROP);
}
