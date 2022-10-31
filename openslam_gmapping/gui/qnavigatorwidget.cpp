#include "gmapping/gui/qnavigatorwidget.h"
#include <stdio.h>
using namespace GMapping;


QNavigatorWidget::QNavigatorWidget( QWidget * parent, const char * name, WFlags f)
: QMapPainter(parent, name, f), dumper("navigator", 1){
	robotPose=IntPoint(0,0);
	robotHeading=0;
	confirmLocalization=false;
	repositionRobot=false;
	startWalker=false;
	enableMotion=false;
	goHome=false;
	trajectorySent=false;
	writeImages=false;
	drawRobot=true;
	wantsQuit=false;
}

QNavigatorWidget::~QNavigatorWidget(){}


void QNavigatorWidget::mousePressEvent ( QMouseEvent * e ){
	QPoint p=e->pos();
	int mx=p.x();
	int my=height()-p.y();
	if (!(e->state()&Qt::ShiftButton) && e->button()==Qt::LeftButton) {
		if (trajectorySent)
			trajectoryPoints.clear();
		e->accept();
		IntPoint p=IntPoint(mx, my);
		trajectoryPoints.push_back(p);
		trajectorySent=false;
	} 
	if (e->state()&Qt::ControlButton && e->button()==Qt::LeftButton){
		e->accept();
		robotPose=IntPoint(mx, my);
		repositionRobot=true;
		confirmLocalization=true;
	}
	if (e->state()&Qt::ControlButton && e->button()==Qt::RightButton){
		e->accept();
		IntPoint p(mx, my);
		p=p-robotPose;
		robotHeading=atan2(p.y, p.x);
		repositionRobot=true;
		confirmLocalization=true;
	}
}

void QNavigatorWidget::keyPressEvent ( QKeyEvent * e ){
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
		startGlobalLocalization=true;
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
		confirmLocalization=true;
		
	}
	if (e->key()==Qt::Key_Q){
		e->accept();
		wantsQuit=true;
		
	}
	if (e->key()==Qt::Key_D){
		e->accept();
		drawRobot=!drawRobot;;
		
	}
}

void QNavigatorWidget::paintEvent ( QPaintEvent * ){
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
