/*****************************************************************
 *
 * This file is part of the GMAPPING project
 *
 * GMAPPING Copyright (c) 2004 Giorgio Grisetti, 
 * Cyrill Stachniss, and Wolfram Burgard
 *
 * This software is licensed under the 3-Clause BSD License
 * and is copyrighted by Giorgio Grisetti, Cyrill Stachniss, 
 * and Wolfram Burgard.
 * 
 * Further information on this license can be found at:
 * https://opensource.org/licenses/BSD-3-Clause
 * 
 * GMAPPING is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied 
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  
 *
 *****************************************************************/


#include <iostream>
#include "gmapping/gui/qgraphpainter.h"
#include "moc_qgraphpainter.cpp"
using namespace std;

QGraphPainter::QGraphPainter( QWidget * parent, const char * name, WFlags f):
	QWidget(parent, name, f|WRepaintNoErase|WResizeNoErase){
	m_pixmap=new QPixmap(size());
	m_pixmap->fill(Qt::white);
	autoscale=false;
	m_useYReference = false;
}

void QGraphPainter::resizeEvent(QResizeEvent * sizeev){
	m_pixmap->resize(sizeev->size());
}

QGraphPainter::~QGraphPainter(){
	delete m_pixmap;
}

void QGraphPainter::clear(){
	values.clear();
}

void QGraphPainter::valueAdded(double v){
	values.push_back(v);
}

void QGraphPainter::valueAdded(double v, double _min, double _max){
	setRange(_min, _max);
	values.push_back(v);
}

void QGraphPainter::setYReference(double y){
  m_useYReference = true;
        reference=y;
}

void QGraphPainter::disableYReference(){
  m_useYReference = false;
}


void QGraphPainter::setTitle(const char* t){
	title=t;
}

void QGraphPainter::setRange(double _min, double _max){
	min=_min;
	max=_max;
}

void QGraphPainter::setAutoscale(bool a) {
	autoscale=a;
}

bool QGraphPainter::getAutoscale() const {
	return autoscale;
}

void QGraphPainter::timerEvent(QTimerEvent * te) {
        if (te->timerId()==timer) 
		update();
}

void QGraphPainter::start(int period){
	timer=startTimer(period);
}



void QGraphPainter::paintEvent ( QPaintEvent * ){
	m_pixmap->fill(Qt::white);
	QPainter painter(m_pixmap);
	double _min=MAXDOUBLE, _max=-MAXDOUBLE;
	if (autoscale){
		for (unsigned int i=0; i<(unsigned int)width() && i<values.size(); i++){
			_min=_min<values[i]?_min:values[i];
			_max=_max>values[i]?_max:values[i];
		}
	} else {
		_min=min;
		_max=max;
	}
	

	painter.setPen(Qt::black);
	painter.drawRect(0, 0, width(), height());
	const int boundary=2;
	int xoffset=40;
	double scale=((double)height()-2*boundary-2)/(_max-_min);

	if (m_useYReference) {
	  painter.setPen(Qt::green);
	  painter.drawLine(xoffset+boundary/2, height()-(int)(scale*(reference-_min)), 
			   width()-boundary/2, height()-(int)(scale*(reference-_min)));
	}
	painter.setPen(Qt::blue);
	unsigned int start=0;
	if (values.size()>(unsigned int)width()-2*boundary-xoffset)
		start=values.size()-width()+2*boundary+xoffset;
	int oldv=0;
	if ((unsigned int)width()-2*boundary-xoffset>1 && values.size()>1)
	  oldv = (int)(scale*(values[1+start]-_min)) + boundary;
	
	for (unsigned int i=1; i<(unsigned int)width()-2*boundary-xoffset && i<values.size(); i++){
		int v=(int)(scale*(values[i+start]-_min)) + boundary;
		painter.drawLine(i-1+boundary+xoffset, height()-boundary-oldv, 
				 xoffset+i+boundary, height()-boundary-v);
		oldv=v;
	}
	painter.setPen(Qt::black);
	painter.drawText( 3, height()/2, title);
	QFont sansFont( "Helvetica [Cronyx]", 6);	
	painter.setFont(sansFont);
	bitBlt(this,0,0,m_pixmap,0,0,m_pixmap->width(),m_pixmap->height(),CopyROP);
}

