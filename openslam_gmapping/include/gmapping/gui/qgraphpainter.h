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


#ifndef QGRAPHPAINTER_H
#define QGRAPHPAINTER_H

#include <qpainter.h>
#include <qpixmap.h>
#include <qstring.h>
#include <qwidget.h>
#include <qwmatrix.h>
#include <deque>
#include <gmapping/utils/gvalues.h>

typedef std::deque<double> DoubleDeque;

class QGraphPainter :  public QWidget{
	Q_OBJECT
	public:
		QGraphPainter( QWidget * parent = 0, const char * name = 0, WFlags f = 0);
		virtual ~QGraphPainter();
	public slots:
		void clear();
		void valueAdded(double);
		void valueAdded(double, double, double);
		void setYReference(double y);
		void disableYReference();
		void setRange(double min, double max);
		void start(int period);
		void setTitle(const char* title);
		void setAutoscale(bool a);
		bool getAutoscale() const;
	protected:
		virtual void timerEvent(QTimerEvent * te);
		virtual void resizeEvent(QResizeEvent *);
		double min, max, reference;
		DoubleDeque values;
		bool autoscale;
		bool m_useYReference;
		int timer;
		virtual void paintEvent ( QPaintEvent *paintevent );
		QPixmap * m_pixmap;
		QString title;
};

#endif

