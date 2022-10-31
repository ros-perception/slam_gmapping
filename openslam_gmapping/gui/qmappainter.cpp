#include "gmapping/gui/qmappainter.h"
#include "moc_qmappainter.cpp"

QMapPainter::QMapPainter( QWidget * parent, const char * name, WFlags f):
	QWidget(parent, name, f|WRepaintNoErase|WResizeNoErase){
	m_pixmap=new QPixmap(size());
	m_pixmap->fill(Qt::white);
}

void QMapPainter::resizeEvent(QResizeEvent * sizeev){
	m_pixmap->resize(sizeev->size());
}

QMapPainter::~QMapPainter(){
	delete m_pixmap;
}


void QMapPainter::timerEvent(QTimerEvent * te) {
        if (te->timerId()==timer) 
		update();
}

void QMapPainter::start(int period){
	timer=startTimer(period);
}


void QMapPainter::paintEvent ( QPaintEvent * ){
	bitBlt(this,0,0,m_pixmap,0,0,m_pixmap->width(),m_pixmap->height(),CopyROP);
}

