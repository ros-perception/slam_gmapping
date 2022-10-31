#include "gmapping/gui/qpixmapdumper.h"
#include <cstdio>
#include <cstring>

QPixmapDumper::QPixmapDumper(std::string p, int c){
	format="PNG";
	prefix=p;
	reset();
	cycles=c;
}

void QPixmapDumper::reset(){
	cycles=0;
	frame=0;
	counter=0;
}

#define filename_bufsize 1024

bool QPixmapDumper::dump(const QPixmap& pixmap){
	bool processed=false;
	if (!(counter%cycles)){
		char buf[filename_bufsize];
		sprintf(buf,"%s-%05d.%s",prefix.c_str(), frame, format.c_str());
		QImage image=pixmap.convertToImage();
		image.save(QString(buf), format.c_str(),0);
		frame ++;
	}
	counter++;
	return processed;
}


