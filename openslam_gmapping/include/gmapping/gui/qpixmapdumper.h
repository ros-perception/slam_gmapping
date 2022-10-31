#ifndef _QPIXMAPDUMPER_H_
#define _QPIXMAPDUMPER_H_
#include <qpixmap.h>
#include <qimage.h>
#include <string>


struct QPixmapDumper{
	QPixmapDumper(std::string prefix, int cycles);
	void reset();
	std::string prefix;
	std::string format;	
	bool dump(const QPixmap& pixmap);
	int counter;
	int cycles;
	int frame;
};

#endif
