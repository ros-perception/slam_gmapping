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


#include "gmapping/gui/qparticleviewer.h"
#include "gmapping/gui/qgraphpainter.h"
#include <qapplication.h>
#include <qframe.h>
#include <qlabel.h>
#include <qlayout.h>
#include <qvbox.h>
#include <qmainwindow.h>

class GFSMainWindow: public QMainWindow{
public:
  GFSMainWindow(GridSlamProcessorThread* t){
    gsp_thread=t;
    QVBoxLayout* layout=new QVBoxLayout(this);
    pviewer=new QParticleViewer(this,0,0,gsp_thread);
    pviewer->setGeometry(0,0,500,500);
    pviewer->setFocusPolicy(QParticleViewer::ClickFocus);
    layout->addWidget(pviewer);
						
    gpainter=new QGraphPainter(this);
    gpainter->setFixedHeight(100);
    layout->addWidget(gpainter);
    gpainter->setRange(0,1);
    gpainter->setTitle("Neff");
		
    help = new QLabel(QString("+/- - zoom | b - show/hide best path | p - show/hide all paths | c - center robot "),this); 
    help->setMaximumHeight(30);
    layout->addWidget(help);
	
    QObject::connect( pviewer, SIGNAL(neffChanged(double) ), gpainter, SLOT(valueAdded(double)) );
    setTabOrder(pviewer, pviewer);
  }
		
  void start(int c){
    pviewer->start(c);
    gpainter->start(c);
  }

protected:
  GridSlamProcessorThread* gsp_thread;
  QVBoxLayout* layout;
  QParticleViewer* pviewer;
  QGraphPainter* gpainter;
  QLabel* help;
};


int  main (int argc, char ** argv){
  cerr << "GMAPPING copyright 2004 by Giorgio Grisetti, Cyrill Stachniss," << endl ;
  cerr << "and Wolfram Burgard. Published under the 3-Clause BSD License," << endl;
  cerr << "see: https://opensource.org/licenses/BSD-3-Clause" << endl << endl;


  GridSlamProcessorThread* gsp=  new GridSlamProcessorThread;
  if (gsp->init(argc, argv)){
    cerr << "GridFastSlam: Initialization Error!" << endl;
    cerr << "(Did you specified an input file for reading?)" << endl;
    return -1;
  }
  if (gsp->loadFiles()){
    cerr <<"Error reading file!"<< endl;
    return -2;
  }
  cerr <<"File successfully loaded!"<< endl;
  QApplication app(argc, argv);
  GFSMainWindow* mainWin=new GFSMainWindow(gsp);
  app.setMainWidget(mainWin);
  mainWin->show();
  gsp->setEventBufferSize(10000);
  gsp->start();
  mainWin->start(1000);
  return app.exec();
}

