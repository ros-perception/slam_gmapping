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
#include "moc_qparticleviewer.cpp"
#include <qimage.h>


using namespace GMapping;

QParticleViewer::QParticleViewer( QWidget * parent, const char * name , WFlags f, GridSlamProcessorThread* thread): QWidget(parent, name, f|WRepaintNoErase|WResizeNoErase){
  viewCenter=Point(0.,0.);
  setMinimumSize(500,500);
  mapscale=10.;
  m_pixmap=new QPixmap(500,500);
  m_pixmap->fill(Qt::white);
  gfs_thread=thread;
  tis=0;
  m_particleSize=0;
  m_refresh=false;
  bestMap=0;
  dragging=false;
  showPaths=0;
  showBestPath=1;
  count=0;
  writeToFile=0;
}

QParticleViewer::~QParticleViewer(){
  if (m_pixmap)
    delete m_pixmap;
}

void QParticleViewer::paintEvent ( QPaintEvent *paintevent ){
  if (! m_pixmap)
    return;
  bitBlt(this,0,0,m_pixmap,0,0,m_pixmap->width(),m_pixmap->height(),CopyROP);
}

void QParticleViewer::mousePressEvent ( QMouseEvent *event ){
  if (event->button()==LeftButton){
    dragging=true;
    draggingPos=event->pos();	
  }
}
void QParticleViewer::mouseMoveEvent ( QMouseEvent *event ){
  if (dragging){
    QPoint delta=event->pos()-draggingPos;
    draggingPos=event->pos();
    viewCenter.x-=delta.x()/mapscale;
    viewCenter.y+=delta.y()/mapscale;
    update();
  }
}

void QParticleViewer::mouseReleaseEvent ( QMouseEvent *event ){
  if (event->button()==LeftButton){
    dragging=false;
  }
}

void QParticleViewer::keyPressEvent ( QKeyEvent* e ){
  switch (e->key()){
  case Qt::Key_B: showBestPath=!showBestPath; break;
  case Qt::Key_P: showPaths=!showPaths; break;
  case Qt::Key_Plus: mapscale *=1.25;  break;
  case Qt::Key_Minus: mapscale/=1.25;  break;
  case Qt::Key_C: viewCenter=bestParticlePose; break;
  default:;
  }
}

		
void QParticleViewer::resizeEvent(QResizeEvent * sizeev){
  if (!m_pixmap)
    return;
  cerr << "QParticleViewer::resizeEvent" <<  sizeev->size().width()<< " " << sizeev->size().height() << endl;
  m_pixmap->resize(sizeev->size());
}

void QParticleViewer::drawParticleMove(const QParticleViewer::OrientedPointVector& oldPose, const QParticleViewer::OrientedPointVector& newPose){
  assert(oldPose.size()==newPose.size());
  QPainter painter(m_pixmap);
  painter.setPen(Qt::red);
  OrientedPointVector::const_iterator nit=newPose.begin();
  for(OrientedPointVector::const_iterator it=oldPose.begin(); it!=oldPose.end(); it++, nit++){
    IntPoint p0=map2pic(*it);
    IntPoint p1=map2pic(*nit);
    painter.drawLine( 
		     (int)(p0.x), (int)(p0.y), (int)(p1.x), (int)(p1.y)
		     );
  }
}

void QParticleViewer::drawFromFile(){
  if(! tis)
    return;
  if (tis->atEnd())
    return;	
  QTextIStream& is=*tis;
	
  string line=is.readLine();
  istringstream lineStream(line);
  string recordType;
  lineStream >> recordType;
  if (recordType=="LASER_READING"){
    //do nothing with the laser
    cout << "l" << flush;
  }
  if (recordType=="ODO_UPDATE"){
    //just move the particles
    if (m_particleSize)
      m_refresh=true;
    m_oldPose=m_newPose;
    m_newPose.clear();
    unsigned int size;
    lineStream >> size;
    if (!m_particleSize)
      m_particleSize=size;
    assert(m_particleSize==size);
    for (unsigned int i=0; i< size; i++){
      OrientedPoint p;
      double w;
      lineStream >> p.x;
      lineStream >> p.y;
      lineStream >> p.theta;
      lineStream >> w;
      m_newPose.push_back(p);
    }
    cout << "o" << flush;
  }
  if (recordType=="SM_UPDATE"){
    if (m_particleSize)
      m_refresh=true;
    m_oldPose=m_newPose;
    m_newPose.clear();
    unsigned int size;
    lineStream >> size;
    if (!m_particleSize)
      m_particleSize=size;
    assert(m_particleSize==size);
    for (unsigned int i=0; i< size; i++){
      OrientedPoint p;
      double w;
      lineStream >> p.x;
      lineStream >> p.y;
      lineStream >> p.theta;
      lineStream >> w;
      m_newPose.push_back(p);
    }
    cout << "u" << flush;
  }
  if (recordType=="RESAMPLE"){
    unsigned int size;
    lineStream >> size;
    if (!m_particleSize)
      m_particleSize=size;
    assert(m_particleSize==size);
    OrientedPointVector temp(size);
    for (unsigned int i=0; i< size; i++){
      unsigned int ind;
      lineStream >> ind;
      temp[i]=m_newPose[ind];
    }
    m_newPose=temp;
    cout << "r" << flush;
  }
  if (m_refresh){
    drawParticleMove(m_oldPose, m_newPose);
    m_refresh=false;
  }
}

void QParticleViewer::drawMap(const ScanMatcherMap& map){
  //cout << "Map received" << map.getMapSizeX() << " " << map.getMapSizeY() << endl;
  QPainter painter(m_pixmap);
  painter.setPen(Qt::black);
  m_pixmap->fill(QColor(200,200,255));
  unsigned int count=0;
	
  Point wmin=Point(pic2map(IntPoint(-m_pixmap->width()/2,m_pixmap->height()/2)));
  Point wmax=Point(pic2map(IntPoint(m_pixmap->width()/2,-m_pixmap->height()/2)));
  IntPoint imin=map.world2map(wmin);
  IntPoint imax=map.world2map(wmax);
  /*	cout << __func__ << endl;
	cout << " viewCenter=" << viewCenter.x << "," << viewCenter.y <<   endl;	
	cout << " wmin=" << wmin.x << "," << wmin.y <<  " wmax=" << wmax.x << "," << wmax.y << endl;	
	cout << " imin=" << imin.x << "," << imin.y <<  " imax=" << imax.x << "," << imax.y << endl;
	cout << " mapSize=" << map.getMapSizeX() << "," << map.getMapSizeY() << endl;*/
  for(int x=0; x<m_pixmap->width(); x++)
    for(int y=0; y<m_pixmap->height(); y++){
      //IntPoint ip=IntPoint(x,y)+imin;
      //Point p=map.map2world(ip);
      Point p=pic2map(IntPoint(x-m_pixmap->width()/2,
			       y-m_pixmap->height()/2));

      //if (map.storage().isInside(map.world2map(p))){
      double v=map.cell(p);
      if (v>=0){
	int grayValue=255-(int)(255.*v);
	painter.setPen(QColor(grayValue, grayValue, grayValue));
	painter.drawPoint(x,y);
	count++;
      }
    }
}


void QParticleViewer::drawFromMemory(){
  if (! gfs_thread)
    return;
  m_pixmap->fill(Qt::white);
  GridSlamProcessorThread::EventDeque events=gfs_thread->getEvents();
  for (GridSlamProcessorThread::EventDeque::const_iterator it=events.begin(); it!=events.end();it++){
    GridSlamProcessorThread::MapEvent* mapEvent= dynamic_cast<GridSlamProcessorThread::MapEvent*>(*it);
    if (mapEvent){
      //cout << "Map: bestIdx=" << mapEvent->index <<endl;
      if (bestMap)
	delete bestMap;
      else {
				
      }
      bestMap=mapEvent->pmap;
      mapEvent->pmap=0;
      bestParticlePose=mapEvent->pose;
      delete mapEvent;
    }else{
      GridSlamProcessorThread::DoneEvent* doneEvent= dynamic_cast<GridSlamProcessorThread::DoneEvent*>(*it);
      if (doneEvent){
	gfs_thread->stop();
	delete doneEvent;
      } else
	history.push_back(*it);
    }	
			
  }
  if (bestMap)
    drawMap(*bestMap);
	
  unsigned int particleSize=0;
  std::vector<OrientedPoint> oldPose, newPose;
  vector<unsigned int> indexes;
	
  GridSlamProcessorThread::EventDeque::reverse_iterator it=history.rbegin();
  while (!particleSize && it!=history.rend()){
    GridSlamProcessorThread::ParticleMoveEvent* move= dynamic_cast<GridSlamProcessorThread::ParticleMoveEvent*>(*it);
    GridSlamProcessorThread::ResampleEvent* resample= dynamic_cast<GridSlamProcessorThread::ResampleEvent*>(*it);
    if (move)
      particleSize=move->hypotheses.size();
    if (resample)
      particleSize=resample->indexes.size();
    it++;
  }
	
  //check for the best index
  double wmax=-1e2000;
  unsigned int bestIdx=0;
  bool emitted=false;
  for (unsigned int i=0; i<particleSize; i++){
    unsigned int currentIndex=i;
    bool done=false;
    for(GridSlamProcessorThread::EventDeque::reverse_iterator it=history.rbegin(); it!=history.rend()&& !done; it++){
      GridSlamProcessorThread::ParticleMoveEvent* move= dynamic_cast<GridSlamProcessorThread::ParticleMoveEvent*>(*it);
      if (move && move->scanmatched){
	double cw=move->weightSums[currentIndex];
	if (cw>wmax){
	  wmax=cw;
	  bestIdx=currentIndex;
	} 
	done=true;
	if (! emitted){
	  emit neffChanged(move->neff/particleSize);
	  emitted=true;
	}
      }
      GridSlamProcessorThread::ResampleEvent* resample= dynamic_cast<GridSlamProcessorThread::ResampleEvent*>(*it);
      if (resample){
	currentIndex=resample->indexes[currentIndex];
      }
    }
  }
  //cout << "bestIdx=" << bestIdx << endl;
  QPainter painter(m_pixmap);
	
  for (unsigned int i=0; i<particleSize+1; i++){
    painter.setPen(Qt::yellow);
    unsigned int currentIndex=i;
    if (i==particleSize && showBestPath){
      currentIndex=bestIdx;
      painter.setPen(Qt::red);
    }
    bool first=true;
    OrientedPoint pnew(0,0,0);
    for(GridSlamProcessorThread::EventDeque::reverse_iterator it=history.rbegin(); it!=history.rend(); it++){
      GridSlamProcessorThread::ParticleMoveEvent* move= dynamic_cast<GridSlamProcessorThread::ParticleMoveEvent*>(*it);
      if (move){
	OrientedPoint pold=move->hypotheses[currentIndex];
	IntPoint p0=map2pic(pold)+IntPoint(m_pixmap->width()/2,m_pixmap->height()/2);
	IntPoint p1=map2pic(pnew)+IntPoint(m_pixmap->width()/2,m_pixmap->height()/2);;
	if (first){
	  painter.drawPoint(p0.x, p0.y);
	} else {
	  painter.drawLine(p0.x, p0.y, p1.x, p1.y);
	}
	first=false;
	if (!(showPaths || showBestPath&&i==particleSize))
	  break;
	pnew=pold;
      }
      GridSlamProcessorThread::ResampleEvent* resample= dynamic_cast<GridSlamProcessorThread::ResampleEvent*>(*it);
      if (resample && ! first){
	currentIndex=resample->indexes[currentIndex];
      }
    }
  }
  if (writeToFile && bestMap){
    if (! (count%writeToFile) ){
      char name[100];
      sprintf(name,"dump-%05d.png", count/writeToFile);
      cout << " Writing " << name <<" ..." << flush;
      QImage image=m_pixmap->convertToImage();
      bool rv=image.save(name,"PNG");
      if (rv)
	cout << " Done";
      else
	cout << " ERROR";
      cout << endl;
    }
    count++;
  }
}

void QParticleViewer::timerEvent(QTimerEvent * te) {
  if (te->timerId()==timer) {
    if ( tis)
      drawFromFile();
    else{
      drawFromMemory();
      update();
    }
  }
}


void QParticleViewer::start(int period){
  timer=startTimer(period);
}

void QParticleViewer::refreshParameters(){
  //scanmatcher
  matchingParameters.maxrange=gfs_thread->getlaserMaxRange();
  matchingParameters.urange=gfs_thread->getusableRange();
  matchingParameters.ssigma=gfs_thread->getgaussianSigma();
  matchingParameters.sreg=gfs_thread->getregScore();
  matchingParameters.scrit=gfs_thread->getcritScore();
  matchingParameters.ksize=gfs_thread->getkernelSize();
  matchingParameters.lstep=gfs_thread->getoptLinearDelta();
  matchingParameters.astep=gfs_thread->getoptAngularDelta();
  matchingParameters.iterations=gfs_thread->getoptRecursiveIterations();

  //start
  startParameters.srr=gfs_thread->getsrr();
  startParameters.stt=gfs_thread->getstt();
  startParameters.str=gfs_thread->getstr();
  startParameters.srt=gfs_thread->getsrt();
	
  startParameters.xmin=gfs_thread->getxmin();
  startParameters.ymin=gfs_thread->getymin();
  startParameters.xmax=gfs_thread->getxmax();
  startParameters.ymax=gfs_thread->getymax();
  startParameters.delta=gfs_thread->getdelta();
	
  startParameters.particles=gfs_thread->getParticles().size();
  startParameters.resampleThreshold=gfs_thread->getresampleThreshold();
  startParameters.outFileName=0;
}

void QParticleViewer::start(){
  gfs_thread->setMatchingParameters(
				    matchingParameters.urange, 
				    matchingParameters.maxrange, 
				    matchingParameters.ssigma, 
				    matchingParameters.ksize, 
				    matchingParameters.lstep, 
				    matchingParameters.astep, 
				    matchingParameters.iterations, 
				    startParameters.lsigma,
				    startParameters.lgain,
				    startParameters.lskip);
  gfs_thread->setMotionModelParameters(
				       startParameters.srr,
				       startParameters.srt,
				       startParameters.srt,
				       startParameters.stt);
  gfs_thread->setUpdateDistances(
				 startParameters.linearUpdate,
				 startParameters.angularUpdate,
				 startParameters.resampleThreshold
				 );
  ((GridSlamProcessor*)(gfs_thread))->init(
					   startParameters.particles,
					   startParameters.xmin, 
					   startParameters.ymin, 
					   startParameters.xmax, 
					   startParameters.ymax, 
					   startParameters.delta, 
					   startParameters.initialPose);
  gfs_thread->start();
}

void QParticleViewer::setMatchingParameters(const QParticleViewer::MatchingParameters& mp){
  matchingParameters=mp;
}

void QParticleViewer::setStartParameters(const QParticleViewer::StartParameters& sp){
  startParameters=sp;
}

void QParticleViewer::stop(){
  gfs_thread->stop();
}

void QParticleViewer::loadFile(const char * fn){
  gfs_thread->loadFiles(fn);
  /*	
    startParameters.initialPose=
    gfs_thread->boundingBox(
    startParameters.xmin, 
    startParameters.ymin, 
    startParameters.xmax,
    startParameters.ymax);
  */	
}
