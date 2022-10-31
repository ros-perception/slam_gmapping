
#ifdef MACOSX
// This is to overcome a possible bug in Apple's GCC.
#define isnan(x) (x==FP_NAN)
#endif

/**Just scan match every single particle.
If the scan matching fails, the particle gets a default likelihood.*/
inline void GridSlamProcessor::scanMatch(const double* plainReading){
  // sample a new pose from each scan in the reference
  
  double sumScore=0;
  for (ParticleVector::iterator it=m_particles.begin(); it!=m_particles.end(); it++){
    OrientedPoint corrected;
    double score, l, s;
    score=m_matcher.optimize(corrected, it->map, it->pose, plainReading);
    //    it->pose=corrected;
    if (score>m_minimumScore){
      it->pose=corrected;
    } else {
	if (m_infoStream){
	  m_infoStream << "Scan Matching Failed, using odometry. Likelihood=" << l <<std::endl;
	  m_infoStream << "lp:" << m_lastPartPose.x << " "  << m_lastPartPose.y << " "<< m_lastPartPose.theta <<std::endl;
	  m_infoStream << "op:" << m_odoPose.x << " " << m_odoPose.y << " "<< m_odoPose.theta <<std::endl;
	}
    }

    m_matcher.likelihoodAndScore(s, l, it->map, it->pose, plainReading);
    sumScore+=score;
    it->weight+=l;
    it->weightSum+=l;

    //set up the selective copy of the active area
    //by detaching the areas that will be updated
    m_matcher.invalidateActiveArea();
    m_matcher.computeActiveArea(it->map, it->pose, plainReading);
  }
  if (m_infoStream)
    m_infoStream << "Average Scan Matching Score=" << sumScore/m_particles.size() << std::endl;	
}

inline void GridSlamProcessor::normalize(){
  //normalize the log m_weights
  double gain=1./(m_obsSigmaGain*m_particles.size());
  double lmax= -std::numeric_limits<double>::max();
  for (ParticleVector::iterator it=m_particles.begin(); it!=m_particles.end(); it++){
    lmax=it->weight>lmax?it->weight:lmax;
  }
  //cout << "!!!!!!!!!!! maxwaight= "<< lmax << endl;
  
  m_weights.clear();
  double wcum=0;
  m_neff=0;
  for (std::vector<Particle>::iterator it=m_particles.begin(); it!=m_particles.end(); it++){
    m_weights.push_back(exp(gain*(it->weight-lmax)));
    wcum+=m_weights.back();
    //cout << "l=" << it->weight<< endl;
  }
  
  m_neff=0;
  for (std::vector<double>::iterator it=m_weights.begin(); it!=m_weights.end(); it++){
    *it=*it/wcum;
    double w=*it;
    m_neff+=w*w;
  }
  m_neff=1./m_neff;
  
}

inline bool GridSlamProcessor::resample(const double* plainReading, int adaptSize, const RangeReading* reading){
  
  bool hasResampled = false;
  
  TNodeVector oldGeneration;
  for (unsigned int i=0; i<m_particles.size(); i++){
    oldGeneration.push_back(m_particles[i].node);
  }
  
  if (m_neff<m_resampleThreshold*m_particles.size()){		
    
    if (m_infoStream)
      m_infoStream  << "*************RESAMPLE***************" << std::endl;
    
    uniform_resampler<double, double> resampler;
    m_indexes=resampler.resampleIndexes(m_weights, adaptSize);
    
    if (m_outputStream.is_open()){
      m_outputStream << "RESAMPLE "<< m_indexes.size() << " ";
      for (std::vector<unsigned int>::const_iterator it=m_indexes.begin(); it!=m_indexes.end(); it++){
	m_outputStream << *it <<  " ";
      }
      m_outputStream << std::endl;
    }
    
    onResampleUpdate();
    //BEGIN: BUILDING TREE
    ParticleVector temp;
    unsigned int j=0;
    std::vector<unsigned int> deletedParticles;  		//this is for deleteing the particles which have been resampled away.
    
    //		cerr << "Existing Nodes:" ;
    for (unsigned int i=0; i<m_indexes.size(); i++){
      //			cerr << " " << m_indexes[i];
      while(j<m_indexes[i]){
	deletedParticles.push_back(j);
	j++;
			}
      if (j==m_indexes[i])
	j++;
      Particle & p=m_particles[m_indexes[i]];
      TNode* node=0;
      TNode* oldNode=oldGeneration[m_indexes[i]];
      //			cerr << i << "->" << m_indexes[i] << "B("<<oldNode->childs <<") ";
      node=new	TNode(p.pose, 0, oldNode, 0);
      //node->reading=0;
      node->reading=reading;
      //			cerr << "A("<<node->parent->childs <<") " <<endl;
      
      temp.push_back(p);
      temp.back().node=node;
      temp.back().previousIndex=m_indexes[i];
    }
    while(j<m_indexes.size()){
      deletedParticles.push_back(j);
      j++;
    }
    //		cerr << endl;
    std::cerr <<  "Deleting Nodes:";
    for (unsigned int i=0; i<deletedParticles.size(); i++){
      std::cerr <<" " << deletedParticles[i];
      delete m_particles[deletedParticles[i]].node;
      m_particles[deletedParticles[i]].node=0;
    }
    std::cerr  << " Done" <<std::endl;
    
    //END: BUILDING TREE
    std::cerr << "Deleting old particles..." ;
    m_particles.clear();
    std::cerr << "Done" << std::endl;
    std::cerr << "Copying Particles and  Registering  scans...";
    for (ParticleVector::iterator it=temp.begin(); it!=temp.end(); it++){
      it->setWeight(0);
      m_matcher.invalidateActiveArea();
      m_matcher.registerScan(it->map, it->pose, plainReading);
      m_particles.push_back(*it);
    }
    std::cerr  << " Done" <<std::endl;
    hasResampled = true;
  } else {
    int index=0;
    std::cerr << "Registering Scans:";
    TNodeVector::iterator node_it=oldGeneration.begin();
    for (ParticleVector::iterator it=m_particles.begin(); it!=m_particles.end(); it++){
      //create a new node in the particle tree and add it to the old tree
      //BEGIN: BUILDING TREE  
      TNode* node=0;
      node=new TNode(it->pose, 0.0, *node_it, 0);
      
      //node->reading=0;
      node->reading=reading;
      it->node=node;

      //END: BUILDING TREE
      m_matcher.invalidateActiveArea();
      m_matcher.registerScan(it->map, it->pose, plainReading);
      it->previousIndex=index;
      index++;
      node_it++;
      
    }
    std::cerr  << "Done" <<std::endl;
    
  }
  //END: BUILDING TREE
  
  return hasResampled;
}
