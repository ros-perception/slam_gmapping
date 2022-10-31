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


#include <cstdlib>
#include <iostream>
#include "gmapping/configfile/configfile.h"

using namespace std;
using namespace GMapping;

int main(int argc, char** argv) {

  if (argc != 2) {
    cerr << "Usage:  " << argv[0] << " [initifle]" << endl;
    exit(0);
  }
  
  ConfigFile cfg;
  cfg.read(argv[argc-1]);

  cout << "-- values from configfile --" << endl;
  cfg.dumpValues(cout);

  cout << "-- adding a value --" << endl;
  cfg.value("unkown","unkown",std::string("the new value!"));





  cout << "-- values from configfile & added values --" << endl;
  cfg.dumpValues(cout);

  if ( ((std::string) cfg.value("unkown","unkown",std::string("the new value!"))) != std::string("the new value!"))
    cerr << "strange error, check strings" << endl;

  return 0;
}
