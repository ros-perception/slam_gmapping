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


#ifndef COMMANDLINE_H
#define COMMANDLINE_H

	
#define parseFlag(name,value)\
if (!strcmp(argv[c],name)){\
	value=true;\
	cout << name << " on"<< endl;\
	recognized=true;\
}\

#define parseString(name,value)\
if (!strcmp(argv[c],name) && c<argc-1){\
	c++;\
	value=argv[c];\
	cout << name << "=" << value << endl;\
	recognized=true;\
}\


#define parseDouble(name,value)\
if (!strcmp(argv[c],name) && c<argc-1){\
	c++;\
	value=atof(argv[c]);\
	cout << name << "=" << value << endl;\
	recognized=true;\
}\

#define parseInt(name,value)\
if (!strcmp(argv[c],name) && c<argc-1){\
	c++;\
	value=atoi(argv[c]);\
	cout << name << "=" << value << endl;\
	recognized=true;\
}\

#define CMD_PARSE_BEGIN(i, count)\
{\
	int c=i;\
	while (c<count){\
		bool recognized=false;
	
#define CMD_PARSE_END\
		if (!recognized)\
			cout << "COMMAND LINE: parameter " << argv[c] << " not recognized" << endl;\
		c++;\
	}\
}

#define CMD_PARSE_BEGIN_SILENT(i, count)\
{\
	int c=i;\
	while (c<count){\
		bool recognized=false;
	
#define CMD_PARSE_END_SILENT\
		c++;\
	}\
}

	
#define parseFlagSilent(name,value)\
if (!strcmp(argv[c],name)){\
	value=true;\
	recognized=true;\
}\

#define parseStringSilent(name,value)\
if (!strcmp(argv[c],name) && c<argc-1){\
	c++;\
	value=argv[c];\
	recognized=true;\
}\


#define parseDoubleSilent(name,value)\
if (!strcmp(argv[c],name) && c<argc-1){\
	c++;\
	value=atof(argv[c]);\
	recognized=true;\
}\

#define parseIntSilent(name,value)\
if (!strcmp(argv[c],name) && c<argc-1){\
	c++;\
	value=atoi(argv[c]);\
	recognized=true;\
}\


#endif

