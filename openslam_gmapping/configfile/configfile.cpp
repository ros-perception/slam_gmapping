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
#include "gmapping/configfile/configfile.h"

#include <fstream>
#include <iostream>

#include <string>
#include <sstream>
#include <iostream>
#include <ctype.h>

namespace GMapping{
using namespace std;

AutoVal::AutoVal(const std::string& value) {
  m_value=value;
}

AutoVal::AutoVal(const char* c) {
  m_value=c;
}

AutoVal::AutoVal(double d) {
  std::stringstream s;
  s<<d;
  m_value=s.str();
}

AutoVal::AutoVal(bool d) {
  std::stringstream s;
  if(d)
    s << "on";
  else 
    s << "off";
  m_value=s.str();
}


AutoVal::AutoVal(int i) {
  std::stringstream s;
  s<<i;
  m_value=s.str();
}

AutoVal::AutoVal(unsigned int i) {
  std::stringstream s;
  s<<i;
  m_value=s.str();
}

AutoVal::AutoVal(const AutoVal& other) {
  m_value=other.m_value;
}

AutoVal& AutoVal::operator=(const AutoVal& other) {
  m_value=other.m_value;
  return *this;
}

AutoVal& AutoVal::operator=(double d) {
  std::stringstream s;
  s << d;
  m_value = s.str();
  return *this;
}

AutoVal& AutoVal::operator=(bool d) {
  std::stringstream s;
  if(d)
    s << "on";
  else 
    s << "off";
  m_value = s.str();
  return *this;
}


AutoVal& AutoVal::operator=(int i) {
  std::stringstream s;
  s << i;
  m_value = s.str();
  return *this;
}

AutoVal& AutoVal::operator=(unsigned int i) {
  std::stringstream s;
  s << i;
  m_value = s.str();
  return *this;
}


AutoVal& AutoVal::operator=(const std::string& s) {
  m_value=s;
  return *this;
}

AutoVal::operator std::string() const {
  return m_value;
}

AutoVal::operator double() const {
  return atof(m_value.c_str());
}

AutoVal::operator int() const {
  return atoi(m_value.c_str());
}

AutoVal::operator unsigned int() const {
  return (unsigned int) atoi(m_value.c_str());
}

AutoVal::operator bool() const {
  // stupid c++ does not allow compareNoCase...
  if (toLower(m_value)=="on" || atoi(m_value.c_str()) == 1)
    return true;
  return false;
}

std::string AutoVal::toLower(const std::string& source) const {
  std::string result(source);
  char c='\0';
  for (unsigned int i=0; i<result.length(); i++) {
    c = result[i];
    c = ::tolower(c);
    result[i] = c;
  }
  return result;
}


//////////////////////////////////////////////////////////

ConfigFile::ConfigFile() {
}

ConfigFile::ConfigFile(const std::string& configFile) {
  read(configFile);
}

ConfigFile::ConfigFile(const char* configFile) {
  read(configFile);
}

bool ConfigFile::read(const char* configFile) {
  return read(std::string(configFile));
}

bool ConfigFile::read(const std::string& configFile) {
  std::ifstream file(configFile.c_str());

  if (!file || file.eof())
    return false;

  std::string line;
  std::string name;
  std::string val;
  std::string inSection;
  while (std::getline(file,line)) {

    if (! line.length()) continue;
    // cute the comments
    if (line[0] == '#') continue;
    line = truncate(line,"#");
    line = trim(line);
    if (! line.length()) continue;
    
    if (line[0] == '[') {
      inSection=trim(line.substr(1,line.find(']')-1));
      continue;
    }
    
    istringstream lineStream(line);
    lineStream >> name;
    lineStream >> val;
    insertValue(inSection,name,val);
  }
  return true;
}

void ConfigFile::insertValue(const std::string& section, const std::string& entry, const std::string& thevalue)  {
  m_content[toLower(section)+'/'+toLower(entry)]=AutoVal(thevalue);
}

const AutoVal& ConfigFile::value(const std::string& section, const std::string& entry) const {

  std::map<std::string,AutoVal>::const_iterator ci = 
    m_content.find(toLower(section) + '/' + toLower(entry));
  if (ci == m_content.end()) throw "entry does not exist";

  return ci->second;
}

const AutoVal& ConfigFile::value(const std::string& section, const std::string& entry, double def) {
  try {
    return value(section, entry);
  } catch(const char *) {
    return m_content.insert(std::make_pair(section+'/'+entry, AutoVal(def))).first->second;
  }
}

const AutoVal& ConfigFile::value(const std::string& section, const std::string& entry, bool def) {
  try {
    return value(section, entry);
  } catch(const char *) {
    return m_content.insert(std::make_pair(section+'/'+entry, AutoVal(def))).first->second;
  }
}

const AutoVal& ConfigFile::value(const std::string& section, const std::string& entry, int def) {
  try {
    return value(section, entry);
  } catch(const char *) {
    return m_content.insert(std::make_pair(section+'/'+entry, AutoVal(def))).first->second;
  }
}

const AutoVal& ConfigFile::value(const std::string& section, const std::string& entry, unsigned int def) {
  try {
    return value(section, entry);
  } catch(const char *) {
    return m_content.insert(std::make_pair(section+'/'+entry, AutoVal(def))).first->second;
  }
}

const AutoVal& ConfigFile::value(const std::string& section, const std::string& entry, const std::string& def) {
  try {
    return value(section, entry);
  } catch(const char *) {
    return m_content.insert(std::make_pair(section+'/'+entry, AutoVal(def))).first->second;
  }
}

const AutoVal& ConfigFile::value(const std::string& section, const std::string& entry, const char* def) {
  try {
    return value(section, entry);
  } catch(const char *) {
    return m_content.insert(std::make_pair(section+'/'+entry, AutoVal(def))).first->second;
  }
}

void ConfigFile::dumpValues(std::ostream& out) {

  for (std::map<std::string,AutoVal>::const_iterator it = m_content.begin();
       it != m_content.end(); it++) {
    out << (std::string) it->first << " " << (std::string)it->second << std::endl;
  }
}


std::string ConfigFile::trim(const std::string& source, char const* delims) const {
  std::string result(source);
  std::string::size_type index = result.find_last_not_of(delims);
  if(index != std::string::npos)
    result.erase(++index);

  index = result.find_first_not_of(delims);
  if(index != std::string::npos)
    result.erase(0, index);
  else
    result.erase();
  return result;
}

std::string ConfigFile::truncate(const std::string& source, const char* atChar) const {
  std::string::size_type index = source.find_first_of(atChar);

  if(index == 0) 
    return "";
  else if(index == std::string::npos) {
    return source;
  }
  return 
    source.substr(0,index);
}

std::string ConfigFile::toLower(const std::string& source) const {
  std::string result(source);
  char c='\0';
  for (unsigned int i=0; i<result.length(); i++) {
    c = result[i];
    c = ::tolower(c);
    result[i] = c;
  }
  return result;
}
};
