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


#ifndef CONFIGFILE_H
#define CONFIGFILE_H

#include <iostream>
#include <string>
#include <map>
#include <gmapping/configfile/configfile_export.h>

namespace GMapping{

class CONFIGFILE_EXPORT AutoVal {
public:
  AutoVal() {};
  explicit AutoVal(const std::string&);
  explicit AutoVal(double);
  explicit AutoVal(int);
  explicit AutoVal(unsigned int);
  explicit AutoVal(bool);
  explicit AutoVal(const char*);

  AutoVal(const AutoVal&);
  AutoVal& operator=(const AutoVal&);

  AutoVal& operator=(double);
  AutoVal& operator=(int);
  AutoVal& operator=(unsigned int);
  AutoVal& operator=(bool);
  AutoVal& operator=(const std::string&);

public:
  operator std::string() const;
  operator double() const;
  operator int() const;
  operator unsigned int() const;
  operator bool() const;

protected:
  std::string toLower(const std::string& source) const;

private:
  std::string m_value;
};

class CONFIGFILE_EXPORT ConfigFile {
  std::map<std::string,AutoVal> m_content;
  
public:
  ConfigFile();
  ConfigFile(const std::string& configFile);
  ConfigFile(const char* configFile);

  bool read(const std::string& configFile);
  bool read(const char* configFile);
  
 
  const AutoVal& value(const std::string& section, 
		       const std::string& entry) const;

  const AutoVal& value(const std::string& section, 
		       const std::string& entry, 
		       double def);

  const AutoVal& value(const std::string& section, 
		       const std::string& entry, 
		       const char* def);

  const AutoVal& value(const std::string& section,
		       const std::string& entry,
		       bool def);

  const AutoVal& value(const std::string& section, 
		       const std::string& entry, 
		       int def);

  const AutoVal& value(const std::string& section, 
		       const std::string& entry, 
		       unsigned int def);

  const AutoVal& value(const std::string& section, 
		       const std::string& entry, 
		       const std::string& def);
  
  void dumpValues(std::ostream& out);


 protected:
  std::string trim(const std::string& source, char const* delims = " \t\r\n") const;
  std::string truncate(const std::string& source, const char* atChar) const;
  std::string toLower(const std::string& source) const;
  void insertValue(const std::string& section, const std::string& entry, const std::string& thevalue );
};

};

#endif
