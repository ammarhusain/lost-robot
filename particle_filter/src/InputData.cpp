/** ----------------------------------------------------------------------
 * Copyright 2014 < Ammar Husain (Carnegie Mellon University) > 
 * 
 * @file   InputData.cpp
 * @author Ammar Husain <ahusain@nrec.ri.cmu.edu> 
 * @date   Tue Nov 11 10:52:45 2014
 * 
 * @brief  
 * Implementation of OdometryReading and LaserReading constructors and
 * parsers
 ---------------------------------------------------------------------- */

#include "InputData.h"


/** ----------------------------------------------------------------------
 * 
 * 
 * @date 11/11/2014 
 ---------------------------------------------------------------------- */
OdometryReading::OdometryReading()
    : x(0), y(0), theta(0), t(0) {
    /// nothing left to do
}


/** ----------------------------------------------------------------------
 * 
 * 
 * @date 11/11/2014 
 * @param logline 
 ---------------------------------------------------------------------- */
OdometryReading::OdometryReading(std::string logline){
    /// instantiate a large character array and copy string
    char logcstr[logline.size()+1];
    strncpy(logcstr, logline.c_str(), logline.size()+1);

    /// first token is just a flag
    strtok(logcstr, " \t\n\r"); // O
    this->x = atof(strtok(NULL," \t\n\r"));
    this->y = atof(strtok(NULL," \t\n\r"));
    this->theta = atof(strtok(NULL," \t\n\r"));
    this->t = atof(strtok(NULL," \t\n\r"));
}


/** ----------------------------------------------------------------------
 * 
 * 
 * @date 11/11/2014 
 ---------------------------------------------------------------------- */
LaserReading::LaserReading()
    : t(0) 
{
    /// set the ranges to zeros
    for (uint i = 0; i < 180; i++)
        ranges[i] = 0;
}


/** ----------------------------------------------------------------------
 * 
 * 
 * @date 11/11/2014 
 * @param logline 
 ---------------------------------------------------------------------- */
LaserReading::LaserReading(std::string logline){
    /// instantiate a large character array and copy string
    char logcstr[logline.size()+1];
    strncpy(logcstr,logline.c_str(),logline.size()+1);

    /// first tooken is just a flag
    strtok(logcstr, " \t\n\r"); // L
    /// ignore the first several position related fields
    /// we use these from odometry values
    strtok(NULL," \t\n\r"); // x
    strtok(NULL," \t\n\r"); // y
    strtok(NULL," \t\n\r"); // theta
    strtok(NULL," \t\n\r"); // xl
    strtok(NULL," \t\n\r"); // yl
    strtok(NULL," \t\n\r"); // thetal
    /// iterate and get the range readings
    for(int i = 0; i < 180; i++){
        this->ranges[i] = atoi(strtok(NULL," \t\n\r"));
    }
    /// get timestamp
    this->t = atof(strtok(NULL," \t\n\r"));
}
