/** ----------------------------------------------------------------------
 * Copyright 2014 < Ammar Husain (Carnegie Mellon University) > 
 * 
 * @file   InputData.h
 * @author Ammar Husain <ahusain@nrec.ri.cmu.edu> 
 * @date   Tue Nov 11 10:20:44 2014
 * 
 * @brief  Stores the data structures necessary to parse input datastreams
 * 
 * 
 ---------------------------------------------------------------------- */

#ifndef _INPUTDATA_H_
#define _INPUTDATA_H_

#include <stdio.h>
#include <string>
#include <cstring>
#include <cstdlib>

/** -------------------------------------------------------------- 
 * @struct OdometryReading 
 * 
 * @author Ammar Husain <ahusain@nrec.ri.cmu.edu>
 * @date   11/11/2014
 * 
 * @brief  Data structure to convert input data stream into odometry
 * Its a struct because no members need to be private (or hidden)
 * Just a collection of variables
 * ------------------------------------------------------------ */

struct OdometryReading {
    /// ------------------------------------------------------- ///
    /// Data
    double x;
    double y;
    double theta;
    double t;
    /// ------------------------------------------------------- ///

    /// ------------------------------------------------------- ///
    /// Functions
    /// no arguments constructor
    OdometryReading();
    
    /// constructor to parse raw log
    OdometryReading(std::string logline);
    /// ------------------------------------------------------- ///

};

/** -------------------------------------------------------------- 
 * @class  
 * 
 * @author Ammar Husain <ahusain@nrec.ri.cmu.edu>
 * @date   11/11/2014
 * 
 * @brief  Data structure to convert input data stream into laser range
 * readings. Its a struct because no members need to be private (or hidden)
 * Just a collection of variables
 * 
 * ------------------------------------------------------------ */
    
struct LaserReading {

    /// ------------------------------------------------------- ///
    /// Data
    int ranges[180];
    double t;
    /// ------------------------------------------------------- ///

    /// ------------------------------------------------------- ///
    /// Functions
    /// no arguments constructor
    LaserReading();
    
    /// constructor to parse raw log
    LaserReading(std::string logline);
    /// ------------------------------------------------------- ///

    
};


#endif
