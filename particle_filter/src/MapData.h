/** ----------------------------------------------------------------------
 * Copyright 2014 < Ammar Husain (Carnegie Mellon University) > 
 * 
 * @file   MapData.h
 * @author Ammar Husain <ahusain@nrec.ri.cmu.edu> 
 * @date   Tue Nov 11 15:39:00 2014
 * 
 * @brief  Declarations for MapData 
 * This class handles all manipulations and checks required by ParticleFilter
 * on a map
 ---------------------------------------------------------------------- */
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cstdio>
#include <iostream>
#include <cmath>

class MapData{
  public:
    /// ------------------------------------------------------- ///
    /// Member Functions
    /// constructor: no args
    MapData();
    /// constructor
    MapData(char* mapName);
    /// initializes the object with map specified 
    void initialize(char* mapName);

    /// returns width in centimeters
    int getMapWidth_cm();
    /// return height in centimeters
    int getMapHeight_cm();
    /// checks if the cell is free to place robot
    bool checkMap(double x, double y);
    /// initialize a new image for visualization
    void initImg();
    /// draws particle to image
    void addPoint(double x, double y);
    /// casts a ray in space to compute expected range to hit 
    int rayCast(double lx,double ly,double ltheta);
    /// turns the write flag on and sets the directory to write to
    void setWrite(std::string output);
    /// visualize map with particles to screen
    void showMap();
    
  private:

    /// initializes a template image colored according to P(occupied)
    int initThreshMap();
    /// read the map and load into memory: provided by CMU 16-831 course staff
    int popMap(char *mapName);
    /// save the current visual map to disk
    void saveMap();
    
    /// ------------------------------------------------------- ///
    /// Memeber Variables
    /// map containing probability of free space at each grid
    cv::Mat m_prob;
    /// image to store initial colored prob threshold
    cv::Mat m_threshMap;
    /// image of map with particles floating inside
    cv::Mat m_img;
    /// resolution of the map
    int m_resolution;
    double m_offset_x;
    double m_offset_y;
    int m_size_x;
    int m_size_y;
    int m_min_x;
    int m_max_x;
    int m_min_y;
    int m_max_y;
    bool writeFlag;
    std::string writeDirectory;
    /// ------------------------------------------------------- ///


};
