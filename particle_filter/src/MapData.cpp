/** ----------------------------------------------------------------------
 * Copyright 2014 < Ammar Husain (Carnegie Mellon University) > 
 * 
 * @file   MapData.cpp
 * @author Ammar Husain <ahusain@nrec.ri.cmu.edu> 
 * @date   Tue Nov 11 15:39:43 2014
 * 
 * @brief  Function Definitions for the MapData class
 * 
 * 
 ---------------------------------------------------------------------- */

#include "MapData.h"

#define OBJ_THRESH 0.03


/** ----------------------------------------------------------------------
 * No arguments constructor
 * 
 * @date 11/11/2014 
 ---------------------------------------------------------------------- */
MapData::MapData() {
    /// nothing to do until we get a map
    m_resolution = 0;
}


/** ----------------------------------------------------------------------
 * Constructs the object by loading a map specified by the argument
 * 
 * @date 11/11/2014 
 * @param mapName 
 ---------------------------------------------------------------------- */
MapData::MapData(char* mapName){
    /// load the map and set the other member variables
    initialize(mapName);
}


/** ----------------------------------------------------------------------
 * Function to load a map into memory
 * 
 * @date 11/11/2014 
 * @param mapName 
 ---------------------------------------------------------------------- */
void MapData::initialize(char* mapName) {

    /// assert(popMap("../../data/map/wean.dat") != -1);
    /// load the map: assert if failed
    assert(popMap(mapName) != -1);
    initThreshMap();
    initImg();

    /// default is not to store images
    writeFlag = false;
}


/** ----------------------------------------------------------------------
 * Function that sets the flag to dump image output to disk
 * 
 * @date 11/11/2014 
 * @param output 
 ---------------------------------------------------------------------- */
void MapData::setWrite(std::string output) {
    writeFlag = true;
    
    
    /// set the writedirectory
    writeDirectory = output;
    
    /// append with a / is not already
    if (output[output.size() - 1] != '/')
        writeDirectory = output + "/";

    std::cout << "# Writing data to: " << writeDirectory << std::endl;
    
}


/** ----------------------------------------------------------------------
 * Initializes a template map with map occupancy thresholds. This map is used to 
 * initialize a new map to visualize particles
 * @date 11/11/2014 
 * 
 * @return 
 ---------------------------------------------------------------------- */
int MapData::initThreshMap() {
    for( int y = 0; y < m_threshMap.rows; y++) {
        for( int x = 0; x < m_threshMap.cols; x++) {
            float &curprob = m_prob.at<float>(y,x);
            if(curprob < 0) {
                /// unexplored areas of the map
                m_threshMap.at<cv::Vec3b>(y,x)[0] = 100;
                m_threshMap.at<cv::Vec3b>(y,x)[1] = 0;
                m_threshMap.at<cv::Vec3b>(y,x)[2] = 0;
            } else if(curprob > 100) {
                /// definitely occupied areas
                m_threshMap.at<cv::Vec3b>(y,x)[0] = 0;
                m_threshMap.at<cv::Vec3b>(y,x)[1] = 100;
                m_threshMap.at<cv::Vec3b>(y,x)[2] = 0;
            } else {
                /// relatively unoccupied: stuff behind walls
                m_threshMap.at<cv::Vec3b>(y,x)[0] = 255*curprob;
                m_threshMap.at<cv::Vec3b>(y,x)[1] = 255*curprob;
                m_threshMap.at<cv::Vec3b>(y,x)[2] = 255*curprob;
            }
        }
    }
    return 0;
}


/** ----------------------------------------------------------------------
 * Getter: Width of the map in centimeters
 * 
 * @date 11/11/2014 
 * 
 * @return 
 ---------------------------------------------------------------------- */
int MapData::getMapWidth_cm(){
  return m_prob.cols*m_resolution;
}


/** ----------------------------------------------------------------------
 * Getter: Height of the map in centimeters
 * 
 * @date 11/11/2014 
 * 
 * @return 
 ---------------------------------------------------------------------- */
int MapData::getMapHeight_cm(){
  return m_prob.rows*m_resolution;
}


/** ----------------------------------------------------------------------
 * Checks whether this cell is unoccupied and good for a robot to traverse on
 * 
 * @date 11/11/2014 
 * @param x 
 * @param y 
 * 
 * @return True if we can place a robot here
 ---------------------------------------------------------------------- */
bool MapData::checkMap(double x, double y){
  int xch = floor((x/m_resolution)+0.5);
  int ych = floor((y/m_resolution)+0.5);
  if(xch >= 0 && xch < m_img.cols && ych >= 0 && ych < m_img.rows
     && m_prob.at<float>(ych,xch) < OBJ_THRESH
     && m_prob.at<float>(ych,xch) >= 0){
    return true;
  }else{return false;}
}


/** ----------------------------------------------------------------------
 * Function to initialize an image map for visualization
 * 
 * @date 11/11/2014 
 ---------------------------------------------------------------------- */
void MapData::initImg(){
  m_img = m_threshMap.clone();
}


/** ----------------------------------------------------------------------
 * Function to add a particle point on the visualized map
 * x,y are in centimeters
 * @date 11/11/2014 
 * @param x 
 * @param y 
 ---------------------------------------------------------------------- */
void MapData::addPoint(double x, double y){ 
  int xadd = floor((x/m_resolution)+0.5);
  int yadd = floor((y/m_resolution)+0.5);
  if(xadd >= 0 && xadd < m_img.cols && yadd >= 0 && yadd < m_img.rows){
    cv::circle(m_img, cv::Point(xadd,yadd), 1, cv::Scalar(0,0,255),1);
  }
}


/** ----------------------------------------------------------------------
 * Function to perform ray casting from a certain position and heading
 * Sends a ray into space from scanner location until it hits something in the map
 * 
 * @date 11/11/2014 
 * @param lx 
 * @param ly 
 * @param ltheta 
 * 
 * @return 
 ---------------------------------------------------------------------- */
int MapData::rayCast(double lx,double ly,double ltheta){
    /// variables to stores indices within the map that need to be checked
    int chx,chy;

    /// set the max range as 6000cm: even though its shorter than the
    /// prescribed 7000; things far away tend to be noisy
    int maxray = 6000;
    /// set a minimum threshold for hit to be beyond 30cm
    int minray = 30;

    /// compute the sine and cosine for this orientation
    double sltheta = sin(ltheta); double cltheta = cos(ltheta);

    /// start stepping through the ray 5 cms at a time
    for(int rcheck = 0; rcheck <= maxray; rcheck += 5){
        /// compute index into the map from this world coordinates
        chx = floor(((lx + rcheck*cltheta)/m_resolution)+0.5);
        chy = floor(((ly + rcheck*sltheta)/m_resolution)+0.5);
        /// have we escaped the bounds of the map
        if(chx < 0 || chx >= m_img.cols || chy < 0 || chy >= m_img.rows){
            return -1;
        }
        /// is there beyond a reasonable level of doubt that this grid is occupied
        if(m_prob.at<float>(chy,chx) >= OBJ_THRESH){
            /// have we cleared a minimum threshold
            if(rcheck < minray) {
                return -1;
            } else {
                /// this is good hit: return this range
                return rcheck;
            }
        } else if(m_prob.at<float>(chy,chx) < 0) {
            /// strange occupied probability: we have entered a worm hole
            return maxray;
        }
    }
    /// couldnt find anything occupied within range
    return maxray;
}


/** ----------------------------------------------------------------------
 * Function to visualize the map with particles
 * 
 * @date 11/11/2014 
 ---------------------------------------------------------------------- */
void MapData::showMap(){
  cv::imshow("Map", m_img(cv::Range(m_min_y,m_max_y),
                          cv::Range(m_min_x,m_max_x)));
  if (writeFlag)
      saveMap();
  
  cv::waitKey(5);
}


/** ----------------------------------------------------------------------
 * Function to write the visualized map to file
 * 
 * @date 11/11/2014 
 ---------------------------------------------------------------------- */
void MapData::saveMap(){
    std::stringstream out;
    static int frnum = 0;
    out << (frnum/1000)%10 << (frnum/100)%10 << (frnum/10)%10 << (frnum)%10;
    cv::Mat flipImage; cv::flip(m_img(cv::Range(m_min_y,m_max_y),
                                      cv::Range(m_min_x,m_max_x)),flipImage,0);
    frnum++; cv::imwrite(writeDirectory + "img_"+out.str()+".jpg",flipImage);
}


/** ----------------------------------------------------------------------
 * Helper function to read Wean Hall map from data provided
 * Provided by course staff of CMU 16-831 from where I got this data
 * @author: Harry Bovik
 * @date 11/11/2014 
 * @param mapName 
 * 
 * @return Flag to indicate if map read was successful
 ---------------------------------------------------------------------- */
int MapData::popMap(char *mapName){
  int x, y, count;
  float temp;
  char line[256];
  FILE *fp;

  if((fp = fopen(mapName, "rt")) == NULL) {
      fprintf(stderr, "# Could not open file %s\n", mapName);
      return -1;
  }
  fprintf(stderr, "# Reading map: %s\n", mapName);
  while((fgets(line, 256, fp) != NULL)
	&& (strncmp("global_map[0]", line , 13) != 0)) {
    if(strncmp(line, "robot_specifications->resolution", 32) == 0)
      if(sscanf(&line[32], "%d", &(m_resolution)) != 0)
	printf("# Map resolution: %d cm\n", m_resolution);
    if(strncmp(line, "robot_specifications->autoshifted_x", 35) == 0)
      if(sscanf(&line[35], "%g", &(m_offset_x)) != 0) {
	m_offset_x = m_offset_x;
	printf("# Map offsetX: %g cm\n", m_offset_x);
      }
    if(strncmp(line, "robot_specifications->autoshifted_y", 35) == 0) {
      if (sscanf(&line[35], "%g", &(m_offset_y)) != 0) {
	m_offset_y = m_offset_y;
	printf("# Map offsetY: %g cm\n", m_offset_y);
      }
    }
  }

  if(sscanf(line,"global_map[0]: %d %d", &m_size_y, &m_size_x) != 2) {
    fprintf(stderr, "ERROR: corrupted file %s\n", mapName);
    fclose(fp);
    return -1;
  }
  printf("# Map size: %d %d\n", m_size_x, m_size_y);

  m_prob.create(m_size_y, m_size_x, CV_32FC1);
  m_threshMap.create(m_size_y, m_size_x, CV_8UC3);
  m_img.create(m_size_y, m_size_x, CV_8UC3);

  m_min_x = m_size_x;
  m_max_x = 0;
  m_min_y = m_size_y;
  m_max_y = 0;
  count = 0;
  for(x = 0; x < m_size_x; x++)
    for(y = 0; y < m_size_y; y++, count++) {
      if(count % 10000 == 0)
	fprintf(stderr, "\r# Reading ... (%.2f%%)",
		count / (float)(m_size_x * m_size_y) * 100);
      
      fscanf(fp,"%e", &temp);
      if(temp < 0.0)
	m_prob.at<float>(y,x) = -1;
      else {
	if(x < m_min_x)
	  m_min_x = x;
	else if(x > m_max_x)
	  m_max_x = x;
	if(y < m_min_y)
	  m_min_y = y;
	else if(y > m_max_y)
	  m_max_y = y;
	m_prob.at<float>(y,x) = 1 - temp;
      }
    }
  fprintf(stderr, "\r# Reading ... (%.2f%%)\n\n",
	  count / (float)(m_size_x * m_size_y) * 100);
  fclose(fp);
  return 0;
}

