/** ----------------------------------------------------------------------
 * Copyright 2014 < Ammar Husain (Carnegie Mellon University) > 
 * 
 * @file   ParticleFilter.h
 * @author Ammar Husain <ahusain@nrec.ri.cmu.edu> 
 * @date   Tue Nov 11 11:02:42 2014
 * 
 * @brief  Declaration of the ParticleFilter class
 * This class implements the particle filter algorithm used to localize robots
 * Relies on CMU Wean Hall dataset
 ---------------------------------------------------------------------- */

#include "MapData.h"
#include "InputData.h"
#include <vector>
#include <time.h>
#include <cstdlib>
#include <fstream>
#include <limits>


/** -------------------------------------------------------------- 
 * @struct  Particle
 * 
 * @author Ammar Husain <ahusain@nrec.ri.cmu.edu>
 * @date   11/11/2014
 * 
 * @brief  
 * Data structure for a particle in a map
 * ------------------------------------------------------------ */
struct Particle {
    /// contains the xy, yaw and weight of the particle
    double x;
    double y;
    double theta;
    double weight;
};


/** -------------------------------------------------------------- 
 * @class  ParticleFilter
 * 
 * @author Ammar Husain <ahusain@nrec.ri.cmu.edu>
 * @date   11/11/2014
 * 
 * @brief  
 * Encapsulates the particle filtering algorithm to localize a robot 
 * within a given map. Implementation is based on Probabilistic Robotics book
 * (Thrun et al) sec 4.2
 * ------------------------------------------------------------ */
class ParticleFilter {
  public:
  

    /// ------------------------------------------------------- ///
    /// Member Functions
    /// Constructor
    ParticleFilter(char* mapName, int totalNumParticles = 10000);
    /// initializes the object
    void initialize(int totalNumParticles);
    /// scaters particles throughout the map
    void addParticles(int numParticles);
    /// runs a probabilistic robot motion model based on odometry
    void motionModel(OdometryReading reading);
    /// computes the weight of each particle based on correlation between
    /// prediction and measurement
    void observationModel(LaserReading reading);
    /// sample with replacement particles based on their weights
    void resampleParticles(int desiredNum);
    /// return how far the robot has moved
    int getDistTrav();
    /// return how many particles are in the map 
    int getNumParticles();
    /// request the map object to store data to disk
    void writeToDisk(std::string outputDir);
    

  private:

    /// generates a random number from a gaussian distribution
    double randGauss();
    /// creates a lookup table to make the observation model faster
    void createProbLookupTable();

    /// ------------------------------------------------------- ///
    /// Member Variables
    /// lookup table for probabilities
    double **m_p_beam;
    /// beam ranges
    double m_radians[180];
    /// map object to localize in
    MapData m_map;
    /// keep track of how far the robot has moved
    int m_distTrav;
    /// collection of particles in the map
    std::vector<Particle> m_particles;
    /// previous robot position: x_t-1
    OdometryReading m_prevOdo;
    /// current robot position: x_t
    OdometryReading m_curOdo;
    /// total weight of particles in the map
    double m_totalWeight;
    /// ------------------------------------------------------- ///

};

