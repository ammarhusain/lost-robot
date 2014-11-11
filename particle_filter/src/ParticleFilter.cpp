/** ----------------------------------------------------------------------
 * Copyright 2014 < Ammar Husain (Carnegie Mellon University) > 
 * 
 * @file   ParticleFilter.cpp
 * @author Ammar Husain <ahusain@nrec.ri.cmu.edu> 
 * @date   Tue Nov 11 11:03:24 2014
 * 
 * @brief  Function Definitions of the ParticleFilter class
 * 
 * 
 ---------------------------------------------------------------------- */

#include "ParticleFilter.h"

#define MAX_RANGE 7000
#define INIT_WEIGHT 0
#define PI 3.1415


/** ----------------------------------------------------------------------
 * No arguments constructor
 * Default: 10000 particles
 * @date 11/11/2014 
 ---------------------------------------------------------------------- */
/*ParticleFilter::ParticleFilter(){
    /// by default use 10000 particles to add to map
    initialize(10000);  
}
*/

/** ----------------------------------------------------------------------
 * Constructor that takes in as argument the number of particles to initialize
 * 
 * @date 11/11/2014 
 * @param numParticles 
 ---------------------------------------------------------------------- */
ParticleFilter::ParticleFilter(char* mapName, int numParticles){
    /// initialize the map
    m_map.initialize(mapName);
    
    initialize(numParticles);
}


/** ----------------------------------------------------------------------
 * Initializes the member variables for this class
 * 
 * @date 11/11/2014 
 * @param numParticles 
 ---------------------------------------------------------------------- */
void ParticleFilter::initialize(int numParticles) 
{
    for(int i = 0; i < 180; i++){m_radians[i] = (i*PI)/180;}

    /// generate a lookup table for probability of hit
    createProbLookupTable();
    /// seed a random number generator
    srand(time(NULL));
    /// set the distance moved to 0
    m_distTrav = 0;
    /// set an extremely large initial value as a flag that we just started
    m_curOdo.x = std::numeric_limits<double>::max();
    /// add particles to the map
    addParticles(numParticles);
}

/** ----------------------------------------------------------------------
 * Getter: how far has the robot moved since the last resampling
 * 
 * @date 11/11/2014 
 * 
 * @return 
 ---------------------------------------------------------------------- */
int ParticleFilter::getDistTrav(){
  return m_distTrav;
}

/** ----------------------------------------------------------------------
 * Getter: number of particles currently in the map
 * 
 * @date 11/11/2014 
 * 
 * @return # particles
 ---------------------------------------------------------------------- */
int ParticleFilter::getNumParticles() 
{
    return m_particles.size();
}

/** ----------------------------------------------------------------------
 * Function that samples a random number from a gaussian distribution
 * Found on the internet: but works really well
 * /// !!!!---- TODO: test with std::normal_distribution in c++11
 * @author: Harry Bovik
 * @date 11/11/2014 
 * 
 * @return Random number from a gaussian with mean=0 and sigma=1
 ---------------------------------------------------------------------- */
double ParticleFilter::randGauss(){
  double u,v,x,y,q;
  do{
    u = (rand()%10000)/10000.0;
    v = 1.7156*((rand()%10000)/10000.0-0.5);
    x = u-0.449871;
    y = fabs(v) + 0.386595;
    q = x*x + y*(0.1960*y-0.25472*x);
  }while(q>0.27597 && (q>0.27846 || v*v > -4*log(u) * u*u));
  return v/u;
}


/** ----------------------------------------------------------------------
 * Function to randomly sample locations on the map and add particles
 * if the location is traversable
 * @date 11/11/2014 
 * @param numParticles 
 ---------------------------------------------------------------------- */
void ParticleFilter::addParticles(int numParticles){
    /// iterate through the number of particles that must be added
    for(int i = 0; i < numParticles; i++){
        Particle curParticle;
        /// keep sampling until we land on a free cell
        do {
            curParticle.x = rand()%m_map.getMapWidth_cm();
            curParticle.y = rand()%m_map.getMapHeight_cm();
        } while(!m_map.checkMap(curParticle.x,curParticle.y));

        /// now sample the heading for this particle
        curParticle.theta = 2*PI*(rand()%10000)/10000.0;
        /// set an initial weight
        curParticle.weight = INIT_WEIGHT;
        /// add particle to final set
        m_particles.push_back(curParticle);
    }
}


/** ----------------------------------------------------------------------
 * This function creates a probability lookup table. It follows the 
 * sensor noise model from Probabilistic Robotics book, sec 6.3.1
 * Aka 4 types of noise: gaussian, exponential, max range discrete and uniform distr
 * Please refer to book for detailed mathematical information
 * @date 11/11/2014 
 ---------------------------------------------------------------------- */
void ParticleFilter::createProbLookupTable(){
    /// instantiate a 2D array of size MAX_RANGE x MAX_RANGE
    /// row stores expected hit and column stores actual hit
    m_p_beam = new double*[MAX_RANGE]; 
    for(int i = 0; i < MAX_RANGE; i++)
        m_p_beam[i] = new double[MAX_RANGE];
    
    /// expected hit zk*, actual hit zk
    /// iterate over all possible expected hits
    for(int exphit = 0; exphit < MAX_RANGE; exphit++){
        /// instantiate a pdf arrays for the noise sources

        /// gaussian distribution around a hit 
        double p_hit[MAX_RANGE];
        /// normalizer to keep integral of pdf = 1
        double norm_hit = 0;
        
        /// exponental distribution if there was a dynamic obstacle
        double p_short[MAX_RANGE];
        /// normalizer to keep integral of pdf = 1
        double norm_short = 0;

        /// uniform distribution for random measurements
        double p_rand[MAX_RANGE];
        /// normalizer to keep integral of pdf = 1
        double norm_rand = 0;
        

        /// ------------------------------------------------------- ///
        /// !!!!---- TODO: Parameters to tune: encapsulate if time
        /// weight associated with having the right laser hit
        double weight_hit = exphit/13000.0+0.05;
        /// sigma of gaussian around a valid hit
        double sigma_hit = exphit/12.0 + 50.0;

        /// weight associated with having a dynamic object in the laser
        double weight_short = exphit/20000.0;
        double lambda_short = 3.0/(exphit + 1.0);

        /// weight associated with a maximum hit: discrete prob
        double weight_max = 0.00015 + exphit/100000000.0;

        /// sum of weights should equal 1
        double weight_rand = 1 - weight_hit - weight_short - weight_max;
        /// ------------------------------------------------------- ///

        /// iterate over all possible laser hit for this zk*
        for(int i = 0; i < MAX_RANGE; i ++){
            /// compute probability within gaussian of a given laser reading
            /// mean: expected hit
            p_hit[i] =
                1/sqrt(2*PI*sigma_hit*sigma_hit)*exp(-0.5*(i-exphit)*(i-exphit)/(sigma_hit*sigma_hit));
            /// accumulate prob to normalize later
            norm_hit += p_hit[i];

            /// compute prob in exponential of an obstacle
            /// only valid if the laser hit is less than actual hit
            /// moving object must be closer than a map hit to count
            if(i < exphit) {
                p_short[i] = exp(-lambda_short*i);
                norm_short += p_short[i];
            } else {
                p_short[i] = 0;
            }

            /// prob of a random noisy measurement
            /// uniformly possible over entire range
            p_rand[i] = 1;
            norm_rand += p_rand[i];
        }

        /// iterate over all possible laser hits again and merge
        /// 3 continuous distributions and 1 discrete
        for(int i = 0; i < MAX_RANGE; i++) {
            /// normalize the 3 continuous probability distributions
            /// gaussian
            if(norm_hit == 0) {
                p_hit[i]=0;
            } else {
                p_hit[i]*=(100.0/norm_hit);
            }
            /// exponential
            if(norm_short == 0) {
                p_short[i]=0;
            } else {
                p_short[i]*=(100.0/norm_short);
            }
            /// uniform
            if(norm_rand == 0) {
                p_rand[i]=0;
            } else {
            p_rand[i]*=(100.0/norm_rand);
            }

            /// compute a weighted mix of the 3 distributions
            m_p_beam[exphit][i] =
                (weight_hit*p_hit[i]) +
                (weight_short*p_short[i]) +
                (weight_rand*p_rand[i]);
        }

        /// if this is at max range for laser: add a discrete noise probability
        m_p_beam[exphit][MAX_RANGE-1] += (weight_max*100.0);
        
        fprintf(stderr,
                "\r# Generating a lookup table for beam probabilities ... (%.2d%%)",
                (exphit*100)/MAX_RANGE);

    } /// end exphit
  
    /// final print: 100%
    fprintf(stderr,
            "\r# Generating a lookup table for beam probabilities ... (%.2d%%)\n\n",
            100);

}


/** ----------------------------------------------------------------------
 * This function implements a robot motion model as described in Probabilistic
 * Robotics (Thrun et al) book section 5.4
 * Aka 3 sources of noise to move from x_t-1 to x_t
 * (i) rotational to orient towards x_t, (ii) translate to x_t
 * (iii) rotational to achieve heading of x_t
 * Please refer to book for detailed mathematical information
 * @date 11/11/2014 
 * @param reading 
 ---------------------------------------------------------------------- */
void ParticleFilter::motionModel(OdometryReading reading){

    /// initialize a fresh map
    m_map.initImg();

    /// update previous odometry: x_t-1
    m_prevOdo = m_curOdo;
    /// store current odometry: x_t
    m_curOdo = reading;

    /// if this is the first step: nothing to do here yet
    if(m_prevOdo.x == std::numeric_limits<double>::max()){
        return;
    }

    
    double deltaRot1, deltaTrans, deltaRot2;

    /// initialize the 3 required motion to get from x_t-1 to x_t
    /// each will have a source of noise
    double deltaRot1base, deltaTransbase, deltaRot2base;
    /// compute required change in orientation from x_t-1 to face x_t
    deltaRot1base = atan2(m_curOdo.y-m_prevOdo.y, m_curOdo.x-m_prevOdo.x) 
        - m_prevOdo.theta;
    /// once we are facing x_t, compute the translation needed to reach x_t from x_t-1
    deltaTransbase = sqrt(((m_curOdo.x-m_prevOdo.x)*(m_curOdo.x-m_prevOdo.x))
                          +((m_curOdo.y-m_prevOdo.y)*(m_curOdo.y-m_prevOdo.y)));
    /// finally compute the rotation required to achieve the heading of x_t
    deltaRot2base = m_curOdo.theta-m_prevOdo.theta-deltaRot1base;

    /// update the distance we have traveled since last resampling
    /// this will get reset after resampling
    m_distTrav += deltaTransbase;

    /// ------------------------------------------------------- ///
    /// Parameters to add noise
    /// !!!!---- TODO: Encapsulate and tune if time
    double alpha1 = 0.005;
    double alpha2 = 0.005;
    double alpha3 = 0.2;
    double alpha4 = 0.2;
    /// ------------------------------------------------------- ///

    /// iterate over all particles and randomly sample noise in their motions
    std::vector<Particle>::iterator cpart; 
    for (cpart=m_particles.begin() ; cpart != m_particles.end(); cpart++) {
        /// sample noise in initial rotation
        deltaRot1 = deltaRot1base + 
            randGauss()*(alpha1*deltaRot1base*deltaRot1base +
                         alpha2*deltaRot2base*deltaRot2base);
        
        /// sample noise in translation
        deltaTrans = deltaTransbase + 
            randGauss()*(alpha3*deltaTransbase*deltaTransbase +
                         alpha4*deltaRot1base*deltaRot1base +
                         alpha4*deltaRot2base*deltaRot2base);

        /// sample noise in final rotaion
        deltaRot2 = deltaRot2base +
            randGauss()*(alpha1*deltaRot1base*deltaRot1base +
                         alpha2*deltaRot2base*deltaRot2base);

        /// noisily update the position of the particle
        cpart->x = cpart->x + deltaTrans*cos(cpart->theta+deltaRot1);
        cpart->y = cpart->y + deltaTrans*sin(cpart->theta+deltaRot1);
        cpart->theta = cpart->theta + deltaRot1 + deltaRot2;
        /// add this particle in the map
        m_map.addPoint(cpart->x,cpart->y);

    }
}


/** ----------------------------------------------------------------------
 * This function implements a robot observation model for range finders
 * as described in Probabilistic Robotics (Thrun et al) book section 6.3
 * It computes the weight for a given particle given its sensor measurements
 * and location within a map. Sensor measurements must have a high correlation
 * with what we can predict looking at the map. 
 * 
 * @date 11/11/2014 
 * @param reading 
 ---------------------------------------------------------------------- */
void ParticleFilter::observationModel(LaserReading reading){
    
    /// reset total weight in the map
    m_totalWeight = 0;

    /// iterate over all particles
    std::vector<Particle>::iterator cpart; 
    for (cpart=m_particles.begin() ; cpart != m_particles.end(); cpart++ ){
        /// variable to store the expected hit given a map and this location
        /// computed via raycasting: send a ray into space from scanner location
        /// until it hits something in the map
        int exprange = -1;
        /// variable to store what the scanner measured
        int measrange = -1;
        /// scanner calibration: 25 cm forward from the base
        /// compute the location of the scanner in the world
        double lx = cpart->x + 25*cos(cpart->theta);
        double ly = cpart->y + 25*sin(cpart->theta);

        /// set an initial weight for the particle
        cpart->weight = INIT_WEIGHT;

        /// initialize for each beam from scanner
        double ltheta;
        /// iterate over all 180 beams produced by scanner
        for (int i = 0; i < 180; i += 5){ 
            /// compute the orientation in the world
            ltheta = cpart->theta - m_radians[90] + m_radians[i];
            /// call raycast on the map to get the expected hit at this location
            exprange = m_map.rayCast(lx,ly,ltheta);
            /// get measurement from the scanner
            measrange = reading.ranges[i];
            /// make sure the hit values are sane
            if(measrange < MAX_RANGE && exprange < MAX_RANGE &&
               measrange >= 0 && exprange >=0){
                /// index in the lookup table we initialized for the probabilistic
                /// weight that this is a good beam and add to total weight
                cpart->weight += m_p_beam[exprange][measrange];
            }else if(measrange >= MAX_RANGE && exprange < MAX_RANGE &&
                     exprange >=0){
                /// is the expected hit within bounds but scanner is max_range
                /// add the prob weight of a max hit
                cpart->weight += m_p_beam[exprange][MAX_RANGE - 1];
            }
        }
        /// update total weight in this map with this particles weight
        m_totalWeight += cpart->weight;
    }
    /// visualize
    m_map.showMap();
}



/** ----------------------------------------------------------------------
 * This function performs Importance Sampling to sample particles in the map
 * with replacement given their current weight. Refer to Prob Robotics book
 * figure 4.3 for intuition.
 * 
 * @date 11/11/2014 
 * @param desiredNum 
 ---------------------------------------------------------------------- */
void ParticleFilter::resampleParticles(int desiredNum){
    /// initialize a vector for new particles
    std::vector<Particle> tempList;

    /// compute a step size according to weight
    double wtStep = (double)m_totalWeight/desiredNum; 
    /// get a perturbation within step
    double perturb = (rand()%10000)*wtStep/10000.0;

    /// fetch the iterator to the particle
    std::vector<Particle>::iterator cpart = m_particles.begin();
    /// initialize a cumulative weight with this particle
    double cumulWt = cpart->weight;
    
    /// start stepping through the particles
    double currStep;
    for(int m = 0; m < desiredNum; m++){
        /// take a step based on step size, pertubation and particle number
        currStep = perturb + m*wtStep;
        /// is this step larger than the cumulative weight added so far
        while(currStep > cumulWt){
            /// keep traversing particles until we get one with relatively high weight
            /// must bring enough to the set
            cpart++;
            cumulWt += cpart->weight;
        }

        /// this particle looks good to add
        cpart->weight = INIT_WEIGHT;
        tempList.push_back(*cpart);
  }

    /// update the member particles
    m_particles = tempList;
    /// set the distance to 0 - must move before we come here again
    m_distTrav = 0;
}


/** ----------------------------------------------------------------------
 * Requests the map to dump its visual data to disk
 * 
 * @date 11/11/2014 
 * @param outputDir 
 ---------------------------------------------------------------------- */
void ParticleFilter::writeToDisk(std::string outputDir) {
    m_map.setWrite(outputDir);
}
