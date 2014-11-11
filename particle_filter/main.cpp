/** ----------------------------------------------------------------------
 * Copyright 2014 < Ammar Husain (Carnegie Mellon University) > 
 * 
 * @file   main.cpp
 * @author Ammar Husain <ahusain@nrec.ri.cmu.edu> 
 * @date   Tue Nov 11 15:41:58 2014
 * 
 * @brief  
 * 
 * 
 ---------------------------------------------------------------------- */


#include <cstdio>
#include <iostream>
#include <fstream>
#include <string>
#include <cstring>
#include "src/ParticleFilter.h"


#define NUM_PARTICLES 10000
#define DATA_PATH "../../data"
#define MAP_NAME "/map/wean.dat"


/// ------------------------------------------------------- ///
/// Forward Declarations
void printUsage(char** argv);
void printApple();
std::string getFileContents (std::ifstream& File);
/// ------------------------------------------------------- ///

/** ----------------------------------------------------------------------
 * Main method: parses user input, sets up the particle filter and reads 
 * through the log file
 * @date 11/11/2014 
 * @param argc 
 * @param argv 
 * 
 * @return 
 ---------------------------------------------------------------------- */
int main(int argc, char** argv){

    /// print apple logo as ascii art
    printApple();
    
    if (argc < 2) {
        printUsage(argv);
        return 0;
    }

    std::string logname = "ascii-robotdata1.log";

    /// get the logname if specified
    if (strncmp(argv[1], "-d", 2) != 0)
        logname = std::string(argv[1]);

    /// create a map name
    char* mapName = new char[strlen(DATA_PATH) + strlen(MAP_NAME)];
    strcpy(mapName, DATA_PATH);
    strcat(mapName, MAP_NAME);
    
    /// instantiate a particle filter
    ParticleFilter particleFilter(mapName, NUM_PARTICLES);

    /// should we be writing output to disk
    if (argc == 4) {
        /// has the right flag been specified
        if (strncmp(argv[2], "-w", 2) != 0) {
            printUsage(argv);
            return 0;
        }

        /// get the name of the output directory
        std::string outputDir = std::string(argv[3]);
        /// request the particle filter to write output to disk in specified directory
        particleFilter.writeToDisk(outputDir);
        
    }
    
    std::string logFileName = std::string(DATA_PATH) + std::string("/log/") + logname;
    std::string logline;
    std::ifstream logfile(logFileName);

    
    if(logfile.is_open()){
        /// churn through the log file
        while(logfile.good()){

            getline(logfile, logline);
            /// is this an odometry update
            if(logline.compare(0,1,"O")==0){
                /// parse a odometry measurement
                OdometryReading odoRead(logline);

                /// move the particles through the motion model
                particleFilter.motionModel(odoRead);
            } else if(logline.compare(0,1,"L")==0) {
                /// parse a laser measurement
                LaserReading laserRead(logline);

                /// weight the value each particle through the observation model
                particleFilter.observationModel(laserRead);

                /// check if we should resample
                /// has the robot moved much in the map
                if(particleFilter.getDistTrav() > 1){
                    int numParticles = particleFilter.getNumParticles();

                    /// slowly start reducing particles in the map over time
                    if(numParticles > 1000){
                        numParticles -= 100;
                    }
                    /// resample and generate new particles in the map
                    particleFilter.resampleParticles(numParticles);
                }
            }
        }
    } else {
        std::cout << "Failed to open log file\n";
    }

    std::cout <<
        "\n Completed parsing the log! \n" << std::endl;
    
}


/** ----------------------------------------------------------------------
 * Prints usage instructions for this program
 * 
 * @date 11/11/2014 
 * @param argv 
 ---------------------------------------------------------------------- */
void printUsage(char** argv) {
    std::cout << "\n USAGE: " << argv[0] << " logname (for default: use -d) "
              << "[optional] -w write-path (for writing images to disk)\n"
              << std::endl;

    std::cout << " EXAMPLE: ./ParticleFilter -d \n" << std::endl;

    std::cout << " AVAILABLE LOGS: \n"
              << "\t ascii-robotdata1.log\n"
              << "\t ascii-robotdata2.log\n"
              << "\t ascii-robotdata3.log\n"
              << "\t ascii-robotdata4.log\n"
              << "\t ascii-robotdata5.log\n"
              << "\n" << std::endl;
    
}


/** ----------------------------------------------------------------------
 * Print Apple logo as ASCII art
 * 
 * @date 11/11/2014 
 ---------------------------------------------------------------------- */
void printApple() {
    std::ifstream appleReader ("../applelogo.txt");

    std::string Art = getFileContents (appleReader);
    /// print art to screen
    std::cout << Art << std::endl;
    std::cout << "\"Not all those who wander are lost\": J.R.R. Tolkien \n\n"
              << "APPLE CODING TEST: Particle Filter to localize a lost robot \n"
              << "Author: Ammar Husain\n" << std::endl;
    
    appleReader.close ();

}

/** ----------------------------------------------------------------------
 * Parses through the ASCII art file
 * 
 * @date 11/11/2014 
 * @param File 
 * 
 * @return 
 ---------------------------------------------------------------------- */
std::string getFileContents (std::ifstream& File) {
    /// All lines
    std::string Lines = "";
    /// Check if everything is good
    if (File) {
        while (File.good ()) {
            std::string TempLine;
            std::getline (File , TempLine);        
            TempLine += "\n";                      
	    
            Lines += TempLine;
        }
        return Lines;
    } else {
        // Return Error
        return "\n\nAPPLE LOGO should have been here: Oops!\n Please take this smiley instead: :-)\n\n";
    }
}

