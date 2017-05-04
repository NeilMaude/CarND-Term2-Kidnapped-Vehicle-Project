/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>

#include "particle_filter.h"

//using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// DONE: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).

  // Test code
  std::cout << "Initialise filter" << std::endl;
  std::cout << "x     = " << x << std::endl;
  std::cout << "y     = " << y << std::endl;
  std::cout << "theta = " << theta << std::endl;
  std::cout << "std[] = " << std[0] << "," << std[1] << "," << std[2] << std::endl;
  // end of test code

  std::default_random_engine gen;                               // create a random number engine                                                         
  std::normal_distribution<double> dist_x(x, std[0]);           // This line creates a normal (Gaussian) distribution mean x.
  std::normal_distribution<double> dist_y(y, std[1]);           // This line creates a normal (Gaussian) distribution mean y.
  std::normal_distribution<double> dist_theta(theta, std[2]);   // This line creates a normal (Gaussian) distribution mean theta.

  num_particles = 10;                                      // magic number of particles to start with
  for (int i = 0; i < num_particles; i++) {
    
    // create and add a new particle

    Particle particle;                                    // create a new particle structure
    particle.id = i;                                      // set the ID value
    particle.x = dist_x(gen);                             // create a x co-ordinate from the distribution
    particle.y = dist_y(gen);                             // create a y co-ordinate from the distribution
    particle.theta = dist_theta(gen);                     // create a bearing from the distribution
    particle.weight = 1.0;                                // default to weight = 1

    particles.push_back(particle);                        // add the particle to the particles vector

  }
  std::cout << "Number of particles created = " << particles.size() << std::endl;

  is_initialized = true;                                  // initialisation complete
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// DONE: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

  // Test code
  std::cout << "Prediction step" << std::endl;
  std::cout << "delta_t   = " << delta_t << std::endl;
  std::cout << "velocity  = " << velocity << std::endl;
  std::cout << "yaw_rate  = " << yaw_rate << std::endl;
  std::cout << "std_pos[] = " << std_pos[0] << "," << std_pos[1] << "," << std_pos[2] << std::endl;
  // end of test code
  
  /*
    *** What elements of the calculation require sensor noise?  
    3 standard deviations have been passed in std_pos[], which suggests adding noise to x,y,yaw
    However, the noise is in the measurements (velocity over the ground, yaw rate), rather than
    the final state.
    Had to stare at this for a while to convince myself that adding noise after the position 
    update is correct (rather than adding noise to the measurements ... but it's already in the
    measurements and we just want the new state to reflect that uncertainty).
  */

  std::default_random_engine gen;                               // create a random number engine
  std::normal_distribution<double> dist_x(0.0, std_pos[0]);     // This line creates a normal (Gaussian) distribution mean 0 for x.
  std::normal_distribution<double> dist_y(0.0, std_pos[1]);     // This line creates a normal (Gaussian) distribution mean 0 for y.
  std::normal_distribution<double> dist_theta(0.0, std_pos[2]);   // This line creates a normal (Gaussian) distribution mean 0 for theta/bearing.

  double noise_x;
  double noise_y;
  double noise_theta;

  // loop over particles in particles vector, update position, add noise
  for (int i = 0; i < particles.size(); i++) {
    
    // test code
    std::cout << "Particle ID: " << particles[i].id << std::endl;                         
    std::cout << "Previous x, y, theta : " << particles[i].x << "," << particles[i].y << "," << particles[i].theta << std::endl;

    // each particle has a current x,y and bearing - use these in the bicycle motion model, along with provided vel and yaw_rate

    // get some noise values for this particle
    noise_x = dist_x(gen);
    noise_y = dist_y(gen);
    noise_theta = dist_theta(gen);

    // do the updates - bicycle calc for each, with noise added 
    double theta_f = particles[i].theta + (yaw_rate * delta_t);         // calculate the final theta value and retain (use again)
    double v_over_yaw = velocity / yaw_rate;                            // calcualte v/yaw term for calculations use
    particles[i].x = particles[i].x + (v_over_yaw * (sin(theta_f) - sin(particles[i].theta))) + noise_x;    // x position calc
    particles[i].y = particles[i].y + (v_over_yaw * (cos(particles[i].theta) - cos(theta_f))) + noise_y;    // y position calc
    particles[i].theta = theta_f + noise_theta;                                                             // update theta

    // test code
    std::cout << "Updated x, y, theta : " << particles[i].x << "," << particles[i].y << "," << particles[i].theta << std::endl;

  }
  std::cout << std::endl;

}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		std::vector<LandmarkObs> observations, Map map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33. Note that you'll need to switch the minus sign in that equation to a plus to account 
	//   for the fact that the map's y-axis actually points downwards.)
	//   http://planning.cs.uiuc.edu/node99.html
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

}

void ParticleFilter::write(std::string filename) {
	// You don't need to modify this file.
	std::ofstream dataFile;
	dataFile.open(filename, std::ios::app);
	for (int i = 0; i < num_particles; ++i) {
		dataFile << particles[i].x << " " << particles[i].y << " " << particles[i].theta << "\n";
	}
	dataFile.close();
}
