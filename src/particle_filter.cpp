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

  num_particles = 1;                                      // magic number of particles to start with
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

  /*
  // Test code
  std::cout << "Prediction step" << std::endl;
  std::cout << "delta_t   = " << delta_t << std::endl;
  std::cout << "velocity  = " << velocity << std::endl;
  std::cout << "yaw_rate  = " << yaw_rate << std::endl;
  std::cout << "std_pos[] = " << std_pos[0] << "," << std_pos[1] << "," << std_pos[2] << std::endl;
  // end of test code
  */

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
    
    /*
    // test code
    std::cout << "Particle ID: " << particles[i].id << std::endl;                         
    std::cout << "Previous x, y, theta : " << particles[i].x << "," << particles[i].y << "," << particles[i].theta << std::endl;
    */

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

    /*
    // test code
    std::cout << "Updated x, y, theta : " << particles[i].x << "," << particles[i].y << "," << particles[i].theta << std::endl;
    */

  }
  std::cout << std::endl;

}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

  // The predicted vector is the list of map points
  // The observations is the observations list, translated into map co-ordinates
  // The aim of this function is to set the .id value for each observations entry to the .id of the nearest map point

  /*
  // test code
  std::cout << "dataAssociation, " << "Landmarks : " << predicted.size() << ", Observations : " << observations.size() << std::endl;
  */

  // for each observation, scan over the landmarks and see which is the nearest
  for (int i = 0; i < observations.size(); i++) {

    double best_distance = DBL_MAX;
    
    /*
    // test code
    std::cout << "Initial best distance : " << best_distance << std::endl;
    // end test code
    */

    for (int j = 0; j < predicted.size(); j++) {

      // get the Euclidean distance between this observation and the prediction
      double x_sq = fabs(observations[i].x - predicted[j].x);
      x_sq = x_sq * x_sq;
      double y_sq = fabs(observations[i].y - predicted[j].y);
      y_sq = y_sq * y_sq;
      double e_distance = sqrt(x_sq + y_sq);                        // can likely save a few cycles by not doing sqrt() here...

      if (e_distance < best_distance) {
        observations[i].id = predicted[j].id;
        best_distance = e_distance;

        /*
        // test code
        std::cout << "New best distance : " << best_distance << ", id = " << observations[i].id <<  std::endl;
        // end test code
        */

      }

    }

  }

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

  /*
  // test code
  std::cout << "UpdateWeights" << std::endl;
  std::cout << "Sensor range  : " << sensor_range << std::endl;
  std::cout << "Observations  : " << observations.size() << std::endl;
  std::cout << "Map landmarks : " << map_landmarks.landmark_list.size() << std::endl;
  // end of test code
  */

  for (int i = 0; i < particles.size(); i++) {
    // update weights for the i-th particle

    /*
    // test code
    std::cout << "Particle #" << i << std::endl;
    std::cout << "Current weight : " << particles[i].weight << std::endl;
    // end of test code
    */

    // find the nearest neighbour for each measurement
    // first create a vector of observations in map co-ordinates
    //    this will be used in the dataAssociation() function and for calculating the differences in the weight update
    std::vector<LandmarkObs> obs_map;
    double sin_theta = sin(particles[i].theta);     // will use this several times
    double cos_theta = cos(particles[i].theta);     // will use this several times
    double particle_x = particles[i].x;             // will use this as well
    double particle_y = particles[i].y;             // ditto

    for (int j = 0; j < observations.size(); j++) {
      // loop over the observations

      /*
      // test code
      std::cout << "Observation #" << j << std::endl;
      std::cout << "id,x,y = " << observations[j].id << "," << observations[j].x << "," << observations[j].y << std::endl;
      // end of test code
      */

      // now convert this measurement to map co-ordinates
      LandmarkObs new_p;
      new_p.id = observations[j].id;                                                              // Although .id is not initialised yet...
      new_p.x = particle_x + (observations[j].x * cos_theta) - (observations[j].y * sin_theta);   // ignoring the "swap sign" in the comments
      new_p.y = particle_y + (observations[j].y * sin_theta) + (observations[j].y * cos_theta);
      obs_map.push_back(new_p);                                                                   // put this observation on the obs_map vector

      /*
      // test code
      std::cout << "New observation id,x,y = " << new_p.id << "," << new_p.x << "," << new_p.y << std::endl;
      // end of test code
      */

    }

    // now get a vector of possible landmarks - i.e. those which are within sensor range of the particle
    std::vector<LandmarkObs> landmarks_near;

    for (int k = 0; k < map_landmarks.landmark_list.size(); k++) {

      double x_sq = fabs(particle_x - map_landmarks.landmark_list[k].x_f);
      x_sq = x_sq * x_sq;
      double y_sq = fabs(particle_y - map_landmarks.landmark_list[k].y_f);
      y_sq = y_sq * y_sq;
      double e_distance = sqrt(x_sq + y_sq);                                  // Euclidean distance from particle to this predicted landmark

      if (e_distance < sensor_range) {                                        // if within range, use this landmark

        /*
        // test code
        std::cout << "Distance to landmark = " << e_distance << " particle = " << particle_x << "," << particle_y;
        std::cout << " landmark #" << map_landmarks.landmark_list[k].id_i << " = " << map_landmarks.landmark_list[k].x_f;
        std::cout << "," << map_landmarks.landmark_list[k].y_f << std::endl;
        // end test code
        */

        LandmarkObs new_l;
        new_l.id = map_landmarks.landmark_list[k].id_i;
        new_l.x = map_landmarks.landmark_list[k].x_f;
        new_l.y = map_landmarks.landmark_list[k].y_f;
        landmarks_near.push_back(new_l);
      }

    }

    // might not be any landmarks within range
    if (landmarks_near.size() > 0) {
      dataAssociation(landmarks_near, obs_map);
    }
    else {
      // no nearby landmarks
      std::cout << "Warning - no landmarks within sensor range of particle #" << i << std::endl;
      for (int k = 0; k < obs_map.size(); k++) {
        // set the obs nearest landmarks to all be the first landmark - will be very unlikely that this particle survives...
        obs_map[k].id = 1;
      }
    }


    /*
    // test code
    for (int k = 0; k < obs_map.size(); k++) {
      // print out the nearest landmarks list
      std::cout << "Observation #" << k << " set to landmark #" << obs_map[k].id << std::endl;
    }
    // end test code
    */

    // update the weight
    // for each observation for this particle, calculate the multivariate Gaussian
    // sigmas from std_landmark[] array
    double sigma_x = std_landmark[0];
    double sigma_y = std_landmark[1];
    double sigma_x_sq_2 = sigma_x * sigma_x * 2.0;                // will use this a lot - might want to make const
    double sigma_y_sq_2 = sigma_y * sigma_y * 2.0;                // will use this a lot
    double denom_term = 1.0 / (2 * M_PI * sigma_x * sigma_y);     // ditto

    double new_weight = 1.0;

    for (int j = 0; j < obs_map.size(); j++) {

      // need to change this to access the original list of landmarks - the ids refer to that, not the ordering in the nearby list...
      double x_term = fabs(obs_map[j].x - map_landmarks.landmark_list[obs_map[j].id - 1].x_f);      // -1 in reference as list is 0=based, IDs 1-based
      x_term = x_term * x_term / sigma_x_sq_2;
      double y_term = fabs(obs_map[j].y - map_landmarks.landmark_list[obs_map[j].id - 1].y_f);
      y_term = y_term * y_term / sigma_y_sq_2;
      double p_xy = (-1.0) * (x_term + y_term);
      p_xy = exp(p_xy) / denom_term;
      new_weight = new_weight * p_xy;

    }

    particles[i].weight = new_weight;

    /*
    // test code
    std::cout << "Particle #" << i << ", Updated weight : " << particles[i].weight << std::endl;
    // end of test code
    */

    // Note: the p_xy terms can be very small for unlikely observations

  }

  // now normalise the particle weights?
  double normaliser = 0.0;
  for (int i = 0; i < particles.size(); i++) {
    normaliser += particles[i].weight;
  }
  
  /*
  // test code
  std::cout << "Normaliser value = " << normaliser << std::endl;
  // end test code
  */

  if (normaliser != 0.0) {
    for (int i = 0; i < particles.size(); i++) {
      particles[i].weight = particles[i].weight / normaliser;
      
      /*
      // test code
      std::cout << "Normalised particle weight = " << particles[i].weight << " (particle id=" << i << ")" << std::endl;
      // end test code
      */

    }
  }
  else {
    // zero normaliser - all particle weights must have collapsed to zero...
    
    // test code
    std::cout << "All particles have zero weight!" << std::endl;
    // end of test code
    
    // might want to set all of the particle weights to the uniform distribution here (i.e. max uncertainty)
    // allows the algorith to continue
    // maybe even want to re-initialise?
    // however, don't want this to ever happen in practice, as we now have no information about where the vehicle is located
    for (int i = 0; i < particles.size(); i++) {
      particles[i].weight = 1.0 / particles.size();
    }
  }
  
}


void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

  std::default_random_engine gen;                               // create a random number engine
  
  // set up the weights for the discrete distribution
  std::vector<double> weights;
  for (int i = 0; i < particles.size(); i++) {
    weights.push_back(particles[i].weight);
  }

  std::discrete_distribution<> discrete_sample(weights.begin(), weights.end());     // set up the discrete distribution

  int n_sample;

  // need to make a copy of the particles list, from which to copy back as samples are made
  std::vector<Particle> old_particles;
  old_particles = particles;              // be very surprised if it's that easy!!!

  for (int i = 0; i < particles.size(); i++) {
    // sample a particle from the distribution
    n_sample = discrete_sample(gen);

    /*
    // test code
    std::cout << "Sampled particle #" << n_sample << " - which had prob: " << old_particles[n_sample].weight << std::endl;
    //
    */

    // copy the particle from old_particles over this particle in particles
    particles[i] = old_particles[n_sample];   // again, can't be this easy can it?

  }

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
