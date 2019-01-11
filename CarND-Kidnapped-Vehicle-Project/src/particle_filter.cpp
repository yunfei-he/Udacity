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
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
   num_particles = 100;

   default_random_engine gen;
   normal_distribution<double> dist_x(x, std[0]);
   normal_distribution<double> dist_y(y, std[1]);
   normal_distribution<double> dist_theta(theta, std[2]);
   for (unsigned int i = 0; i < num_particles; ++i) {
	   Particle p;
	   p.x = dist_x(gen);
	   p.y = dist_y(gen);
	   p.theta = dist_theta(gen);
	   p.weight = 1.0;

	   particles.emplace_back(p);
   }
   is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
	default_random_engine gen;

	double yaw_change = delta_t * yaw_rate;
    for (auto& particle: particles) {
		if (fabs(yaw_rate) > 0.001) {
			particle.x += velocity * (sin(particle.theta + yaw_change) - sin(particle.theta)) / yaw_rate;
			particle.y += velocity * (cos(particle.theta) - cos(particle.theta + yaw_change)) / yaw_rate;
			particle.theta += yaw_change;
		} else {
			particle.x += velocity * delta_t * cos(particle.theta);
			particle.y += velocity * delta_t * sin(particle.theta);
		}

		normal_distribution<double> dist_x(particle.x, std_pos[0]);
   		normal_distribution<double> dist_y(particle.y, std_pos[1]);
   		normal_distribution<double> dist_theta(particle.theta, std_pos[2]);
		particle.x = dist_x(gen);
		particle.y = dist_y(gen);
		particle.theta = dist_theta(gen);
	}
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html

	for (auto& particle: particles) {
		// transfrom obsevations for each partical
		std::vector<LandmarkObs> trans_observations;
		for (auto const& landmark_obs: observations) {
			LandmarkObs obs;
			obs.id = landmark_obs.id;
			obs.x = particle.x + landmark_obs.x * cos(particle.theta) - landmark_obs.y * sin(particle.theta);
			obs.y = particle.y + landmark_obs.x * sin(particle.theta) + landmark_obs.y * cos(particle.theta);

			trans_observations.emplace_back(obs);
		}	

		// calculate normalization term
		double gauss_norm= 1.0/(2 * M_PI * std_landmark[0] * std_landmark[1]);
		double prob = 1.0;
		// find data association
		for (auto const& landmark_obs: trans_observations) {
			double dist_min = std::numeric_limits<double>::max();
			int index = -1;
			for (unsigned int i = 0; i < map_landmarks.landmark_list.size(); ++i) {
				auto const& landmark_map = map_landmarks.landmark_list[i];
				double d = dist(landmark_obs.x, landmark_obs.y, landmark_map.x_f, landmark_map.y_f);
				if (d < dist_min) {
					dist_min = d;
					index = i;
				}	
			}
			if (index != -1) {
				auto const& landmark_map = map_landmarks.landmark_list[index];
				if (dist(particle.x, particle.y, landmark_map.x_f, landmark_map.y_f) <= sensor_range) {	
					// calculate exponent
					double exponent= pow(landmark_obs.x - landmark_map.x_f, 2) / (2 * pow(std_landmark[0], 2)) + 
					                 pow(landmark_obs.y - landmark_map.y_f, 2) / (2 * pow(std_landmark[1], 2));
					// calculate weight using normalization terms and exponent
					double weight= gauss_norm * exp(-exponent);

					// update weights for particle
					prob *= weight;
				}
			}
		}

		particle.weight = prob;

	}

}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
	vector<double> p_weights;
	for (auto const& particle : particles) {
		p_weights.push_back(particle.weight);
	}

	default_random_engine gen;
	discrete_distribution<> dist(p_weights.begin(), p_weights.end());

	vector<Particle> new_particles;
	for (int i = 0; i < particles.size(); ++i) {
		new_particles.push_back(particles[dist(gen)]);
	}
	particles = new_particles;
}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
