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
#include <limits>
#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {

    default_random_engine gen;
    double std_x, std_y, std_theta;
    
    std_x = std[0];
    std_y = std[1];
    std_theta = std[2];
    
    normal_distribution<double> dist_x(x, std_x);
    normal_distribution<double> dist_y(y, std_y);
    normal_distribution<double> dist_theta(theta, std_theta);
    
    num_particles = 500;
    
    for (int i = 0; i < num_particles; i++) {
        Particle p;
        
        p.id = i;
        p.x = dist_x(gen);
        p.y = dist_y(gen);
        p.theta = dist_theta(gen);
        p.weight = 1.0;
        
        p.associations.clear();
        p.sense_x.clear();
        p.sense_y.clear();
        
        particles.push_back(p);
        weights.push_back(1.0);
    }
    
    is_initialized = true;
    
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
    
    default_random_engine gen;
    
    double std_x, std_y, std_theta;
    
    std_x = std_pos[0];
    std_y = std_pos[1];
    std_theta = std_pos[2];
    
    for (int i = 0; i < num_particles; i++) {
        
        double new_x, new_y, new_theta;
        
        if(yaw_rate == 0) {
            new_x = particles[i].x + velocity * delta_t * cos(particles[i].theta);
            new_y = particles[i].y + velocity * delta_t * sin(particles[i].theta);
            new_theta = particles[i].theta;
        }
        else {
            new_theta = particles[i].theta + (yaw_rate * delta_t);
            new_x = particles[i].x + ((velocity/yaw_rate) * (sin(new_theta) - sin(particles[i].theta)));
            new_y = particles[i].y + ((velocity/yaw_rate) * (cos(particles[i].theta) - cos(new_theta)));
        }
        
        normal_distribution<double> dist_x(new_x, std_x);
        normal_distribution<double> dist_y(new_y, std_y);
        normal_distribution<double> dist_theta(new_theta, std_theta);
        
        particles[i].x = dist_x(gen);
        particles[i].y = dist_y(gen);
        particles[i].theta = dist_theta(gen);
    }
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
    // TODO: Find the predicted measurement that is closest to each observed measurement and assign the
    //   observed measurement to this particular landmark.
    // NOTE: this method will NOT be called by the grading code. But you will probably find it useful to
    //   implement this method and use it as a helper during the updateWeights phase.
    
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
		std::vector<LandmarkObs> observations, Map map_landmarks) {
	
    weights.clear();
    
    for (int i = 0; i < num_particles; i++) {
        
        std::vector<LandmarkObs> predictions;
        
        std::vector<int> associations;
        std::vector<double> sense_x;
        std::vector<double> sense_y;
        
        predictions.clear();
        associations.clear();
        sense_x.clear();
        sense_y.clear();
        
        for(int j = 0; j < observations.size(); j++) {
            LandmarkObs trans_obs;
            
            trans_obs.x = (observations[j].x * cos(particles[i].theta)) - (observations[j].y * sin(particles[i].theta)) + particles[i].x;
            trans_obs.y = (observations[j].x * sin(particles[i].theta)) + (observations[j].y * cos(particles[i].theta)) + particles[i].y;
            
            trans_obs.id   = observations[j].id;
            
            predictions.push_back(trans_obs);
        }
        
        for(int j = 0; j < predictions.size(); j++) {
            
            double pos_x = predictions[j].x;
            double pos_y = predictions[j].y;
            
            double min_dist = std::numeric_limits<float>::max();
            int l_id = -1;
            double l_x = 0;
            double l_y = 0;
            
            for(int k = 0; k < map_landmarks.landmark_list.size(); k++) {
                
                double land_x = map_landmarks.landmark_list[k].x_f;
                double land_y = map_landmarks.landmark_list[k].y_f;
                
                double cur_dist = dist(pos_x, pos_y, land_x, land_y);
                
                if(cur_dist < min_dist) {
                    min_dist = cur_dist;
                    l_id = map_landmarks.landmark_list[k].id_i;
                    l_x  = land_x;
                    l_y  = land_y;
                }
            }
            
            sense_x.push_back(l_x);
            sense_y.push_back(l_y);
            associations.push_back(l_id);
        }
        
        SetAssociations(particles[i], associations, sense_x, sense_y);
        
        double std_x = std_landmark[0];
        double std_y = std_landmark[1];
        double acc_weight = 1;
        
        for(int j = 0; j < associations.size(); j++) {
            
            double l_x = sense_x[j];
            double l_y = sense_y[j];
            int l_id   = associations[j];
            
            double pos_x = predictions[j].x;
            double pos_y = predictions[j].y;
            
            double weight = ((1 / (2 * M_PI * std_x * std_y)) * exp(-((((pos_x - l_x) * (pos_x - l_x)) / (2 * std_x * std_x)) + (((pos_y - l_y) * (pos_y - l_y)) / (2 * std_y * std_y)))));
            
            if(weight > 0) {
                acc_weight *= weight;
            }
        }
        
        particles[i].weight = acc_weight;
        weights.push_back(acc_weight);
    }
}


void ParticleFilter::resample() {
    
    default_random_engine gen;
    Particle p;
    
    std::vector<Particle> new_particles;
    std::discrete_distribution<> dist(this->weights.begin(), this->weights.end());
    
    new_particles.clear();
    particles.clear();
    
    for(int i = 0; i < num_particles; i++) {
        int id = dist(gen);
        new_particles.push_back(particles[id]);
    }
    
    particles = new_particles;
    
}

Particle ParticleFilter::SetAssociations(Particle particle, std::vector<int> associations, std::vector<double> sense_x, std::vector<double> sense_y){
	//particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
	// associations: The landmark id that goes along with each listed association
	// sense_x: the associations x mapping already converted to world coordinates
	// sense_y: the associations y mapping already converted to world coordinates

	//Clear the previous associations
	particle.associations.clear();
	particle.sense_x.clear();
	particle.sense_y.clear();

	particle.associations= associations;
 	particle.sense_x = sense_x;
 	particle.sense_y = sense_y;

 	return particle;
}

string ParticleFilter::getAssociations(Particle best){
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best){
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best){
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
