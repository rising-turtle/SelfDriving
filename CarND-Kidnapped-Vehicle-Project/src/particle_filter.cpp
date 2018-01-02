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
#include <cassert>

#include "particle_filter.h"

using namespace std;

#define SQ(x) ((x)*(x))

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
	
	if(is_initialized)
	{
	    cerr <<"particle_filter: has already initialized, init again? "<<endl;
	    return ; 
	}

	particles.resize(num_particles); 
	weights.resize(num_particles, 1.0); 

	default_random_engine gen; 
	normal_distribution<double> dist_x(x, std[0]);
	normal_distribution<double> dist_y(y, std[1]);
	normal_distribution<double> dist_theta(theta, std[2]); 
	
	for(int i=0; i<num_particles; i++)
	{
	    struct Particle p; 
	    p.id = i;
	    p.x = dist_x(gen);
	    p.y = dist_y(gen);
	    p.theta = dist_theta(gen); 
	    p.weight = weights[i]; 
	    particles[i] = p;
	}
	is_initialized = true; 
	cout <<"particle_filter: succeed to sample "<<num_particles<<" particles!"<<endl;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
	
	static default_random_engine gen; 
	static normal_distribution<double> err_x(0, std_pos[0]); 
	static normal_distribution<double> err_y(0, std_pos[1]); 
	static normal_distribution<double> err_theta(0, std_pos[2]); 
	for(int i=0; i<num_particles; i++)
	{
	    double xf, yf, theta_f; 
	    double x0, y0, theta_0; 
	    x0 = particles[i].x;
	    y0 = particles[i].y; 
	    theta_0 = particles[i].theta; 
	    if(fabs(yaw_rate) < 1e-5)
	    {
		theta_f = theta_0; 
		xf = x0 + velocity * delta_t * cos(theta_0); 
		yf = y0 + velocity * delta_t * sin(theta_0);
	    }else
	    {
		theta_f = theta_0 + yaw_rate * delta_t; 
		xf = x0 + velocity*(sin(theta_f) - sin(theta_0))/yaw_rate; 
		yf = y0 + velocity*(cos(theta_0) - cos(theta_f))/yaw_rate; 
	    }
	    particles[i].x = xf + err_x(gen); 
	    particles[i].y = yf + err_y(gen); 
	    particles[i].theta = theta_f + err_theta(gen); 
	}
}

void ParticleFilter::dataAssociation(const Map& map, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
	const std::vector<Map::single_landmark_s>& predicted = map.landmark_list;
	for(int i=0; i<observations.size(); i++)
	{
	    double min_dis = 10000000;
	    int min_id = -1; 
	    LandmarkObs& obs = observations[i]; 
	    for(int j=0; j<predicted.size(); j++)
	    {
		double dis = SQ(predicted[j].x_f - obs.x) + SQ(predicted[j].y_f - obs.y); 
		if(dis < min_dis)
		{
		    min_dis = dis; 
		    min_id = j;
		}
	    }
	    assert(min_id >= 0);
	    obs.id = min_id; // predicted[min_id].id; 
	}
	return ; 
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
    double variance_x = SQ(std_landmark[0]); 
    double variance_y = SQ(std_landmark[1]);
    double sum_weight = 0.; 
    double eps = 1e-6;
	for(int i=0; i<num_particles; i++)
	{
	    // 1. transform observations from car coordinate into map coordinate 
	    vector<LandmarkObs> obs_map  = observations; 
	    for(int k=0; k<obs_map.size(); k++)
	    {	
		double ct = cos(particles[i].theta);
		double st = sin(particles[i].theta); 
		obs_map[k].x = particles[i].x + ct * observations[k].x - st * observations[k].y; 
		obs_map[k].y = particles[i].y + st * observations[k].x + ct * observations[k].y;
	    }
	    // 2. association 
	    // dataAssociation(map_landmarks.landmark_list, obs_map); 
	    dataAssociation(map_landmarks, obs_map);

	    // 3. calculate probability & set sensor measurement to particle
	    double prob = 0.0;
	    double norm = 1./(2*M_PI*std_landmark[0]*std_landmark[1]);
	    unsigned int K = obs_map.size(); 
	    particles[i].sense_x.resize(K); 
	    particles[i].sense_y.resize(K); 
	    particles[i].associations.resize(K); 
	    for(int k=0; k<K; k++)
	    {
		LandmarkObs& obs = obs_map[k]; 
		double e_fac = -(SQ(obs.x - map_landmarks.landmark_list[obs.id].x_f)/(2.*variance_x) + 
				    SQ(obs.y - map_landmarks.landmark_list[obs.id].y_f)/(2.*variance_y));
		prob += (log(norm) + e_fac); 
		particles[i].sense_x[k] = obs.x; 
		particles[i].sense_y[k] = obs.y;
		particles[i].associations[k] = map_landmarks.landmark_list[obs.id].id_i;
		if(0 && i==0)
		{
		    cout << "obs["<<k<<"]: x: "<<obs.x<<" y: "<<obs.y<<endl<<" map_landmark: x: "<<map_landmarks.landmark_list[obs.id].x_f
			<< " y: "<<map_landmarks.landmark_list[obs.id].y_f<<endl;
		    cout <<"e_fac = "<<e_fac<<" log(norm): "<<log(norm)<<endl;
		    cout <<"prob: "<<prob<<endl;
		}
	    }
	    prob = exp(prob) + eps; 
	    
	    // 4. update weights
	    weights[i] = prob;
	    sum_weight += prob; 
	    // cout <<"weights["<<i<<"] = "<<prob<<endl;
	}
    // cout <<"sum_weight = "<<sum_weight<<endl;
    assert(sum_weight > 0); 
    for(int i=0; i<num_particles; i++)
    {
	weights[i] /= sum_weight;
	particles[i].weight = weights[i]; 
    }
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
	
	double wm = 0; 
	for(int i=0; i<num_particles; i++)
	{
	    if(wm < weights[i]) wm = weights[i]; 
	}
	
	vector<double> new_w = weights; 
	vector<Particle> new_p = particles; 
	for(int i=0; i<num_particles; i++)
	{
	    // random 
	    double beta = 0.; 
	    int index = (int)(((double)rand()/(RAND_MAX))*(num_particles-1)); 
	    beta += ((double)rand()/(RAND_MAX))*2*wm; 

	    while(weights[index] < beta)
	    {
		beta -= weights[index]; 
		index = (index+1)%num_particles;
	    }
	    new_w[i] = weights[index]; 
	    new_p[i] = particles[index];
	}
	particles.swap(new_p); 
	weights.swap(new_w); 
	return ; 
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
