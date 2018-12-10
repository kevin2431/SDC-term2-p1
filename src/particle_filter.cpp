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
	num_particles=100;
	default_random_engine gen;

	normal_distribution<double> dist_x(x, std[0]);
	normal_distribution<double> dist_y(y, std[1]);
	normal_distribution<double> dist_theta(theta, std[2]);
	for(int i = 0; i < num_particles; i++)
	{
		Particle p;
		p.id = i;
		p.x = dist_x(gen);
		p.y = dist_y(gen);
		p.theta = dist_theta(gen);
		p.weight = 1;
		weights.push_back(1);
		particles.push_back(p);
	}
	is_initialized = true;
	
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
	default_random_engine gen;
	for(int i = 0; i < num_particles; i++)
	{
		double new_x, new_y, new_theta;
		// 偏航角为很小的时候,视为直线运动
		if(yaw_rate <0.00001 && yaw_rate> -0.00001) {
			new_x = particles[i].x+velocity*delta_t*cos(particles[i].theta);
			new_y = particles[i].y+velocity*delta_t*sin(particles[i].theta);
			new_theta = particles[i].theta;
		}
		else{
		//一定偏航角变化时，位置变化公式
			new_x = particles[i].x+velocity*(sin(particles[i].theta+yaw_rate*delta_t)-sin(particles[i].theta))/yaw_rate;
			new_y = particles[i].y+velocity*(cos(particles[i].theta)-cos(particles[i].theta+yaw_rate*delta_t))/yaw_rate;
			new_theta= particles[i].theta+yaw_rate*delta_t;
		}
		//normal_distribution 测量
		normal_distribution<double> dist_x(new_x, std_pos[0]);
		normal_distribution<double> dist_y(new_y, std_pos[1]);
		normal_distribution<double> dist_theta(new_theta, std_pos[2]);

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
	
	//将观测点跟地标比较，找到最近的地标组合
	for(int i = 0; i < observations.size(); i++)
	{
		double min_dist=dist(observations[i].x,observations[i].y,predicted[0].x,predicted[0].y);
		observations[i].id=predicted[0].id;
		//id对应相应的最近观测点，id是从1开始的
		for(int j = 1; j < predicted.size(); j++)
		{
			double now_dist=dist(observations[i].x,observations[i].y,predicted[j].x,predicted[j].y);
			if(now_dist<min_dist){
				min_dist=now_dist;
				observations[i].id=predicted[j].id;
			}
		}
		
	}
	
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

	for(int p = 0; p < num_particles; p++)
	{
		//loop1 将车辆坐标下的点转到particle map下
		vector<LandmarkObs> trans_observations;
		LandmarkObs obs;
		for(int i = 0; i < observations.size(); i++)
		{
			obs=observations[i];
			LandmarkObs trans_obs;

			trans_obs.x= particles[p].x+obs.x*cos(particles[p].theta)-obs.y*sin(particles[p].theta);
			trans_obs.y= particles[p].y+obs.x*sin(particles[p].theta)+obs.y*cos(particles[p].theta);
			trans_observations.push_back(trans_obs);
		}
		
		//找到在sensor_range内的landmark
		vector<LandmarkObs> predicted;
		for(int j = 0; j < map_landmarks.landmark_list.size(); j++)
			{
				int landmark_id=map_landmarks.landmark_list[j].id_i;
				double landmark_x=map_landmarks.landmark_list[j].x_f;
				double landmark_y=map_landmarks.landmark_list[j].y_f;
				//LANDMARK和x,y的距离是都在sensor——range内，是的话加入vector<LandmarkObs> predicted
				double cal_dist=dist(landmark_x,landmark_y,particles[p].x,particles[p].y);
				if(cal_dist<=sensor_range){
					LandmarkObs l_obs;
					l_obs.id=landmark_id;
					l_obs.x=landmark_x;
					l_obs.y=landmark_y;
					predicted.push_back(l_obs);
				}
			}
		//要找到每一个观测点对应的landmark
		dataAssociation(predicted,trans_observations);
		//更新权重，
		particles[p].weight=1.0;
		double gauss_norm= (1/(2*M_PI*std_landmark[0]*std_landmark[1]));
		for(int i = 0; i < trans_observations.size(); i++)
		{
			double x_obs,y_obs,mu_x,mu_y,sig_x,sig_y;
			x_obs=trans_observations[i].x;
			y_obs=trans_observations[i].y;
			//land_mark_id from 1, so 需要-1访问数组
			mu_x=map_landmarks.landmark_list[trans_observations[i].id-1].x_f;
			mu_y=map_landmarks.landmark_list[trans_observations[i].id-1].y_f;
			sig_x=std_landmark[0];
			sig_y=std_landmark[1];
			double exponent=pow(x_obs-mu_x,2)/(2*pow(sig_x,2))+pow(y_obs-mu_y,2)/(2*pow(sig_y,2));
			double weight= gauss_norm * exp(-exponent);
			
        	particles[p].weight *= weight;
		}
		weights[p]=particles[p].weight;
	}

	
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
	//具体使用方法解释如下
	/*
	int main()
{
    vector<double> weights{0.2,0.3,0.1,0.4};
    default_random_engine gen;
    discrete_distribution<int> distribution(weights.begin(),weights.end());
    map<int,int> m;
    for(int i = 0; i < 10000; i++)
	{
		++m[distribution(gen)];
	}
	for(auto p : m) {
        std::cout << p.first << " generated " << p.second << " times\n";
    }
}
	先设定一个权重向量，然后放入到分布中
	随机生成器根据权重向量产生对应权重的小标，这里的结果如下
	0 generated 2011 times
	1 generated 2983 times
	2 generated 1022 times
	3 generated 3984 times
	*/
	
	default_random_engine gen;
	discrete_distribution<int> distribution(weights.begin(),weights.end());

	vector<Particle> resample_particles;
	for(int i = 0; i < num_particles; i++)
	{
		//distribution(gen)，产生的小标跟weight成正比
		//权重越大的例子，resample概率越大
		resample_particles.push_back(particles[distribution(gen)]);
	}
	particles=resample_particles;
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
