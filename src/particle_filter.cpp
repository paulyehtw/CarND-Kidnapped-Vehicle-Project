/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 */

#include "particle_filter.h"

#include <algorithm>
#include <iostream>
#include <iterator>
#include <math.h>
#include <string>
#include <vector>

#include "helper_functions.h"

using std::string;
using std::vector;

void ParticleFilter::init(double x, double y, double theta, double std[])
{
  /**
   * TODO: Set the number of particles. Initialize all particles to
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1.
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method
   *   (and others in this file).
   */
  num_particles = 100; // TODO: Set the number of particles

  std::normal_distribution<double> distrib_x(x, std[0]);
  std::normal_distribution<double> distrib_y(y, std[1]);
  std::normal_distribution<double> distrib_theta(theta, std[2]);

  // Initialize every particle and its weight
  for (uint8_t idx = 0; idx < num_particles; ++idx)
  {
    Particle particle;
    particle.id = idx;
    particle.x = distrib_x(gen);
    particle.y = distrib_y(gen);
    particle.theta = distrib_theta(gen);
    particle.weight = 1.0;

    particles.push_back(particle);
    weights.push_back(particle.weight);
  }

  is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[],
                                double velocity, double yaw_rate)
{
  /**
   * TODO: Add measurements to each particle and add random Gaussian noise.
   * NOTE: When adding noise you may find std::normal_distribution
   *   and std::default_random_engine useful.
   *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   *  http://www.cplusplus.com/reference/random/default_random_engine/
   */

  double epslon = 0.0001;

  for (Particle &particle : particles)
  {
    // Apply motion model
    if (fabs(yaw_rate) > epslon)
    {
      double theta_0 = particle.theta;               // inital yaw
      double theta_1 = theta_0 + yaw_rate * delta_t; // final yaw
      particle.x += (velocity / yaw_rate) * (std::sin(theta_1) - std::sin(theta_0));
      particle.y += (velocity / yaw_rate) * (std::cos(theta_0) - std::cos(theta_1));
      particle.theta = theta_1;
    }
    else
    {
      particle.x += velocity * delta_t * std::cos(particle.theta);
      particle.y += velocity * delta_t * std::sin(particle.theta);
    }

    // Add noise
    std::normal_distribution<double> distrib_x(particle.x, std_pos[0]);
    std::normal_distribution<double> distrib_y(particle.y, std_pos[1]);
    std::normal_distribution<double> distrib_theta(particle.theta, std_pos[2]);

    particle.x = distrib_x(gen);
    particle.y = distrib_y(gen);
    particle.theta = distrib_theta(gen);
  }
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted,
                                     vector<LandmarkObs> &observations)
{
  /**
   * TODO: Find the predicted measurement that is closest to each
   *   observed measurement and assign the observed measurement to this
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will
   *   probably find it useful to implement this method and use it as a helper
   *   during the updateWeights phase.
   */

  for (LandmarkObs &ob : observations)
  {
    double min_distance = std::numeric_limits<double>::max();
    int idx = -1;
    for (LandmarkObs &pred : predicted)
    {
      double ob_pred_distance = dist(ob.x, ob.y, pred.x, pred.y);
      if (ob_pred_distance < min_distance)
      {
        min_distance = ob_pred_distance;
        idx = pred.id;
      }
    }
    // Associate the nearest prediction ID to observation
    ob.id = idx;
  }
}

std::vector<LandmarkObs> ParticleFilter::landmarksInRange(const Particle particle,
                                                          const double sensor_range,
                                                          const Map &map_landmarks)
{
  vector<LandmarkObs> predictions{};
  const std::vector<Map::single_landmark_s> landmark_list = map_landmarks.landmark_list;
  for (Map::single_landmark_s landmark : landmark_list)
  {
    if (dist(particle.x, particle.y, landmark.x_f, landmark.y_f) < sensor_range)
    {
      LandmarkObs predcted_landmark;
      predcted_landmark.id = landmark.id_i;
      predcted_landmark.x = landmark.x_f;
      predcted_landmark.y = landmark.y_f;
      predictions.push_back(predcted_landmark);
    }
  }
  return predictions;
}

std::vector<LandmarkObs> ParticleFilter::transformToMapCoordinates(const std::vector<LandmarkObs> &observations,
                                                                   const Particle &particle)
{
  vector<LandmarkObs> transformed_observations{};
  for (LandmarkObs ob : observations)
  {
    LandmarkObs transformed_ob;
    transformed_ob.id = ob.id;
    transformed_ob.x = particle.x + (cos(particle.theta) * ob.x) - (sin(particle.theta) * ob.y);
    transformed_ob.y = particle.y + (sin(particle.theta) * ob.x) + (cos(particle.theta) * ob.y);
    transformed_observations.push_back(transformed_ob);
  }
  return transformed_observations;
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
                                   const vector<LandmarkObs> &observations,
                                   const Map &map_landmarks)
{
  /**
   * TODO: Update the weights of each particle using a mult-variate Gaussian
   *   distribution. You can read more about this distribution here:
   *   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
   * NOTE: The observations are given in the VEHICLE'S coordinate system.
   *   Your particles are located according to the MAP'S coordinate system.
   *   You will need to transform between the two systems. Keep in mind that
   *   this transformation requires both rotation AND translation (but no scaling).
   *   The following is a good resource for the theory:
   *   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
   *   and the following is a good resource for the actual equation to implement
   *   (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
   */
  for (Particle particle : particles)
  {
    vector<LandmarkObs> transformed_observations = transformToMapCoordinates(observations, particle);
    vector<LandmarkObs> predictions = landmarksInRange(particle, sensor_range, map_landmarks);
  }
}

void ParticleFilter::resample()
{
  /**
   * TODO: Resample particles with replacement with probability proportional
   *   to their weight.
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */
}

void ParticleFilter::SetAssociations(Particle &particle,
                                     const vector<int> &associations,
                                     const vector<double> &sense_x,
                                     const vector<double> &sense_y)
{
  // particle: the particle to which assign each listed association,
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations = associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best)
{
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length() - 1); // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord)
{
  vector<double> v;

  if (coord == "X")
  {
    v = best.sense_x;
  }
  else
  {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length() - 1); // get rid of the trailing space
  return s;
}