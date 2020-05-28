/**
 * particle_filter.h
 * 2D particle filter class.
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 */

#ifndef PARTICLE_FILTER_H_
#define PARTICLE_FILTER_H_

#include "helper_functions.h"
#include <random>
#include <string>
#include <vector>

struct Particle
{
  int id;
  double x;
  double y;
  double theta;
  double weight;
  std::vector<int> associations;
  std::vector<double> sense_x;
  std::vector<double> sense_y;
};

class ParticleFilter
{
public:
  // Constructor
  // @param num_particles Number of particles
  ParticleFilter() : num_particles(0), is_initialized(false) {}

  // Destructor
  ~ParticleFilter() {}

  /**
   * init Initializes particle filter by initializing particles to Gaussian
   *   distribution around first position and all the weights to 1.
   * @param x Initial x position [m] (simulated estimate from GPS)
   * @param y Initial y position [m]
   * @param theta Initial orientation [rad]
   * @param std[] Array of dimension 3 [standard deviation of x [m],
   *   standard deviation of y [m], standard deviation of yaw [rad]]
   */
  void init(double x, double y, double theta, double std[]);

  /**
   * prediction Predicts the state for the next time step
   *   using the process model.
   * @param delta_t Time between time step t and t+1 in measurements [s]
   * @param std_pos[] Array of dimension 3 [standard deviation of x [m],
   *   standard deviation of y [m], standard deviation of yaw [rad]]
   * @param velocity Velocity of car from t to t+1 [m/s]
   * @param yaw_rate Yaw rate of car from t to t+1 [rad/s]
   */
  void prediction(double delta_t, double std_pos[], double velocity,
                  double yaw_rate);

  /**
   * dataAssociation Finds which observations correspond to which landmarks
   *   (likely by using a nearest-neighbors data association).
   * @param predicted_landmarks Vector of predicted landmark observations
   * @param observations Vector of landmark observations
   */
  void dataAssociation(const std::vector<LandmarkObs> &predicted_landmarks,
                       std::vector<LandmarkObs> &observations);

  /**
   * landmarksInRange Checks which groundtruth landmarks are in sensor range of a particle
   * @param sensor_range sensor range of the particle
   * @param particle The referenced particle
   * @param map_landmarks landmark groundtruth
   * @return vector of landmarks in sensor range
   */
  std::vector<LandmarkObs> landmarksInRange(const Particle particle,
                                            const double sensor_range,
                                            const Map &map_landmarks);

  /**
   * transformToMapCoordinates Transforms observations from local vehicle coordinates to global map coordinates
   * @param observations Vector of landmark observations in vehicle coordinates
   * @param particle The particle which is set as the reference for tranformation
   * @return transformed_observations in global map coordinates
   */
  std::vector<LandmarkObs> transformToMapCoordinates(const std::vector<LandmarkObs> &observations,
                                                     const Particle &particle);

  /**
   * multivariateGaussian Calculates the Multivariate-Gaussian of transformed measurements and  nearest landmarks
   * @param sig_x standard deviation x of landmarks
   * @param sig_y standard deviation y of landmarks
   * @param x x observation in map coordinates
   * @param y y observation in map coordinates
   * @param mu_x x coordinate of the nearest landmark
   * @param mu_y y coordinate of the nearest landmark
   * @return Multivariate-Gaussian
   */
  double multivariateGaussian(double sig_x, double sig_y, double x,
                              double y, double mu_x, double mu_y);

  /**
   * calculateWeight Calculates the importance weight for each particle
   * @param predicted_landmarks predicted landmarks
   * @param transformed_observations transformed observations
   * @param std_landmark stand deviations for landmarks
   * @return importance weight for the particle
   */
  double calculateWeight(const std::vector<LandmarkObs> &predicted_landmarks,
                         const std::vector<LandmarkObs> &transformed_observations,
                         double std_landmark[]);

  /**
   * updateWeights Updates the weights for each particle based on the likelihood
   *   of the observed measurements.
   * @param sensor_range Range [m] of sensor
   * @param std_landmark[] Array of dimension 2
   *   [Landmark measurement uncertainty [x [m], y [m]]]
   * @param observations Vector of landmark observations
   * @param map Map class containing map landmarks
   */
  void updateWeights(double sensor_range, double std_landmark[],
                     const std::vector<LandmarkObs> &observations,
                     const Map &map_landmarks);

  /**
   * normalizeWeights Normalizes weights for resampling
   */
  void normalizeWeights();

  /**
   * resamplingWheel Spins the "Wheel" to resample particles
   */
  void resamplingWheel();

  /**
   * resample Resamples from the updated set of particles to form
   *   the new set of particles.
   */
  void resample();

  /**
   * Set a particles list of associations, along with the associations'
   *   calculated world x,y coordinates
   * This can be a very useful debugging tool to make sure transformations
   *   are correct and assocations correctly connected
   */
  void SetAssociations(Particle &particle, const std::vector<int> &associations,
                       const std::vector<double> &sense_x,
                       const std::vector<double> &sense_y);

  /**
   * initialized Returns whether particle filter is initialized yet or not.
   */
  const bool initialized() const
  {
    return is_initialized;
  }

  /**
   * Used for obtaining debugging information related to particles.
   */
  std::string getAssociations(Particle best);
  std::string getSenseCoord(Particle best, std::string coord);

  // Set of current particles
  std::vector<Particle> particles;

private:
  // Number of particles to draw
  uint8_t num_particles;

  // Flag, if filter is initialized
  bool is_initialized;

  // Vector of weights of all particles
  std::vector<double> weights;

  // Random number generator for Gaussian distribution
  std::default_random_engine gen;
};

#endif // PARTICLE_FILTER_H_