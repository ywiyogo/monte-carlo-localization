// Copyright (c) 2022 Yongkie Wiyogo
//
// Licensed under the MIT license: https://opensource.org/licenses/MIT
// Permission is granted to use, copy, modify, and redistribute the work.
// Full license information available in the project LICENSE file.
//

#ifndef __ROBOT_H__
#define __ROBOT_H__

#include <random>
#include <vector>
#include <cmath>
#include "slam_config.h"
#include "helper.h"

/**
 * @brief Generate a random number from a uniform distribution
 */
double gen_real_random()
{
  // Generate real random between 0 and 1
  std::uniform_real_distribution<double> real_dist(0.0, 1.0);
  // Random Generators
  std::random_device random;
  std::mt19937 gen(random());
  return real_dist(gen);
}

/**
 * @brief Generate a random number from a Gaussian distribution
 */
double gen_gauss_random(double mean, double variance)
{
  // Gaussian random
  std::normal_distribution<double> gauss_dist(mean, variance);
  // Random Generators
  std::random_device random;
  std::mt19937 gen(random());
  return gauss_dist(gen);
}

class Robot
{
public:
  // robot poses
  Pose<double> pose_;

  // robot noises
  double forward_noise = 0.;
  double turn_noise = 0.;
  double sense_noise = 0.;

  // Maximum range of the sensor
  double max_range = 100.;

  // current measurements mapped the landmarks inside the range to the distance
  std::map<int, double> curr_measurements;

  Robot()
  {
    // robot's x,y  coordinate
    pose_.x = gen_real_random() * config::world_size;
    pose_.y = gen_real_random() * config::world_size;
    // robot's orientation
    pose_.orient = gen_real_random() * 2.0 * M_PI;
    // noise of the forward movement
    forward_noise = 0.0;
    // noise of the turn
    turn_noise = 0.0;
    // noise of the sensing
    sense_noise = 0.0;
  }
  Robot(double _x, double _y, double _o)
  {
    // Constructor
    pose_.x = _x;      // robot's x coordinate
    pose_.y = _y;      // robot's y coordinate
    pose_.orient = _o; // robot's orientation

    forward_noise = 0.0; // noise of the forward movement
    turn_noise = 0.0;    // noise of the turn
    sense_noise = 0.0;   // noise of the sensing
  }

  /**
   *
   */
  void set(Pose<double> new_pose)
  {
    // Set robot new position and orientation
    if (std::abs(new_pose.x) >= config::world_size)
    {
      std::string out = "X coordinate out of bound: " + std::to_string(new_pose.x);
      throw std::invalid_argument(out);
    }

    if (std::abs(new_pose.y) >= config::world_size)
    {
      std::string out = "Y coordinate out of bound: " + std::to_string(new_pose.y);
      throw std::invalid_argument(out);
    }
    if (std::abs(new_pose.orient) >= 2 * M_PI)
    {
      std::string out = "Orientation must be in [0..2pi], found " + std::to_string(new_pose.orient);
      throw std::invalid_argument(out);
    }
    pose_ = new_pose;
  }

  // Simulate noise, often useful in particle filters
  void set_noise(double new_forward_noise, double new_turn_noise, double new_sense_noise)
  {

    forward_noise = new_forward_noise;
    turn_noise = new_turn_noise;
    sense_noise = new_sense_noise;
  }

  /**
   * @brief Sense the environment
   */
  std::map<int, double> sense(const std::vector<std::array<double, 2>> &landmarks)
  {
    // Measure the distances from the robot toward the landmarks
    curr_measurements.clear();
    double dist;

    for (int i = 0; i < landmarks.size(); i++)
    {
      dist = sqrt(pow((pose_.x - landmarks[i][0]), 2) + pow((pose_.y - landmarks[i][1]), 2));
      if (std::abs(dist) < max_range)
      {
        dist += gen_gauss_random(0.0, sense_noise);
        curr_measurements.emplace(std::make_pair(i, dist));
      }
    }
    return curr_measurements;
  }

  /**
   * @brief Move the robot (or particle)
   */
  Robot move(double turn, double forward)
  {
    if (forward < 0)
      throw std::invalid_argument("Robot cannot move backward");

    // turn, and add randomness to the turning command
    pose_.orient = pose_.orient + turn + gen_gauss_random(0.0, turn_noise);
    pose_.orient = std::fmod(pose_.orient, 2 * M_PI);

    // move, and add randomness to the motion command
    double dist = forward + gen_gauss_random(0.0, forward_noise);
    pose_.x = pose_.x + (cos(pose_.orient) * dist);
    pose_.y = pose_.y + (sin(pose_.orient) * dist);

    // cyclic truncate
    pose_.x = std::fmod(pose_.x, config::world_size);
    pose_.y = std::fmod(pose_.y, config::world_size);

    // set particle
    Robot res;
    res.set(pose_);
    res.set_noise(forward_noise, turn_noise, sense_noise);

    return res;
  }
  /**
   * @brief print the robot pose
   */
  std::string show_pose() const
  {
    // Returns the robot current position and orientation
    return "[x=" + std::to_string(pose_.x) + " y=" + std::to_string(pose_.y) + " orient=" + std::to_string(pose_.orient) + "]";
  }

  std::string read_sensors(const std::vector<std::array<double, 2>> landmarks)
  {
    // Returns all the distances from the robot toward the landmarks
    std::map<int, double> z = sense(landmarks);
    std::string readings = "[";
    for (const auto &[lmark_idx, measurement_value] : z)
    {
      readings += std::to_string(measurement_value) + " ";
    }
    readings[readings.size() - 1] = ']';

    return readings;
  }

  /**
   * @brief Calculate the probability of the measurements from the landmarks for the particles.
   * The distance between the particle pose (not the robot!) and the landmarks is the mean of the Gaussian.
   */
  double measurement_prob(std::map<int, double> robot_measurements, const std::vector<std::array<double, 2>> landmarks)
  {
    double prob = 1.0;
    double dist;
    for (const auto &[lmark_idx, measurement_value] : robot_measurements)
    {
      dist = sqrt(pow((pose_.x - landmarks[lmark_idx][0]), 2) + pow((pose_.y - landmarks[lmark_idx][1]), 2));
      prob *= calc_gaussian(dist, sense_noise, measurement_value);
    }

    return prob;
  }

  /**
   * @brief Evaluation the estimation by calculating the mean error of the system
   * @return mean error
   */
  double evaluation(std::vector<Robot> particles)
  {
    double sum = 0.0;
    for (int i = 0; i < particles.size(); i++)
    {
      // the second part is because of world's cyclicity
      double dx = std::fmod((particles[i].pose_.x - pose_.x + (config::world_size / 2.0)), config::world_size) - (config::world_size / 2.0);
      double dy = std::fmod((particles[i].pose_.y - pose_.y + (config::world_size / 2.0)), config::world_size) - (config::world_size / 2.0);
      double err = sqrt(pow(dx, 2) + pow(dy, 2));
      sum += err;
    }
    return sum / particles.size();
  }

  /**
   * @brief Visualize the MCL with Matplotlib
   */
  void visualization(int n, Robot robot, int step, std::vector<Robot> &predict, std::vector<Robot> resampling, const std::vector<std::array<double, 2>> &landmarks, double delay = 0.01, std::string save_to = "")
  {

    plt::cla();

    plt::title("Monte Carlo Localization, step " + std::to_string(step));
    plt::xlim(0, static_cast<int>(config::world_size));
    plt::ylim(0, static_cast<int>(config::world_size));

    // Draw particles in green
    std::vector<double> particles_x(n);
    std::vector<double> particles_y(n);
    for (int i = 0; i < n; i++)
    {
      particles_x[i] = (predict[i].pose_.x);
      particles_y[i] = (predict[i].pose_.y);
    }
    plt::plot(particles_x, particles_y, "go", {{"label", "particles"}});

    // Draw resampled particles in red
    std::vector<double> resampled_x(n);
    std::vector<double> resampled_y(n);
    for (int i = 0; i < n; i++)
    {
      resampled_x[i] = resampling[i].pose_.x;
      resampled_y[i] = resampling[i].pose_.y;
    }
    plt::plot(resampled_x, resampled_y, "ro", {{"label", "resampled"}});

    // Draw landmarks in black
    std::vector<double> lmark_x;
    std::vector<double> lmark_y;
    for (int i = 0; i < landmarks.size(); i++)
    {
      lmark_x.push_back(landmarks[i][0]);
      lmark_y.push_back(landmarks[i][1]);
    }
    plt::plot(lmark_x, lmark_y, "ko", {{"label", "landmarks"}});
    // Draw robot position in blue
    double x_val = robot.pose_.x;
    double y_val = robot.pose_.y;
    plt::plot(std::vector<double>({x_val}), std::vector<double>({y_val}), "bo", {{"label", "robot pose"}});

    // Draw measurements
    for (const auto &[key, value] : curr_measurements)
    {
      plt::plot(std::vector({robot.pose_.x, landmarks[key][0]}), std::vector({robot.pose_.y, landmarks[key][1]}), "c:");
    }
    plt::axis("equal");
    plt::legend();
    plt::grid(true);
    plt::pause(delay);

    if (!save_to.empty())
    {
      std::string file = save_to + std::to_string(step) + ".jpg";
      plt::savefig(file);
      plt::clf();
    }
  }

private:
  /**
   * @brief Generate a random value from a Gaussian / normal distribution
   * @return random value
   */
  double gen_gauss_random(double mean, double variance)
  {
    // Gaussian random
    std::normal_distribution<double> gauss_dist(mean, variance);
    // Random Generators
    std::random_device rd;
    std::mt19937 gen(rd());
    return gauss_dist(gen);
  }
};

#endif // __ROBOT_H__