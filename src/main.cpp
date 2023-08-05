// Copyright (c) 2022 Yongkie Wiyogo
//
// Licensed under the MIT license: https://opensource.org/licenses/MIT
// Permission is granted to use, copy, modify, and redistribute the work.
// Full license information available in the project LICENSE file.
//

#include <iostream>
#include <string>
#include <stdexcept>
#include <unistd.h>

#include "robot.h"
#include "helper.h"

int main()
{
  // Landmarks
  // std::vector<std::array<double, 2>> landmarks = {{{10.0, 20.0}, {10.0, 80.0}, {20.0, 50.0}, {50.0, 10.0}, {50.0, 80.0}, {80.0, 80.0}, {80.0, 20.0}, {90.0, 50.0}}};

  std::vector<std::array<double, 2>> landmarks;
  for (int x = 0; x < 20; x++)
  {
    for (int y = 0; y < 2; y++)
    {
      landmarks.push_back(std::array<double, 2>{10. + x * 10., y * 100.});
    }
  }

  Robot robot(100., 10, 0.);

  // Create a set of particles that has the robot
  int n = 1000;
  std::vector<Robot> particles;
  particles.resize(n);

  for (int i = 0; i < n; i++)
  {
    particles[i].set_noise(0.05, 0.05, 5.0);
  }

  std::vector<double> measurements;
  // move simulation parameters for robot and particle prediction must be equal
  double turn_sim = 0.1;
  double move_sim = 3.;

  // Iterating the simulation over the time steps
  int steps = 60;
  for (unsigned int t = 0; t < steps; t++)
  {
    // Move the robot and sense the environment
    robot = robot.move(turn_sim, move_sim);
    measurements = robot.sense(landmarks);

    // Do the predict step by simulating a robot motion for each of these particles
    std::vector<Robot> predict;
    predict.resize(n);

    for (int i = 0; i < predict.size(); i++)
    {
      predict[i] = particles[i].move(turn_sim, move_sim);
      particles[i] = predict[i];
    }

    // Generate particle weights depending on robot's measurement
    std::vector<double> p_weights;
    p_weights.resize(n);

    for (int i = 0; i < n; i++)
    {
      p_weights[i] = particles[i].measurement_prob(measurements, landmarks);
    }

    // Resample the particles with a sample probability proportional to the importance weight
    std::vector<Robot> resampling;
    resampling.resize(n);
    int index = gen_real_random() * n;

    double beta = 0.0;
    double mw = max(p_weights);
    // std::cout << mw;
    for (int i = 0; i < n; i++)
    {
      beta += gen_real_random() * 2.0 * mw;
      while (beta > p_weights[index])
      {
        beta -= p_weights[index];
        index = std::fmod((index + 1), n);
      }
      resampling[i] = particles[index];
    }
    for (int k = 0; k < n; k++)
    {
      particles[k] = resampling[k];
    }

    // Evaluate the Error
    std::cout << "Step = " << t << ", Evaluation = " << robot.evaluation(particles) << std::endl;

    // Create a new folder otherwise the library returns a std::runtime_error
    mkdir("images", 0755);
    robot.visualization(n, robot, t, predict, resampling, landmarks, 0.0001, "images/steps");

  } // End of simulation

  return 0;
}