//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\file    particle-filter.cc
\brief   Particle Filter Starter Code
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <algorithm>
#include <cmath>
#include <iostream>
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "shared/math/geometry.h"
#include "shared/math/line2d.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"

#include "config_reader/config_reader.h"
#include "particle_filter.h"

#include "vector_map/vector_map.h"

using geometry::line2f;
using std::cout;
using std::endl;
using std::string;
using std::swap;
using std::vector;
using Eigen::Vector2f;
using Eigen::Vector2i;
using vector_map::VectorMap;

DEFINE_double(num_particles, 50, "Number of particles");

namespace particle_filter {

config_reader::ConfigReader config_reader_({"config/particle_filter.lua"});

ParticleFilter::ParticleFilter() :
    prev_odom_loc_(0, 0),
    prev_odom_angle_(0),
    odom_initialized_(false) {}

void ParticleFilter::GetParticles(vector<Particle>* particles) const {
  *particles = particles_;
}

void ParticleFilter::GetPredictedPointCloud(const Vector2f& loc,
                                            const float angle,
                                            int num_ranges,
                                            float range_min,
                                            float range_max,
                                            float angle_min,
                                            float angle_max,
                                            vector<Vector2f>* scan_ptr) {
  vector<Vector2f>& scan = *scan_ptr;
  // Compute what the predicted point cloud would be, if the car was at the pose
  // loc, angle, with the sensor characteristics defined by the provided
  // parameters.
  // This is NOT the motion model predict step: it is the prediction of the
  // expected observations, to be used for the update step.

  // Note: The returned values must be set using the `scan` variable:
  scan.resize(num_ranges);
  // Fill in the entries of scan using array writes, e.g. scan[i] = ...
  for (size_t i = 0; i < scan.size(); ++i) {
    scan[i] = Vector2f(0, 0);
  }

  // The line segments in the map are stored in the `map_.lines` variable. You
  // can iterate through them as:
  for (size_t i = 0; i < map_.lines.size(); ++i) {
    const line2f map_line = map_.lines[i];
    // The line2f class has helper functions that will be useful.
    // You can create a new line segment instance as follows, for :
    line2f my_line(1, 2, 3, 4); // Line segment from (1,2) to (3.4).
    // Access the end points using `.p0` and `.p1` members:
    printf("P0: %f, %f P1: %f,%f\n", 
           my_line.p0.x(),
           my_line.p0.y(),
           my_line.p1.x(),
           my_line.p1.y());

    // Check for intersections:
    bool intersects = map_line.Intersects(my_line);
    // You can also simultaneously check for intersection, and return the point
    // of intersection:
    Vector2f intersection_point; // Return variable
    intersects = map_line.Intersection(my_line, &intersection_point);

    // Calculate ray line
    // Calculate the start point at range_min
    Eigen::Vector2f start(range_min * cos(angle_min), range_min * sin(angle_min));

    // Calculate the end point at range_max
    Eigen::Vector2f end(range_max * cos(angle_max), range_max * sin(angle_max));

    // Declare the ray line as line2f variable and initialize with calculated ray start and end points
    line2f ray_line(start, end);

    // Check if laser ray intersects with map lines
    Vector2f ray_intersection_point;
    bool ray_intersects = map_line.Intersects(ray_line);

    // Check for every tenth ray in laser scan, if the laser scan intersects with the map lines
    for(int i = 0; i < num_ranges; i+=10){
      if (ray_intersects) {
        // printf("Intersects at %f,%f\n", 
        //       ray_intersection_point.x(),
        //       ray_intersection_point.y());

      // Calculate ray pose in respect to the world frame
      Eigen::Matrix3f rayWorld;
      Eigen::Vector3f odom_loc(loc(0), loc(1), 1)

      ray_world << cos(angle), -sin(angle), 0,
                  sin(angle), cos(angle) ,  0,
                      0     ,     0      ,  1; 

      // Add odom_loc to colum 2 of rayWorld matrix
      ray_world.col(2) = odom_loc;

      // Extract the rotation matrix
      Eigen::Matrix2f R = ray_world.block<2, 2>(0, 0);

      // Define a unit vector pointing in the direction of the ray
      Eigen::Vector2f direction(1, 0);

      // Rotate the direction vector
      Eigen::Vector2f rotated_direction = R * direction;

      // Define the start point of the ray line (from odom_loc)
      Eigen::Vector2f start(odom_loc(0), odom_loc(1));

      // Calculate the end point of the ray line
      Eigen::Vector2f end = start + rotated_direction;

      // Declare and initialize world ray line
      line2f ray_world_line(start, end);

      // Find intersection of world ray line with map lines
      ray_intersects = map_line.Intersection(ray_world_line, &ray_intersection_point);

      // Push back intersection points to scan variable
      scan.push_back(Vector2f(ray_intersection_point.x(), ray_intersection_point.y()));

      } else {
        printf("No intersection\n");
      }
    }
  }
}

void ParticleFilter::Update(const vector<float>& ranges,
                            float range_min,
                            float range_max,
                            float angle_min,
                            float angle_max,
                            Particle* p_ptr) {
  // Implement the update step of the particle filter here.
  // You will have to use the `GetPredictedPointCloud` to predict the expected
  // observations for each particle, and assign weights to the particles based
  // on the observation likelihood computed by relating the observation to the
  // predicted point cloud.
}

void ParticleFilter::Resample() {
  // Resample the particles, proportional to their weights.
  // The current particles are in the `particles_` variable. 
  // Create a variable to store the new particles, and when done, replace the
  // old set of particles:
  // vector<Particle> new_particles';
  // During resampling: 
  //    new_particles.push_back(...)
  // After resampling:
  // particles_ = new_particles;

  vector<Vector2f> buckets;
  float total_weight = 0;
  for (const Particle& p : particles_) {
    buckets.push_back(Vector2f(total_weight, total_weight + p.weight));
    total_weight += p.weight;
  }
  vector<Particle> new_particles;
  for (size_t j = 0; j < particles_.size(); ++j) {
    float r = rng_.UniformRandom(0, total_weight);
    for (size_t i = 0; i < particles_.size(); ++i) {
      if (r >= buckets[i].x() && r < buckets[i].y()) {
        new_particles.push_back(particles_[i]);
        break;
      }
    }
  }
  particles_ = new_particles; // does this correctly replace particles_ with new_particles?

  // You will need to use the uniform random number generator provided. For
  // example, to generate a random number between 0 and 1:
  // float x = rng_.UniformRandom(0, 1);
  // printf("Random number drawn from uniform distribution between 0 and 1: %f\n",
  //        x);
}

void ParticleFilter::ObserveLaser(const vector<float>& ranges,
                                  float range_min,
                                  float range_max,
                                  float angle_min,
                                  float angle_max) {
  // A new laser scan observation is available (in the laser frame)
  // Call the Update and Resample steps as necessary.
}

void ParticleFilter::Predict(const Vector2f& odom_loc,
                             const float odom_angle) {
  // Implement the predict step of the particle filter here.
  // A new odometry value is available (in the odom frame)
  // Implement the motion model predict step here, to propagate the particles
  // forward based on odometry.

  for (Particle& p : particles_) {
    Vector2f delta(odom_loc - p.loc());
    float delta_angle = odom_angle - p.angle;
    float sigma = k1*delta.norm() + k2*delta_angle;
    Vector2f epsilon(
        rng_.Gaussian(0, sigma),
        rng_.Gaussian(0, sigma));
    p.loc() += delta + epsilon;
    p.angle += delta_angle + rng_.Gaussian(0, k3*delta.norm() + k4*delta_angle);
  }
  prev_odom_loc_ = odom_loc;
  prev_odom_angle_ = odom_angle;


  // You will need to use the Gaussian random number generator provided. For
  // example, to generate a random number from a Gaussian with mean 0, and
  // standard deviation 2:
  // float x = rng_.Gaussian(0.0, 2.0);
  // printf("Random number drawn from Gaussian distribution with 0 mean and "
  //        "standard deviation of 2 : %f\n", x);
}

void ParticleFilter::Initialize(const string& map_file,
                                const Vector2f& loc,
                                const float angle) {
  // The "set_pose" button on the GUI was clicked, or an initialization message
  // was received from the log. Initialize the particles accordingly, e.g. with
  // some distribution around the provided location and angle.
  map_.Load(map_file);
}

void ParticleFilter::GetLocation(Eigen::Vector2f* loc_ptr, 
                                 float* angle_ptr) const {
  Vector2f& loc = *loc_ptr;
  float& angle = *angle_ptr;
  // Compute the best estimate of the robot's location based on the current set
  // of particles. The computed values must be set to the `loc` and `angle`
  // variables to return them. Modify the following assignments:
  loc = Vector2f(0, 0);
  angle = 0;
  for (const Particle& p : particles_) {
    loc += p.loc();
    angle += p.angle;
  }
  loc /= particles_.size();
  angle /= particles_.size();
}


}  // namespace particle_filter
