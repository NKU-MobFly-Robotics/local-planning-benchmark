/******************************************************************************
 * Copyright (c) 2022, NKU Mobile & Flying Robotics Lab
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 'AS IS'
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/

#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>

struct Data {
  double timestamp;
  double x;
  double y;
  double theta;
  double v;
  double omega;
  double obs_dist;
  double time_cost;
};

double computeComputationalEfficiency(const std::vector<Data>& data_log) {
  assert(!data_log.empty());
  double mean = 0.0;
  for (size_t i = 0; i < data_log.size(); ++i) {
    mean += data_log[i].time_cost;
  }
  mean /= static_cast<double>(data_log.size());
  return mean;
}

double computeMotionEfficiency(const std::vector<Data>& data_log) {
  assert(!data_log.empty());
  return data_log.back().timestamp - data_log.front().timestamp;
}

double computeVelocitySmoothness(const std::vector<Data>& data_log) {
  assert(data_log.size() > 1);
  double mean = 0.0;
  for (size_t i = 0; i < data_log.size() - 1; ++i) {
    double acc = fabs(data_log[i + 1].v - data_log[i].v) /
                 (data_log[i + 1].timestamp - data_log[i].timestamp);
    mean += acc;
  }
  mean /= static_cast<double>(data_log.size() - 1);
  return mean;
}

double computeSafety(const std::vector<Data>& data_log,
                     const double safety_distance) {
  assert(!data_log.empty());
  double diff = data_log.back().timestamp - data_log.front().timestamp;
  double start_timestamp = -1;
  double end_timestamp = -1;
  double sum = 0.0;

  for (size_t i = 0; i < data_log.size(); ++i) {
    if (data_log[i].obs_dist < safety_distance) {
      if (start_timestamp == -1) {
        start_timestamp = data_log[i].timestamp;
      } else {
        end_timestamp = data_log[i].timestamp;
      }
    } else {
      if (end_timestamp != -1) {
        sum += (end_timestamp - start_timestamp);
      }

      // reset
      start_timestamp = -1;
      end_timestamp = -1;
    }
  }

  return sum / diff;
}

int main(int argc, char* argv[]) {
  if (argc != 3) {
    printf(
        "Please input the path to the log file and value of safety distance "
        "[m].\n");
    return 1;
  }

  FILE* file = fopen(argv[1], "r");
  if (file == NULL) {
    printf("Could not open the log file\n");
    return 1;
  }
  double safety_distance = std::atof(argv[2]);

  std::vector<Data> data_log;
  Data data;
  while (!feof(file)) {
    if (fscanf(file, "%lf %lf %lf %lf %lf %lf %lf %lf\n", &data.timestamp,
               &data.x, &data.y, &data.theta, &data.v, &data.omega,
               &data.obs_dist, &data.time_cost) != 8) {
      printf("The log file has incorrected format\n");
      return 1;
    }

    data_log.push_back(data);
  }
  fclose(file);

  double safety = computeSafety(data_log, safety_distance);
  std::cout << "Safety: " << safety * 1e2 << "%\n";

  double motion_efficiency = computeMotionEfficiency(data_log);
  std::cout << "Motion efficiency: " << motion_efficiency << " secs\n";

  double computational_efficiency = computeComputationalEfficiency(data_log);
  std::cout << "Computational efficiency: " << computational_efficiency * 1e3
            << " msecs\n";

  double velocity_smoothness = computeVelocitySmoothness(data_log);
  std::cout << "Velocity smoothness: " << velocity_smoothness << " m/s^2\n";

  return 0;
}
