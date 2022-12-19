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
#include <fstream>
#include <iostream>
#include <sstream>
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

double ComputeComputationalEfficiency(const std::vector<Data>& data_log) {
  assert(!data_log.empty());
  double mean = 0.0;
  for (size_t i = 0; i < data_log.size(); ++i) {
    mean += data_log[i].time_cost;
  }
  mean /= static_cast<double>(data_log.size());
  return mean;
}

double ComputeMotionEfficiency(const std::vector<Data>& data_log) {
  assert(!data_log.empty());
  return data_log.back().timestamp - data_log.front().timestamp;
}

double ComputeVelocitySmoothness(const std::vector<Data>& data_log) {
  assert(data_log.size() > 1);
  double mean = 0.0;
  for (size_t i = 0; i < data_log.size() - 1; ++i) {
    mean += fabs(data_log[i + 1].v - data_log[i].v) /
            (data_log[i + 1].timestamp - data_log[i].timestamp);
  }
  mean /= static_cast<double>(data_log.size() - 1);
  return mean;
}

double ComputeSafety(const std::vector<Data>& data_log,
                     const double safety_distance) {
  assert(!data_log.empty());
  double diff = data_log.back().timestamp - data_log.front().timestamp;
  int start_index = -1;
  int end_index = -1;
  double sum = 0.0;

  for (int i = 0; i < static_cast<int>(data_log.size()); ++i) {
    if (data_log[i].obs_dist < safety_distance) {
      if (start_index == -1) {
        start_index = i;
      } else {
        end_index = i;
      }
    } else {
      if (end_index != -1) {
        sum += data_log[end_index].timestamp - data_log[start_index].timestamp;
      }

      // reset
      start_index = -1;
      end_index = -1;
    }
  }
  return sum / diff;
}

int main(int argc, char* argv[]) {
  if (argc != 3) {
    std::cout << "Please input log path and the value of safety distance\n";
    return 1;
  }

  std::ifstream fin(argv[1]);
  if (!fin.is_open()) {
    std::cout << "Failed to open " << argv[1] << "\n";
    return 1;
  }

  std::vector<Data> data_log;
  std::string str;
  std::vector<std::string> str_list;
  while (std::getline(fin, str)) {
    str_list.clear();
    std::stringstream ss(str);
    std::string tmp;
    while (std::getline(ss, tmp, ' ')) {
      str_list.push_back(tmp);
    }
    assert(str_list.size() == 8U);
    Data data;
    data.timestamp = std::stod(str_list[0]);
    data.x = std::stod(str_list[1]);
    data.y = std::stod(str_list[2]);
    data.theta = std::stod(str_list[3]);
    data.v = std::stod(str_list[4]);
    data.omega = std::stod(str_list[5]);
    data.obs_dist = std::stod(str_list[6]);
    data.time_cost = std::stod(str_list[7]);
    data_log.push_back(std::move(data));
  }
  fin.close();

  const double safety_distance = std::atof(argv[2]);
  const double safety = ComputeSafety(data_log, safety_distance);
  std::cout << "Safety: " << safety * 1e2 << "%\n";

  const double motion_efficiency = ComputeMotionEfficiency(data_log);
  std::cout << "Motion efficiency: " << motion_efficiency << " secs\n";

  const double computational_efficiency =
      ComputeComputationalEfficiency(data_log);
  std::cout << "Computational efficiency: " << computational_efficiency * 1e3
            << " msecs\n";

  const double velocity_smoothness = ComputeVelocitySmoothness(data_log);
  std::cout << "Velocity smoothness: " << velocity_smoothness << " m/s^2\n";

  return 0;
}
