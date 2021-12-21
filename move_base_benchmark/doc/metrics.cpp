#include <cstdio>
#include <vector>
#include <cmath>

const double SAFETY_DISTANCE = 0.5;  // [m]

struct Data
{
  double time_stamp;
  double x;
  double y;
  double theta;
  double v;
  double omega;
  double obs_dist;
  double time_cost;
};

double computeComputationalEfficiency(const std::vector<Data>& data_log)
{
  size_t N = data_log.size();
  double mean = 0.0;

  for (size_t i = 0; i < N; i++)
  {
    mean += data_log[i].time_cost;
  }

  mean /= N;

  return mean;
}

double computeMotionEfficiency(const std::vector<Data>& data_log)
{
  double time_diff = data_log.back().time_stamp - data_log.front().time_stamp;

  return time_diff;
}

double computeVelocitySmoothness(const std::vector<Data>& data_log)
{
  size_t N = data_log.size();
  double mean = 0.0;

  for (size_t i = 0; i < N - 1; i++)
  {
    double acc = fabs(data_log[i + 1].v - data_log[i].v) / (data_log[i + 1].time_stamp - data_log[i].time_stamp);

    mean += acc;
  }

  mean /= (N - 1);

  return mean;
}

double computeSafety(const std::vector<Data>& data_log)
{
  double time_diff = data_log.back().time_stamp - data_log.front().time_stamp;
  double start_t = -1;
  double end_t = -1;
  double sum = 0.0;

  for (size_t i = 0; i < data_log.size(); i++)
  {
    if (data_log[i].obs_dist < SAFETY_DISTANCE)
    {
      if (start_t == -1)
        start_t = data_log[i].time_stamp;
      else
        end_t = data_log[i].time_stamp;
    }
    else
    {
      if (end_t != -1)
      {
        sum += (end_t - start_t);
      }

      // reset
      start_t = -1;
      end_t = -1;
    }
  }

  return sum / time_diff;
}

int main(int argc, char* argv[])
{
  if (argc != 2)
  {
    printf("Please input the path of the log file.\n");
    return 1;
  }

  FILE* file = fopen(argv[1], "r");
  if (file == NULL)
  {
    printf("Could not open the log file\n");
    return 1;
  }

  std::vector<Data> data_log;
  Data data;
  while (!feof(file))
  {
    if (fscanf(file, "%lf %lf %lf %lf %lf %lf %lf %lf\n", &data.time_stamp, &data.x, &data.y, &data.theta, &data.v,
               &data.omega, &data.obs_dist, &data.time_cost) != 8)
    {
      printf("The log file has incorrected format\n");
      return 1;
    }

    data_log.push_back(data);
  }
  fclose(file);

  double safety = computeSafety(data_log);
  printf("Safety = %.2f[%%]\n", safety * 1e2);

  double motion_efficiency = computeMotionEfficiency(data_log);
  printf("Motion efficiency = %.2f[secs]\n", motion_efficiency);

  double computational_efficiency = computeComputationalEfficiency(data_log);
  printf("Computational efficiency = %.2f[msecs]\n", computational_efficiency * 1e3);

  double velocity_smoothness = computeVelocitySmoothness(data_log);
  printf("Velocity smoothness = %.3f[m/s^2]\n", velocity_smoothness);

  return 0;
}