#include <vector>
#include <algorithm>
#include "psaf_ucbridge/utils.hpp"

double Util::Median(std::vector<int> values)
{
  std::sort(std::begin(values), std::end(values));
  int values_below_zero = Util::CalculateWrongData(values);
  int number_of_values = Util::CountValues(values);

  if ((number_of_values - values_below_zero) < 1) {
    return 0;
  } else if ((number_of_values - values_below_zero) % 2 == 0) {
    return (values[(number_of_values + values_below_zero - 2) / 2] +
           values[(number_of_values + values_below_zero) / 2]) / 2.0;
  } else {  // if ((number_of_values - values_below_zero) % 2 == 1)
    return values[(number_of_values + values_below_zero - 1) / 2];
  }
}

double Util::Average(const std::vector<int> & values)
{
  double average = 0.0;
  int values_below_zero = Util::CalculateWrongData(values);
  int number_of_values = Util::CountValues(values);

  for (int elem : values) {
    if (elem >= 0) {
      average += elem;
    }
  }
  return average / (number_of_values - values_below_zero);
}

double Util::WeightedAverage(const std::vector<int> & values, const std::vector<double> & weight)
{
  int values_below_zero = Util::CalculateWrongData(values);
  int number_of_values = Util::CountValues(values);

  if ((number_of_values - values_below_zero) < 1) {
    return 0;
  }

  double average = 0;
  auto values_iter = values.begin();
  auto weight_iter = weight.begin();

  while (values_iter != values.end()) {
    if (*values_iter > 0) {
      average += (*values_iter * *weight_iter);
      weight_iter++;
    }
    values_iter++;
  }
  return average;
}

int Util::CalculateWrongData(const std::vector<int> & values)
{
  int wrong_data_counter = 0;
  for (int elem : values) {
    if (elem < 0) {
      wrong_data_counter++;
    }
  }
  return wrong_data_counter;
}

std::vector<double> Util::Weighting(const std::vector<int> & values)
{
  int values_below_zero = Util::CalculateWrongData(values);
  int number_of_values = Util::CountValues(values);
  std::vector<double> weight(number_of_values);
  double i = 0.5;
  if ((number_of_values - values_below_zero) > 0) {
    auto iter_weight = weight.begin();
    for (auto it = values.begin(); it != values.end(); it++) {
      *iter_weight = (2 * i / ((number_of_values - values_below_zero) *
        (number_of_values - values_below_zero)));
      iter_weight++;
      i++;
    }
  }
  return weight;
}

int Util::CountValues(const std::vector<int> & values)
{
  int number_of_values = 0;
  for (int elem : values) {
    number_of_values++;
    elem += 0;
  }
  return number_of_values;
}

double Util::CalculateUsDistance(int echo_time)
{
  return 0.5 * CommandElement::kSpeedOfSound * echo_time / 1000000;
}

double Util::CalculateSpeed(const double & radius, const double & spin_time)
{
  return (2 * M_PI * radius) / spin_time;
}

double Util::CalculateAcceleration(const double & value)
{
  switch (Configuration::instance().kImuAcceleration) {
    case CommandElement::IMUAcceleration::ImuAcc2: return value / 16384;
    case CommandElement::IMUAcceleration::ImuAcc4: return value / 8192;
    case CommandElement::IMUAcceleration::ImuAcc8: return value / 4096;
    case CommandElement::IMUAcceleration::ImuAcc16: return value / 2048;
    default: return 0;
  }
}

double Util::CalculateRotationRate(const double & value)
{
  switch (Configuration::instance().kImuRotationRate) {
    case CommandElement::IMURotationRate::ImuRot250: return value / 131;
    case CommandElement::IMURotationRate::ImuRot500: return value / 65.5;
    case CommandElement::IMURotationRate::ImuRot1000: return value / 32.8;
    case CommandElement::IMURotationRate::ImuRot2000: return value / 16.4;
    default: return 0;
  }
}
