#ifndef PSAF_UCBRIDGE__UTILS_HPP_
#define PSAF_UCBRIDGE__UTILS_HPP_

#include <string_view>
#include <system_error>
#include <vector>
#include <algorithm>
#include <string>
#include "psaf_ucbridge/configuration/static_config.hpp"
#include "psaf_ucbridge/configuration/configuration.hpp"

/**
 * Util provides several methods for data processing
 */
namespace Util
{
/**
  * Calculates the median
  * @param values The vector with all values
  * @return Median
  */
double Median(std::vector<int> values);

/**
  * Calculates the average
  * @param values The vector with all values
  * @return Average
  */
double Average(const std::vector<int> & values);

/**
  * Calculates a sample weighted average
  * @param values The vector with all values
  * @param weight The vector with the weights
  * @return Weighted Average
  */
double WeightedAverage(
  const std::vector<int> & values,
  const std::vector<double> & weight);

/**
  * counts how many values below 1 there are in the vector
  * @param values The vector with all values
  * @return Number of values which are below 1
  */
int CalculateWrongData(const std::vector<int> & values);

/**
  * counts how many values there are
  * @param values The vector with the values
  * @return Number of values
  */
int CountValues(const std::vector<int> & values);

/**
  * Calculates example weights for the weighted Average
  * @param values The vector with values that should be weighted
  * @param wrong_data_counter number of values below 1
  */
std::vector<double> Weighting(const std::vector<int> & values);

/**
  * Calculates the distance out of the time measured by the ultasonic sensors
  * @param echo_time The time in microseconds
  * @return The distance the sound travelled
  */
double CalculateUsDistance(int echo_time);

/**
  * Calculates the speed out of the time measured by the hall sensor
  * @param radius The radius of the wheel in meters
  * @param spin_time The time measured by the hall sensor in seconds.
  * Within this time the wheel takes a full spin
  * @return The speed in meters per second
  */
double CalculateSpeed(const double & radius, const double & spin_time);

/**
* Calculates the acceleration out of the value sent by the imu
* @param value The value sent by the imu
* @return The acceleration in g
*/
double CalculateAcceleration(const double & value);

/**
  * Calculates the rotation rate out of the value sent by the imu
  * @param value The value sent by the imu
  * @return The rotation rate in degree per second
  */
double CalculateRotationRate(const double & value);

constexpr auto findSpace = [](unsigned char ch) {return !std::isspace(ch);};

std::string_view ltrim(std::string_view s);

std::string_view rtrim(std::string_view s);

std::string_view trim(std::string_view s);

void toLowerCase(std::string & s);

void ltrim(std::string & s);

void rtrim(std::string & s);

void trim(std::string & s);

/**
 * Checks if the std::errc err is not invalid_argument or result_out_of_range.
 * These possible errors are  returned by std::from_chsrs
 * @param err std::errc to check
 * @return true if err is valid (for std::from_chars)
 */
bool conversionValid(std::errc err);

/**
* Saturates value at min and max.
*
* @param value value which is saturates
* @param min minimum value
* @param max maximum value
* @return saturated value
*/
int saturate(int value, int min, int max);

}  // namespace Util

inline std::string_view Util::ltrim(std::string_view s)
{
  return s.substr(
    std::distance(
      s.begin(),
      std::find_if(s.begin(), s.end(), findSpace)
    )
  );
}

inline std::string_view Util::rtrim(std::string_view s)
{
  return s.substr(
    0,
    std::distance(
      s.begin(),
      std::find_if(
        s.rbegin(),
        s.rend(),
        findSpace
      ).base()
    )
  );
}

inline std::string_view Util::trim(std::string_view s)
{
  return ltrim(rtrim(s));
}

inline void Util::toLowerCase(std::string & s)
{
  std::transform(
    s.begin(), s.end(), s.begin(),
    [](unsigned char c) {return std::tolower(c);});
}

inline void Util::ltrim(std::string & s)
{
  s.erase(s.begin(), std::find_if(s.begin(), s.end(), findSpace));
}

inline void Util::rtrim(std::string & s)
{
  s.erase(std::find_if(s.rbegin(), s.rend(), findSpace).base(), s.end());
}

inline void Util::trim(std::string & s)
{
  ltrim(s);
  rtrim(s);
}

inline bool Util::conversionValid(std::errc err)
{
  return err != std::errc::invalid_argument && err != std::errc::result_out_of_range;
}

inline int Util::saturate(int value, int min, int max)
{
  return std::max(std::min(value, max), min);
}

#endif  // PSAF_UCBRIDGE__UTILS_HPP_
