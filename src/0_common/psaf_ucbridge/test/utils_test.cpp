#include <vector>
#include <string>
#include "gtest/gtest.h"
#include "psaf_ucbridge/utils.hpp"

/*
 * Tests the calculation of the median
 */
TEST(test_utils, test_median) {
  // Some example values to calculate the median for
  std::vector<int> vec_values1 = {1, 2, 3, 4, 5, 6};
  std::vector<int> vec_values2 = {1, 2, 3, 4, 5, 6, 7};
  std::vector<int> vec_values3 = {5, 7, 2, 8, -100, 12};
  std::vector<int> vec_values4 = {5, 7, 2, 8, -34, 12, 6, -12};
  std::vector<int> vec_values5 = {0, 0, 0, 0, 0, 0};
  std::vector<int> vec_values6 = {-1, -2, -3, -4, -5};

  ASSERT_EQ(Util::Median(vec_values1), 3.5);
  ASSERT_EQ(Util::Median(vec_values2), 4);
  ASSERT_EQ(Util::Median(vec_values3), 7);
  ASSERT_EQ(Util::Median(vec_values4), 6.5);
  ASSERT_EQ(Util::Median(vec_values5), 0);
  ASSERT_EQ(Util::Median(vec_values6), 0);
}

/*
 * Tests the calculation of the average
 */
TEST(test_utils, test_average) {
  // Some example values to calculate the average for
  std::vector<int> vec_values1 = {1, 2, 3, 4, 5, 6};
  std::vector<int> vec_values2 = {1, 2, 3, 4, 5, 6, 7};
  std::vector<int> vec_values3 = {5, 7, 2, 8, -100, 12};
  std::vector<int> vec_values4 = {5, 7, 2, 8, -34, 12, 6, -12};
  std::vector<int> vec_values5 = {0, 0, 0, 0, 0, 0};

  ASSERT_EQ(Util::Average(vec_values1), 21.0 / 6.0);
  ASSERT_EQ(Util::Average(vec_values2), 28.0 / 7.0);
  ASSERT_EQ(Util::Average(vec_values3), 34.0 / 5.0);
  ASSERT_EQ(Util::Average(vec_values4), 40.0 / 6.0);
  ASSERT_EQ(Util::Average(vec_values5), 0.0 / 6.0);
}

/*
 * Tests the calculation of wrong data = values below 0
 */
TEST(test_utils, test_calculate_wrong_data) {
  std::vector<int> vec_values = {1, 0, -1};

  ASSERT_EQ(Util::CalculateWrongData(vec_values), 1);
}

/*
 * Tests the calculation of the amount of values in the vector
 */
TEST(test_utils, test_count_values) {
  std::vector<int> vec_values = {0, 1, 2, 3, 4};

  ASSERT_EQ(Util::CountValues(vec_values), 5);
}

/*
 * Tests the calculation of the weightings of values
 */
TEST(test_utils, test_weighting) {
  std::vector<int> vec_values = {1, 2, 3, 4, 5};
  std::vector<double> vec_expected_weights = {0.04, 0.12, 0.2, 0.28, 0.36};

  auto vec_weight = Util::Weighting(vec_values);
  ASSERT_EQ(vec_expected_weights, vec_weight);

  std::vector<int> vec_values2 = {1};
  std::vector<double> vec_expected_weights2 = {1.0};

  auto vec_weight2 = Util::Weighting(vec_values2);
  ASSERT_EQ(vec_expected_weights2, vec_weight2);
}

/*
 * Tests the calculation of the weighted average
 */
TEST(test_utils, test_weighted_average) {
  std::vector<int> vec_values1 = {1, 2, 3, 4, 5};
  auto vec_weight1 = Util::Weighting(vec_values1);
  ASSERT_EQ(Util::WeightedAverage(vec_values1, vec_weight1), 3.8);

  std::vector<int> vec_values2 = {5, 4, 3, 2, 1};
  auto vec_weight2 = Util::Weighting(vec_values2);
  ASSERT_EQ(Util::WeightedAverage(vec_values2, vec_weight2), 2.2);

  std::vector<int> vec_values3 = {-1};
  auto vec_weight3 = Util::Weighting(vec_values3);
  ASSERT_EQ(Util::WeightedAverage(vec_values3, vec_weight3), 0);
}

/*
 * Tests the calculation of the distance based on ultrasonic sensor data
 */
TEST(test_utils, test_calculate_us_distance) {
  ASSERT_EQ(Util::CalculateUsDistance(20000), 3.432);
}

/*
 * Tests the calculation of the driving speed
 */
TEST(test_utils, test_calculate_speed) {
  ASSERT_GE(Util::CalculateSpeed(0.05, 0.25), 1.25);
  ASSERT_LE(Util::CalculateSpeed(0.05, 0.25), 1.26);
}

/*
 * Tests the calculation of the acceleration
 */
TEST(test_utils, test_calculte_acceleration) {
  ASSERT_GE(Util::CalculateAcceleration(10000), 1.2);
  ASSERT_LE(Util::CalculateAcceleration(10000), 1.3);
}

/**
 * Tests the calculation of the rotation rate
 */
TEST(test_utils, test_calculate_rotation_rate) {
  ASSERT_GE(Util::CalculateRotationRate(10000), 152.6);
  ASSERT_LE(Util::CalculateRotationRate(10000), 152.7);
}

/*
 * Tests the ltrim
 */
TEST(test_utils, test_ltrim) {
  std::string_view test(" abc123 abc");
  std::string_view result = Util::ltrim(test);
  ASSERT_EQ(result, "abc123 abc");
  std::string test1(" abc123 abc");
  Util::ltrim(test1);
  ASSERT_EQ(test1, "abc123 abc");

  std::string_view test2("abc123 abc ");
  std::string_view result1 = Util::ltrim(test2);
  ASSERT_EQ(result1, "abc123 abc ");
  std::string test3("abc123 abc ");
  Util::ltrim(test3);
  ASSERT_EQ(test3, "abc123 abc ");
}

/*
 * Test the rtrim
 * */
TEST(test_utils, test_rtrim) {
  std::string_view test("abc123 abc ");
  std::string_view result = Util::rtrim(test);
  ASSERT_EQ(result, "abc123 abc");
  std::string test1("abc123 abc ");
  Util::rtrim(test1);
  ASSERT_EQ(test1, "abc123 abc");

  std::string_view test2(" abc123 abc");
  std::string_view result1 = Util::rtrim(test2);
  ASSERT_EQ(result1, " abc123 abc");
  std::string test3(" abc123 abc");
  Util::rtrim(test3);
  ASSERT_EQ(test3, " abc123 abc");
}

TEST(test_utils, test_trim) {
  std::string_view test(" abc123 abc ");
  std::string_view result = Util::trim(test);
  ASSERT_EQ(result, "abc123 abc");
  std::string test1(" abc123 abc ");
  Util::trim(test1);
  ASSERT_EQ(test1, "abc123 abc");
}

TEST(test_utils, test_toLowerCase) {
  std::string test("AABBcCDd123");
  Util::toLowerCase(test);
  ASSERT_EQ(test, "aabbccdd123");
}

TEST(test_utils, test_conversionValid) {
  std::errc test = std::errc::invalid_argument;
  ASSERT_FALSE(Util::conversionValid(test));
  test = std::errc::result_out_of_range;
  ASSERT_FALSE(Util::conversionValid(test));
  test = std::errc::address_in_use;
  ASSERT_TRUE(Util::conversionValid(test));
}

TEST(test_utils, test_saturate) {
  int test = 2;
  int max = 3;
  int min = 1;
  ASSERT_EQ(Util::saturate(test, min, max), 2);
  test = 0;
  ASSERT_EQ(Util::saturate(test, min, max), 1);
  test = 4;
  ASSERT_EQ(Util::saturate(test, min, max), 3);
}
