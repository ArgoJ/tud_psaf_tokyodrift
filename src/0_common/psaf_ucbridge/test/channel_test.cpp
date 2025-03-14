#include <string>
#include <vector>
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "psaf_ucbridge/sensor_groups/channel.hpp"

TEST(test_Channel, test_size) {
  std::string channel_name("test123");

  Channel<uint32_t> ch1(channel_name, 1);

  ASSERT_THAT(ch1.size(), testing::Eq(4));

  Channel<uint16_t> ch2(channel_name, 1);
  ASSERT_THAT(ch2.size(), testing::Eq(2));

  Channel<uint8_t> ch3(channel_name, 1);
  ASSERT_THAT(ch3.size(), testing::Eq(1));

  Channel<int16_t> ch4(channel_name, 1);

  ASSERT_THAT(ch4.size(), testing::Eq(2));
}

TEST(test_Channel, test_readValue) {
  std::string channel_name("test123");
  Channel<uint8_t> ch3(channel_name, 1);
  std::vector<uint8_t> test_data = {1, 2, 3, 4, 5, 6, 7, 8, 9};
  for (auto it = test_data.begin(); it < test_data.end(); it++) {
    std::cout << " " << std::to_string(*it);
  }
  std::vector<uint8_t>::iterator it = test_data.begin();
  ch3.readValue(it, test_data.end());
  ASSERT_THAT(ch3.rawValue(), testing::Eq((uint8_t) 1));
  ch3.readValue(it, test_data.end());
  ASSERT_THAT(ch3.rawValue(), testing::Eq((uint8_t) 2));
  ch3.readValue(it, test_data.end());
  ASSERT_THAT(ch3.rawValue(), testing::Eq((uint8_t) 3));
  ch3.readValue(it, test_data.end());
  ASSERT_THAT(ch3.rawValue(), testing::Eq((uint8_t) 4));
  ch3.readValue(it, test_data.end());
  ASSERT_THAT(ch3.rawValue(), testing::Eq((uint8_t) 5));
  ch3.readValue(it, test_data.end());
  ASSERT_THAT(ch3.rawValue(), testing::Eq((uint8_t) 6));
  ch3.readValue(it, test_data.end());
  ASSERT_THAT(ch3.rawValue(), testing::Eq((uint8_t) 7));
  ch3.readValue(it, test_data.end());
  ASSERT_THAT(ch3.rawValue(), testing::Eq((uint8_t) 8));
  ch3.readValue(it, test_data.end());
  ASSERT_THAT(ch3.rawValue(), testing::Eq((uint8_t) 9));
  ch3.readValue(it, test_data.end());
  ASSERT_THAT(ch3.rawValue(), testing::Eq((uint8_t) 0));
}
