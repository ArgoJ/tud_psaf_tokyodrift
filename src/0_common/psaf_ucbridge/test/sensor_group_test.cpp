#include <string>
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "psaf_ucbridge/communication/command.hpp"
#include "psaf_ucbridge/sensor_groups/sensor_group.hpp"
#include "psaf_ucbridge/configuration/static_config.hpp"

static ComDAQ message;
static ComDAQ message1;
static ComDAQ message2;
static ComDAQ message3;
static ComDAQ message4;
static ComDAQ message5;
static ComDAQ message6;


class Environment : public testing::Environment
{
public:
  ~Environment() override {}

  // Override this to define how to set up the environment.
  void SetUp() override
  {
    // Cmd for IMU
    message.group(0)
    .channel(CommandElement::ChannelEnum::AX)
    .channel(CommandElement::ChannelEnum::AY)
    .channel(CommandElement::ChannelEnum::AZ)
    .channel(CommandElement::ChannelEnum::GX)
    .channel(CommandElement::ChannelEnum::GY)
    .channel(CommandElement::ChannelEnum::GZ)
    .ts(10)
    .avg();

    // Cmd for US sensor
    message1.group(1)
    .channel(CommandElement::ChannelEnum::USF)
    .channel(CommandElement::ChannelEnum::USL)
    .channel(CommandElement::ChannelEnum::USR)
    .all();

    // Cmd for Mag
    message2.group(2)
    .channel(CommandElement::ChannelEnum::MX)
    .channel(CommandElement::ChannelEnum::MY)
    .channel(CommandElement::ChannelEnum::MZ);

    // Cmd for Dt
    message3.group(3)
    .channel(CommandElement::ChannelEnum::HALL_CNT)
    .channel(CommandElement::ChannelEnum::HALL_DT);

    // Cmd for Dt8
    message4.group(4)
    .channel(CommandElement::ChannelEnum::HALL_DT8);

    // Cmd for enc_step
    message5.group(5)
    .channel(CommandElement::ChannelEnum::ENC_STEPS)
    .channel(CommandElement::ChannelEnum::TICS);

    // Cmd for enc_cm
    message6.group(6)
    .channel(CommandElement::ChannelEnum::ENC_CMS);

    // Cmd for
  }

  // Override this to define how to tear down the environment.
  void TearDown() override {}
};

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  testing::AddGlobalTestEnvironment(new Environment);
  return RUN_ALL_TESTS();
}
// Test IMU group
TEST(test_ImuGroup, test_missing_callback) {
  message.encoding(CommandElement::DAQEncoding::HEX);
  SensorGroup test_obj(message);
  std::string test("1111AAAA1111AAAA1111aaaa");
  ASSERT_FALSE(test_obj.publishData(test));
}

TEST(test_ImuGroup, test_hex) {
  float data[6];
  message.encoding(CommandElement::DAQEncoding::HEX);
  SensorGroup test_obj(message);
  std::string test("1111AAAA1111AAAA1111aaaa");
  test_obj.registerCallback(
    [&data](SensorGroup & grp) {
      data[0] = grp[0];
      data[1] = grp[1];
      data[2] = grp[2];
      data[3] = grp[3];
      data[4] = grp[4];
      data[5] = grp[5];
    });
  ASSERT_TRUE(test_obj.publishData(test));

  ASSERT_THAT(
    data, testing::ElementsAre(
      static_cast<float>( 4369 * CommandElement::kAxConversionFactor),
      static_cast<float>( -21846 * CommandElement::kAyConversionFactor),
      static_cast<float>( 4369 * CommandElement::kAzConversionFactor),
      static_cast<float>( -21846 * CommandElement::kWxConversionFactor),
      static_cast<float>( 4369 * CommandElement::kWyConversionFactor),
      static_cast<float>( -21846 * CommandElement::kWzConversionFactor)));
}

TEST(test_ImuGroup, test_ASCII) {
  float data[6];
  message.encoding(CommandElement::DAQEncoding::ASCII);
  SensorGroup test_obj(message);
  std::string test("4369 | -21846 | 4369 | -21846 | 4369 | -21846");
  test_obj.registerCallback(
    [&data](SensorGroup & grp) {
      data[0] = grp[0];
      data[1] = grp[1];
      data[2] = grp[2];
      data[3] = grp[3];
      data[4] = grp[4];
      data[5] = grp[5];
    });
  ASSERT_TRUE(test_obj.publishData(test));

  ASSERT_THAT(
    data, testing::ElementsAre(
      static_cast<float>( 4369 * CommandElement::kAxConversionFactor),
      static_cast<float>( -21846 * CommandElement::kAyConversionFactor),
      static_cast<float>( 4369 * CommandElement::kAzConversionFactor),
      static_cast<float>( -21846 * CommandElement::kWxConversionFactor),
      static_cast<float>( 4369 * CommandElement::kWyConversionFactor),
      static_cast<float>( -21846 * CommandElement::kWzConversionFactor)));
}

TEST(test_ImuGroup, test_BASE64) {
  float data[6];
  message.encoding(CommandElement::DAQEncoding::BASE64);
  SensorGroup test_obj(message);
  std::string test("ERGqqhERqqoREaqq");
  test_obj.registerCallback(
    [&data](SensorGroup & grp) {
      data[0] = grp[0];
      data[1] = grp[1];
      data[2] = grp[2];
      data[3] = grp[3];
      data[4] = grp[4];
      data[5] = grp[5];
    });
  ASSERT_TRUE(test_obj.publishData(test));

  ASSERT_THAT(
    data, testing::ElementsAre(
      static_cast<float>( 4369 * CommandElement::kAxConversionFactor),
      static_cast<float>( -21846 * CommandElement::kAyConversionFactor),
      static_cast<float>( 4369 * CommandElement::kAzConversionFactor),
      static_cast<float>( -21846 * CommandElement::kWxConversionFactor),
      static_cast<float>( 4369 * CommandElement::kWyConversionFactor),
      static_cast<float>( -21846 * CommandElement::kWzConversionFactor)));
}

TEST(test_ImuGroup, test_unbind_callback) {
  float data[6];
  message.encoding(CommandElement::DAQEncoding::BASE64);
  SensorGroup test_obj(message);
  std::string test("ERGqqhERqqoREaqq");
  test_obj.registerCallback(
    [&data](SensorGroup & grp) {
      data[0] = grp[0];
      data[1] = grp[1];
      data[2] = grp[2];
      data[3] = grp[3];
      data[4] = grp[4];
      data[5] = grp[5];
    });
  ASSERT_TRUE(test_obj.publishData(test));

  ASSERT_THAT(
    data, testing::ElementsAre(
      static_cast<float>( 4369 * CommandElement::kAxConversionFactor),
      static_cast<float>( -21846 * CommandElement::kAyConversionFactor),
      static_cast<float>( 4369 * CommandElement::kAzConversionFactor),
      static_cast<float>( -21846 * CommandElement::kWxConversionFactor),
      static_cast<float>( 4369 * CommandElement::kWyConversionFactor),
      static_cast<float>( -21846 * CommandElement::kWzConversionFactor)));
  test_obj.unregisterCallback();
  ASSERT_FALSE(test_obj.publishData(test));
}

TEST(test_ImuGroup, test_others) {
  message.encoding(CommandElement::DAQEncoding::BASE64);
  SensorGroup test_obj(message);
  ASSERT_EQ(test_obj.numberOfChannels(), 6);
  ASSERT_EQ(test_obj[0], test_obj(0));
  ASSERT_EQ(test_obj.getSensorGroupNo(), 0);
}

// test US group
TEST(test_USGroup, test_missing_callbcak) {
  message1.encoding(CommandElement::DAQEncoding::HEX);
  SensorGroup test_obj(message1);
  std::string test("1111AAAA1111");
  ASSERT_FALSE(test_obj.publishData(test));
}

TEST(test_USGroup, test_hex) {
  float data[3];
  message1.encoding(CommandElement::DAQEncoding::HEX);
  SensorGroup test_obj(message1);
  std::string test("1111AAAA1111");
  test_obj.registerCallback(
    [&data](SensorGroup & grp) {
      data[0] = grp[0];
      data[1] = grp[1];
      data[2] = grp[2];
    });
  ASSERT_TRUE(test_obj.publishData(test));

  ASSERT_THAT(
    data, testing::ElementsAre(
      static_cast<float>( 4369 * CommandElement::kUltrasonicConversionFactor),
      static_cast<float>( 43690 * CommandElement::kUltrasonicConversionFactor),
      static_cast<float>( 4369 * CommandElement::kUltrasonicConversionFactor)));
}

TEST(test_USGroup, test_BASE64) {
  float data[3];
  message1.encoding(CommandElement::DAQEncoding::BASE64);
  SensorGroup test_obj(message1);
  std::string test("ERGqqhER");
  test_obj.registerCallback(
    [&data](SensorGroup & grp) {
      data[0] = grp[0];
      data[1] = grp[1];
      data[2] = grp[2];
    });
  ASSERT_TRUE(test_obj.publishData(test));

  ASSERT_THAT(
    data, testing::ElementsAre(
      static_cast<float>( 4369 * CommandElement::kUltrasonicConversionFactor),
      static_cast<float>( 43690 * CommandElement::kUltrasonicConversionFactor),
      static_cast<float>( 4369 * CommandElement::kUltrasonicConversionFactor)));
}

TEST(test_USGroup, test_ASCII) {
  float data[3];
  message1.encoding(CommandElement::DAQEncoding::ASCII);
  SensorGroup test_obj(message1);
  std::string test("4369 | 43690 | 4369");
  test_obj.registerCallback(
    [&data](SensorGroup & grp) {
      data[0] = grp[0];
      data[1] = grp[1];
      data[2] = grp[2];
    });
  ASSERT_TRUE(test_obj.publishData(test));

  ASSERT_THAT(
    data, testing::ElementsAre(
      static_cast<float>( 4369 * CommandElement::kUltrasonicConversionFactor),
      static_cast<float>( 43690 * CommandElement::kUltrasonicConversionFactor),
      static_cast<float>( 4369 * CommandElement::kUltrasonicConversionFactor)));
}

TEST(test_USGroup, test_unbind_callback) {
  float data[3];
  message1.encoding(CommandElement::DAQEncoding::BASE64);
  SensorGroup test_obj(message1);
  std::string test("ERGqqhER");
  test_obj.registerCallback(
    [&data](SensorGroup & grp) {
      data[0] = grp[0];
      data[1] = grp[1];
      data[2] = grp[2];
    });
  ASSERT_TRUE(test_obj.publishData(test));

  ASSERT_THAT(
    data, testing::ElementsAre(
      static_cast<float>( 4369 * CommandElement::kUltrasonicConversionFactor),
      static_cast<float>( 43690 * CommandElement::kUltrasonicConversionFactor),
      static_cast<float>( 4369 * CommandElement::kUltrasonicConversionFactor)));
  test_obj.unregisterCallback();
  ASSERT_FALSE(test_obj.publishData(test));
}

TEST(test_USGroup, test_others) {
  message1.encoding(CommandElement::DAQEncoding::BASE64);
  SensorGroup test_obj(message1);
  ASSERT_EQ(test_obj.numberOfChannels(), 3);
  ASSERT_EQ(test_obj[0], test_obj(0));
  ASSERT_EQ(test_obj.getSensorGroupNo(), 1);
}

// test Mag Group
TEST(test_MagGroup, test_missing_callback) {
  message2.encoding(CommandElement::DAQEncoding::HEX);
  SensorGroup test_obj(message2);
  std::string test("1111AAAA1111");
  ASSERT_FALSE(test_obj.publishData(test));
}

TEST(test_MagGroup, test_hex) {
  float data[3];
  message2.encoding(CommandElement::DAQEncoding::HEX);
  SensorGroup test_obj(message2);
  std::string test("1111AAAA1111");
  test_obj.registerCallback(
    [&data](SensorGroup & grp) {
      data[0] = grp[0];
      data[1] = grp[1];
      data[2] = grp[2];
    });
  ASSERT_TRUE(test_obj.publishData(test));

  ASSERT_THAT(
    data, testing::ElementsAre(
      static_cast<float>( 4369 * CommandElement::kMxConversionFactor),
      static_cast<float>( -21846 * CommandElement::kMyConversionFactor),
      static_cast<float>( 4369 * CommandElement::kMzConversionFactor)));
}

TEST(test_MagGroup, test_BASE64) {
  float data[3];
  message2.encoding(CommandElement::DAQEncoding::BASE64);
  SensorGroup test_obj(message2);
  std::string test("ERGqqhER");
  test_obj.registerCallback(
    [&data](SensorGroup & grp) {
      data[0] = grp[0];
      data[1] = grp[1];
      data[2] = grp[2];
    });
  ASSERT_TRUE(test_obj.publishData(test));

  ASSERT_THAT(
    data, testing::ElementsAre(
      static_cast<float>( 4369 * CommandElement::kMxConversionFactor),
      static_cast<float>( -21846 * CommandElement::kMyConversionFactor),
      static_cast<float>( 4369 * CommandElement::kMzConversionFactor)));
}

TEST(test_MagGroup, test_ASCII) {
  float data[3];
  message2.encoding(CommandElement::DAQEncoding::ASCII);
  SensorGroup test_obj(message2);
  std::string test("4369 | 43690 | 4369");
  test_obj.registerCallback(
    [&data](SensorGroup & grp) {
      data[0] = grp[0];
      data[1] = grp[1];
      data[2] = grp[2];
    });
  ASSERT_TRUE(test_obj.publishData(test));

  ASSERT_THAT(
    data, testing::ElementsAre(
      static_cast<float>( 4369 * CommandElement::kMxConversionFactor),
      static_cast<float>( -21846 * CommandElement::kMyConversionFactor),
      static_cast<float>( 4369 * CommandElement::kMzConversionFactor)));
}

TEST(test_MagGroup, test_unbind_callback) {
  float data[3];
  message2.encoding(CommandElement::DAQEncoding::ASCII);
  SensorGroup test_obj(message2);
  std::string test("4369 | 43690 | 4369");
  test_obj.registerCallback(
    [&data](SensorGroup & grp) {
      data[0] = grp[0];
      data[1] = grp[1];
      data[2] = grp[2];
    });
  ASSERT_TRUE(test_obj.publishData(test));

  ASSERT_THAT(
    data, testing::ElementsAre(
      static_cast<float>( 4369 * CommandElement::kMxConversionFactor),
      static_cast<float>( -21846 * CommandElement::kMyConversionFactor),
      static_cast<float>( 4369 * CommandElement::kMzConversionFactor)));
  test_obj.unregisterCallback();
  ASSERT_FALSE(test_obj.publishData(test));
}

TEST(test_MagGroup, test_others) {
  message2.encoding(CommandElement::DAQEncoding::BASE64);
  SensorGroup test_obj(message2);
  ASSERT_EQ(test_obj.numberOfChannels(), 3);
  ASSERT_EQ(test_obj[0], test_obj(0));
  ASSERT_EQ(test_obj.getSensorGroupNo(), 2);
}

// test DT Group
TEST(test_DtGroup, test_missing_callback) {
  message3.encoding(CommandElement::DAQEncoding::HEX);
  SensorGroup test_obj(message3);
  std::string test("11AAAA");
  ASSERT_FALSE(test_obj.publishData(test));
}

TEST(test_DtGroup, test_hex) {
  float data[2];
  message3.encoding(CommandElement::DAQEncoding::HEX);
  SensorGroup test_obj(message3);
  std::string test("11AAAA");
  test_obj.registerCallback(
    [&data](SensorGroup & grp) {
      data[0] = grp[0];
      data[1] = grp[1];
    });
  ASSERT_TRUE(test_obj.publishData(test));

  ASSERT_THAT(
    data, testing::ElementsAre(
      static_cast<float>( 17 * 1.0f),
      static_cast<float>( 43690 * CommandElement::kDtConversionFactor)));
}

TEST(test_DtGroup, test_BASE64) {
  float data[2];
  message3.encoding(CommandElement::DAQEncoding::BASE64);
  SensorGroup test_obj(message3);
  std::string test("Eaqq");
  test_obj.registerCallback(
    [&data](SensorGroup & grp) {
      data[0] = grp[0];
      data[1] = grp[1];
    });
  ASSERT_TRUE(test_obj.publishData(test));

  ASSERT_THAT(
    data, testing::ElementsAre(
      static_cast<float>( 17 * 1.0f),
      static_cast<float>( 43690 * CommandElement::kDtConversionFactor)));
}

TEST(test_DtGroup, test_ASCII) {
  float data[2];
  message3.encoding(CommandElement::DAQEncoding::ASCII);
  SensorGroup test_obj(message3);
  std::string test("17 | 43690");
  test_obj.registerCallback(
    [&data](SensorGroup & grp) {
      data[0] = grp[0];
      data[1] = grp[1];
    });
  ASSERT_TRUE(test_obj.publishData(test));

  ASSERT_THAT(
    data, testing::ElementsAre(
      static_cast<float>( 17 * 1.0f),
      static_cast<float>( 43690 * CommandElement::kDtConversionFactor)));
}

TEST(test_DtGroup, test_unbind_callback) {
  float data[2];
  message3.encoding(CommandElement::DAQEncoding::ASCII);
  SensorGroup test_obj(message3);
  std::string test("17 | 43690");
  test_obj.registerCallback(
    [&data](SensorGroup & grp) {
      data[0] = grp[0];
      data[1] = grp[1];
    });
  ASSERT_TRUE(test_obj.publishData(test));

  ASSERT_THAT(
    data, testing::ElementsAre(
      static_cast<float>( 17 * 1.0f),
      static_cast<float>( 43690 * CommandElement::kDtConversionFactor)));

  test_obj.unregisterCallback();
  ASSERT_FALSE(test_obj.publishData(test));
}

TEST(test_DtGroup, test_others) {
  message3.encoding(CommandElement::DAQEncoding::BASE64);
  SensorGroup test_obj(message3);
  ASSERT_EQ(test_obj.numberOfChannels(), 2);
  ASSERT_EQ(test_obj[0], test_obj(0));
  ASSERT_EQ(test_obj.getSensorGroupNo(), 3);
}

// test DT8 Group
TEST(test_Dt8Group, test_missing_callback) {
  message4.encoding(CommandElement::DAQEncoding::HEX);
  SensorGroup test_obj(message4);
  std::string test("1111");
  ASSERT_FALSE(test_obj.publishData(test));
}

TEST(test_Dt8Group, test_hex) {
  float data;
  message4.encoding(CommandElement::DAQEncoding::HEX);
  SensorGroup test_obj(message4);
  std::string test("1111");
  test_obj.registerCallback(
    [&data](SensorGroup & grp) {
      data = grp[0];
    });
  ASSERT_TRUE(test_obj.publishData(test));
  ASSERT_EQ(data, 4369 * CommandElement::kDt8ConversionFactor);
}

TEST(test_Dt8Group, test_BASE64) {
  float data;
  message4.encoding(CommandElement::DAQEncoding::BASE64);
  SensorGroup test_obj(message4);
  std::string test("ERE=");
  test_obj.registerCallback(
    [&data](SensorGroup & grp) {
      data = grp[0];
    });
  ASSERT_TRUE(test_obj.publishData(test));
  ASSERT_EQ(data, 4369 * CommandElement::kDt8ConversionFactor);
}

TEST(test_Dt8Group, test_ASCII) {
  float data;
  message4.encoding(CommandElement::DAQEncoding::ASCII);
  SensorGroup test_obj(message4);
  std::string test("4369");
  test_obj.registerCallback(
    [&data](SensorGroup & grp) {
      data = grp[0];
    });
  ASSERT_TRUE(test_obj.publishData(test));
  ASSERT_EQ(data, 4369 * CommandElement::kDt8ConversionFactor);
}

TEST(test_Dt8Group, test_unbind_callback) {
  float data;
  message4.encoding(CommandElement::DAQEncoding::ASCII);
  SensorGroup test_obj(message4);
  std::string test("4369");
  test_obj.registerCallback(
    [&data](SensorGroup & grp) {
      data = grp[0];
    });
  ASSERT_TRUE(test_obj.publishData(test));
  ASSERT_EQ(data, 4369 * CommandElement::kDt8ConversionFactor);
  test_obj.unregisterCallback();
  ASSERT_FALSE(test_obj.publishData(test));
}

TEST(test_Dt8Group, test_others) {
  message4.encoding(CommandElement::DAQEncoding::BASE64);
  SensorGroup test_obj(message4);
  ASSERT_EQ(test_obj.numberOfChannels(), 1);
  ASSERT_EQ(test_obj[0], test_obj(0));
  ASSERT_EQ(test_obj.getSensorGroupNo(), 4);
}

// test enc_step Group
TEST(test_enc_stepGroup, test_missing_callback) {
  message5.encoding(CommandElement::DAQEncoding::HEX);
  SensorGroup test_obj(message5);
  std::string test("1111");
  ASSERT_FALSE(test_obj.publishData(test));
}

TEST(test_enc_stepGroup, test_hex) {
  float data;
  message5.encoding(CommandElement::DAQEncoding::HEX);
  SensorGroup test_obj(message5);
  std::string test("1111");
  test_obj.registerCallback(
    [&data](SensorGroup & grp) {
      data = grp[0];
    });
  ASSERT_TRUE(test_obj.publishData(test));
  ASSERT_EQ(data, 4369 * 1.0);
}

// TODO(...): Test wieder reparieren (Issue #29)
// TEST(test_enc_stepGroup, test_BASE64) {
//  float data;
//  message5.encoding(CommandElement::DAQEncoding::BASE64);
//  SensorGroup test_obj(message5);
//  std::string test("ERE=");
//  test_obj.registerCallback(
//    [&data](SensorGroup & grp) {
//      data = grp[0];
//    });
//  ASSERT_TRUE(test_obj.publishData(test));
//  ASSERT_EQ(data, 4369 * 1.0);
//}

TEST(test_enc_stepGroup, test_ASCII) {
  float data;
  message5.encoding(CommandElement::DAQEncoding::ASCII);
  SensorGroup test_obj(message5);
  std::string test("4369");
  test_obj.registerCallback(
    [&data](SensorGroup & grp) {
      data = grp[0];
    });
  ASSERT_TRUE(test_obj.publishData(test));
  ASSERT_EQ(data, 4369 * 1.0);
}

TEST(test_enc_stepGroup, test_unbind_callback) {
  float data[2];
  message5.encoding(CommandElement::DAQEncoding::ASCII);
  SensorGroup test_obj(message5);
  std::string test("4369 115200");
  test_obj.registerCallback(
    [&data](SensorGroup & grp) {
      data[0] = grp[0];
      data[1] = grp[1];
    });
  ASSERT_TRUE(test_obj.publishData(test));
  ASSERT_EQ(data[0], 4369 * 1.0);
  ASSERT_EQ(data[1], 115200 * 1.0);
  test_obj.unregisterCallback();
  ASSERT_FALSE(test_obj.publishData(test));
}

TEST(test_enc_stepGroup, test_others) {
  message5.encoding(CommandElement::DAQEncoding::BASE64);
  SensorGroup test_obj(message5);
  ASSERT_EQ(test_obj.numberOfChannels(), 2);
  ASSERT_EQ(test_obj[0], test_obj(0));
  ASSERT_EQ(test_obj.getSensorGroupNo(), 5);
}

// test enc_cm Group
TEST(test_enc_cmGroup, test_missing_callback) {
  message6.encoding(CommandElement::DAQEncoding::HEX);
  SensorGroup test_obj(message6);
  std::string test("1111");
  ASSERT_FALSE(test_obj.publishData(test));
}

TEST(test_enc_cmGroup, test_hex) {
  float data;
  message6.encoding(CommandElement::DAQEncoding::HEX);
  SensorGroup test_obj(message6);
  std::string test("1111");
  test_obj.registerCallback(
    [&data](SensorGroup & grp) {
      data = grp[0];
    });
  ASSERT_TRUE(test_obj.publishData(test));
  ASSERT_EQ(data, 4369 * 1.0);
}

TEST(test_enc_cmGroup, test_BASE64) {
  float data;
  message6.encoding(CommandElement::DAQEncoding::BASE64);
  SensorGroup test_obj(message6);
  std::string test("ERE=");
  test_obj.registerCallback(
    [&data](SensorGroup & grp) {
      data = grp[0];
    });
  ASSERT_TRUE(test_obj.publishData(test));
  ASSERT_EQ(data, 4369 * 1.0);
}

TEST(test_enc_cmGroup, test_ASCII) {
  float data;
  message6.encoding(CommandElement::DAQEncoding::ASCII);
  SensorGroup test_obj(message6);
  std::string test("4369");
  test_obj.registerCallback(
    [&data](SensorGroup & grp) {
      data = grp[0];
    });
  ASSERT_TRUE(test_obj.publishData(test));
  ASSERT_EQ(data, 4369 * 1.0);
}

TEST(test_enc_cmGroup, test_unbind_callback) {
  float data;
  message6.encoding(CommandElement::DAQEncoding::ASCII);
  SensorGroup test_obj(message6);
  std::string test("4369");
  test_obj.registerCallback(
    [&data](SensorGroup & grp) {
      data = grp[0];
    });
  ASSERT_TRUE(test_obj.publishData(test));
  ASSERT_EQ(data, 4369 * 1.0);
  test_obj.unregisterCallback();
  ASSERT_FALSE(test_obj.publishData(test));
}

TEST(test_enc_cmGroup, test_others) {
  message6.encoding(CommandElement::DAQEncoding::BASE64);
  SensorGroup test_obj(message6);
  ASSERT_EQ(test_obj.numberOfChannels(), 1);
  ASSERT_EQ(test_obj[0], test_obj(0));
  ASSERT_EQ(test_obj.getSensorGroupNo(), 6);
}
