#include <stdexcept>
#include <string>
#include "gtest/gtest.h"
#include "psaf_ucbridge/communication/message.hpp"
#include "psaf_ucbridge/communication/command.hpp"
#include "psaf_ucbridge/configuration/static_config.hpp"

/**
 * @brief Tests Reset Command ComReset
 */
TEST(ComReset, ReturnMessage) {
  ComReset test;
  Message & test_obj = test;
  ASSERT_EQ(test_obj.ReturnMessage(), "!RESET NOW\n");
}

/**
 * @brief Tests DRV Command ComDRV
 */
TEST(ComDRV, ReturnMessage) {
  ComDRV test_obj = ComDRV();
  ASSERT_EQ(test_obj.ReturnMessage(), "!DRV\n");

  // forwards
  ASSERT_EQ(test_obj.forwards(100).ReturnMessage(), "!DRV F 100\n");
  ASSERT_EQ(test_obj.repeat(true).ReturnMessage(), "!DRV F 100 100\n");
  test_obj.repeat(false);
  ASSERT_EQ(
    test_obj.forwards(CommandElement::kMinDrivingSpeed - 1000).ReturnMessage(),
    "!DRV F " + std::to_string(CommandElement::kMinDrivingSpeed) + "\n");
  ASSERT_EQ(
    test_obj.forwards(CommandElement::kMaxDrivingSpeed + 1000).ReturnMessage(),
    "!DRV F " + std::to_string(CommandElement::kMaxDrivingSpeed) + "\n");

  // backwards
  ASSERT_EQ(test_obj.backwards(100).ReturnMessage(), "!DRV B 100\n");
  ASSERT_EQ(test_obj.repeat(true).ReturnMessage(), "!DRV B 100 100\n");
  test_obj.repeat(false);
  ASSERT_EQ(
    test_obj.backwards(CommandElement::kMinDrivingSpeedBackwards - 1000).ReturnMessage(),
    "!DRV B " + std::to_string(CommandElement::kMinDrivingSpeedBackwards) + "\n");
  ASSERT_EQ(
    test_obj.backwards(CommandElement::kMaxDrivingSpeedBackwards + 1000).ReturnMessage(),
    "!DRV B " + std::to_string(CommandElement::kMaxDrivingSpeedBackwards) + "\n");

  // direct
  ASSERT_EQ(test_obj.direct(100).ReturnMessage(), "!DRV D 100\n");
  ASSERT_EQ(test_obj.repeat(true).ReturnMessage(), "!DRV D 100 100\n");
  test_obj.repeat(false);
  // on
  ASSERT_EQ(test_obj.on().ReturnMessage(), "!DRV ON\n");

  // off
  ASSERT_EQ(test_obj.off().ReturnMessage(), "!DRV OFF\n");

  // dms
  ASSERT_EQ(test_obj.dms(1000).ReturnMessage(), "!DRV ~DMS=1000\n");

  test_obj.forwards(100);
  Message & m = test_obj;
  ASSERT_EQ(m.ReturnMessage(), "!DRV F 100\n");
}

TEST(ComDRV, parseAnswer) {
  ComDRV test_obj;
  auto promise = std::promise<void>();
  test_obj.setFuture(promise.get_future());
  promise.set_value();
  test_obj.forwards(200);

  test_obj.parseAnswer(":F 200");
  ASSERT_TRUE(test_obj.success());
  ASSERT_EQ("", test_obj.errorMessage());
  ASSERT_EQ(0, test_obj.errorCode());

  test_obj.parseAnswer(":F -400");
  ASSERT_FALSE(test_obj.success());
  ASSERT_EQ("", test_obj.errorMessage());
  ASSERT_EQ(0, test_obj.errorCode());

  test_obj.parseAnswer(":ERR(666): Got empty result from Uc Board");
  ASSERT_FALSE(test_obj.success());
  ASSERT_EQ(" Got empty result from Uc Board", test_obj.errorMessage());
  ASSERT_EQ(666, test_obj.errorCode());

  ComDRV test_obj1;
  auto promise1 = std::promise<void>();
  test_obj1.setFuture(promise1.get_future());
  promise1.set_value();
  test_obj1.dms(200);
  test_obj1.parseAnswer(":OFF ~DMS=200");
  ASSERT_TRUE(test_obj1.success());
}

/**
 * @brief Tests STEER Command ComSTEER
 */
TEST(ComSTEER, ReturnMessage) {
  ComSTEER test_obj;
  ASSERT_EQ(test_obj.steer(-200).ReturnMessage(), "!STEER -200\n");
  ASSERT_EQ(test_obj.steer(200).ReturnMessage(), "!STEER 200\n");
  ASSERT_EQ(test_obj.repeat(true).ReturnMessage(), "!STEER 200 200\n");
  test_obj.repeat(false);
  ASSERT_EQ(
    test_obj.steer(CommandElement::kMaxSteerValue + 100).ReturnMessage(),
    "!STEER " + std::to_string(CommandElement::kMaxSteerValue) + "\n");
  ASSERT_EQ(
    test_obj.steer(CommandElement::kMinSteerValue - 100).ReturnMessage(),
    "!STEER " + std::to_string(CommandElement::kMinSteerValue) + "\n");
}

TEST(ComSTEER, parseAnswer) {
  ComSTEER test_obj;
  test_obj.steer(200);
  auto promise = std::promise<void>();
  test_obj.setFuture(promise.get_future());
  promise.set_value();
  test_obj.parseAnswer(":200");
  ASSERT_TRUE(test_obj.success());
  ASSERT_EQ("", test_obj.errorMessage());
  ASSERT_EQ(0, test_obj.errorCode());

  test_obj.parseAnswer(":-400");
  ASSERT_FALSE(test_obj.success());
  ASSERT_EQ("", test_obj.errorMessage());
  ASSERT_EQ(0, test_obj.errorCode());

  test_obj.parseAnswer(":ERR(666): Got empty result from Uc Board");
  ASSERT_FALSE(test_obj.success());
  ASSERT_EQ(" Got empty result from Uc Board", test_obj.errorMessage());
  ASSERT_EQ(666, test_obj.errorCode());
}

/**
 * @brief Tests SID Command ComSID
 */
TEST(ComSID, ReturnMessage) {
  ComSID test_obj;
  ASSERT_EQ(test_obj.ReturnMessage(), "!SID 0\n");
  ASSERT_EQ(test_obj.sid(100).ReturnMessage(), "!SID 100\n");
}

TEST(ComSID, parseAnswer) {
  ComSID test_obj;
  test_obj.sid(200);
  auto promise = std::promise<void>();
  test_obj.setFuture(promise.get_future());
  promise.set_value();
  test_obj.parseAnswer(":200");
  ASSERT_TRUE(test_obj.success());
  ASSERT_EQ("", test_obj.errorMessage());
  ASSERT_EQ(0, test_obj.errorCode());

  test_obj.parseAnswer(":-400");
  ASSERT_FALSE(test_obj.success());
  ASSERT_EQ("", test_obj.errorMessage());
  ASSERT_EQ(0, test_obj.errorCode());

  test_obj.parseAnswer(":ERR(666): Got empty result from Uc Board");
  ASSERT_FALSE(test_obj.success());
  ASSERT_EQ(" Got empty result from Uc Board", test_obj.errorMessage());
  ASSERT_EQ(666, test_obj.errorCode());
}

/**
 * @brief Tests DAQ Command ComDAQ
 */
TEST(ComDAQ, ReturnMessage) {
  ComDAQ test_obj;
  ASSERT_EQ(test_obj.ReturnMessage(), "!DAQ\n");

  // start
  ASSERT_EQ(test_obj.start().ReturnMessage(), "!DAQ START\n");

  // stop
  ASSERT_EQ(test_obj.stop().ReturnMessage(), "!DAQ STOP\n");

  // group
  ASSERT_EQ(test_obj.group(1).ReturnMessage(), "!DAQ GRP 1 ~ALL\n");

  // exception (group number out of range)
  ASSERT_THROW(
    test_obj.group(CommandElement::kDaqMaxGroupNumber + 1),
    std::out_of_range);
  ASSERT_THROW(
    test_obj.group(CommandElement::kDaqMinGroupNumber - 1),
    std::out_of_range);

  // delete group
  ASSERT_EQ(test_obj.group(1).remove().ReturnMessage(), "!DAQ GRP 1 ~DELETE\n");

  // deactivate group
  ASSERT_EQ(test_obj.group(1).deactivate().ReturnMessage(), "!DAQ GRP 1 ~DEACTIVATE\n");

  // activate group
  ASSERT_EQ(test_obj.group(1).activate().ReturnMessage(), "!DAQ GRP 1 ~ACTIVATE\n");

  // add channels (AX, AY, AZ)
  ASSERT_EQ(
    test_obj.group(1).channel(CommandElement::AX).channel(CommandElement::AY).channel(
      CommandElement
      ::AZ)
    .ReturnMessage(),
    "!DAQ GRP 1 AX AY AZ ~ALL\n");

  // add channels (GX, GY, GZ) ALL=10
  ASSERT_EQ(
    test_obj.group(1).channel(CommandElement::GX).channel(CommandElement::GY).channel(
      CommandElement
      ::GZ)
    .all(10).ReturnMessage(),
    "!DAQ GRP 1 GX GY GZ ~ALL=10\n");

  // add channel (HALL_DT) ANY
  ASSERT_EQ(
    test_obj.group(10).channel(CommandElement::HALL_DT).any().ReturnMessage(),
    "!DAQ GRP 10 HALL_DT ~ANY\n");

  // add channel (MX, MY, MZ) TS=100, AVG
  ASSERT_EQ(
    test_obj.group(1).channel(CommandElement::MX).channel(CommandElement::MY)
    .channel(CommandElement::MZ).ts(100).avg(50).ReturnMessage(),
    "!DAQ GRP 1 MX MY MZ ~TS=100 ~AVG=50\n");

  // add channel (MX, MY, MZ) TS=100, AVG = 50
  ASSERT_EQ(
    test_obj.group(1).channel(CommandElement::MX).channel(CommandElement::MY)
    .channel(CommandElement::MZ).ts(100).avg().ReturnMessage(),
    "!DAQ GRP 1 MX MY MZ ~TS=100 ~AVG\n");

  // encoding
  ASSERT_EQ(
    test_obj.group(1).channel(CommandElement::HALL_CNT).encoding(
      CommandElement::DAQEncoding::HEX).ReturnMessage(),
    "!DAQ GRP 1 HALL_CNT ~ALL ~ENC=HEX\n");

  ASSERT_EQ(
    test_obj.group(1).channel(CommandElement::HALL_CNT).encoding(
      CommandElement::DAQEncoding::ASCII).
    ReturnMessage(),
    "!DAQ GRP 1 HALL_CNT ~ALL\n");

  ASSERT_EQ(
    test_obj.group(1).channel(CommandElement::HALL_CNT).encoding(
      CommandElement::DAQEncoding::BASE64).
    ReturnMessage(),
    "!DAQ GRP 1 HALL_CNT ~ALL ~ENC=B64\n");

  // all options (skip, enc , crc, age, tics)
  ASSERT_EQ(
    test_obj.group(1).channel(CommandElement::HALL_CNT).ts(100).avg(100).skip(10).
    encoding(CommandElement::DAQEncoding::BASE64).crc().age().tics().ReturnMessage(),
    "!DAQ GRP 1 HALL_CNT ~TS=100 ~AVG=100 ~SKIP=10 ~ENC=B64 ~CRC ~AGE ~TICS\n");
  // unknown grp
  ASSERT_EQ(
    test_obj.group(1).otherOpt("un1 ~kk=1").ReturnMessage(),
    "!DAQ GRP 1 un1 ~kk=1\n"
  );
}

TEST(ComDAQ, parseAnswer) {
  ComDAQ test_obj;
  auto promise = std::promise<void>();
  test_obj.setFuture(promise.get_future());
  promise.set_value();
  test_obj.start();
  test_obj.parseAnswer(":started");
  ASSERT_TRUE(test_obj.success());
  ASSERT_EQ("", test_obj.errorMessage());
  ASSERT_EQ(0, test_obj.errorCode());

  test_obj.parseAnswer(":stopped");
  ASSERT_FALSE(test_obj.success());
  ASSERT_EQ("", test_obj.errorMessage());
  ASSERT_EQ(0, test_obj.errorCode());

  test_obj.stop();
  test_obj.parseAnswer(":stopped");
  ASSERT_TRUE(test_obj.success());
  ASSERT_EQ("", test_obj.errorMessage());
  ASSERT_EQ(0, test_obj.errorCode());

  test_obj.parseAnswer(":started");
  ASSERT_FALSE(test_obj.success());
  ASSERT_EQ("", test_obj.errorMessage());
  ASSERT_EQ(0, test_obj.errorCode());

  test_obj.group(10).channel(CommandElement::HALL_DT).any();
  test_obj.parseAnswer(":stopped");
  ASSERT_FALSE(test_obj.success());
  ASSERT_EQ("", test_obj.errorMessage());
  ASSERT_EQ(0, test_obj.errorCode());

  test_obj.parseAnswer(":started");
  ASSERT_FALSE(test_obj.success());
  ASSERT_EQ("", test_obj.errorMessage());
  ASSERT_EQ(0, test_obj.errorCode());

  test_obj.parseAnswer(":ok");
  ASSERT_TRUE(test_obj.success());
  ASSERT_EQ("", test_obj.errorMessage());
  ASSERT_EQ(0, test_obj.errorCode());

  test_obj.parseAnswer(":ERR(666): Got empty result from Uc Board");
  ASSERT_FALSE(test_obj.success());
  ASSERT_EQ(" Got empty result from Uc Board", test_obj.errorMessage());
  ASSERT_EQ(666, test_obj.errorCode());
}

/**
 * @brief Tests LED Command ComLED
 */
TEST(ComLED, ReturnMessage) {
  // nothing
  {
    ComLED test_obj;
    ASSERT_EQ(test_obj.ReturnMessage(), "!LED\n");
  }


  // on
  {
    ComLED test_obj;
    ASSERT_EQ(
      test_obj.on(CommandElement::LEDType::A)
      .on(CommandElement::LEDType::B)
      .on(CommandElement::LEDType::BLKR)
      .on(CommandElement::LEDType::BLKL)
      .on(CommandElement::LEDType::BRMS)
      .ReturnMessage(), "!LED A ON B ON BRMS ON BLKL ON BLKR ON\n");
  }


  // off
  {
    ComLED test_obj;
    ASSERT_EQ(
      test_obj.off(CommandElement::LEDType::A)
      .off(CommandElement::LEDType::B)
      .off(CommandElement::LEDType::BLKR)
      .off(CommandElement::LEDType::BLKL)
      .off(CommandElement::LEDType::BRMS).ReturnMessage(),
      "!LED A OFF B OFF BRMS OFF BLKL OFF BLKR OFF\n");
  }

  {
    ComLED test_obj;
    ASSERT_EQ(
      test_obj.on(CommandElement::LEDType::BRMS).ReturnMessage(),
      "!LED BRMS ON\n"
    );
  }


  // toggle
  {
    ComLED test_obj;
    ASSERT_EQ(
      test_obj.toggle(CommandElement::LEDType::A)
      .toggle(CommandElement::LEDType::B)
      .toggle(CommandElement::LEDType::BLKR)
      .toggle(CommandElement::LEDType::BLKL)
      .toggle(CommandElement::LEDType::BRMS)
      .ReturnMessage(),
      "!LED A TOG B TOG BRMS TOG BLKL TOG BLKR TOG\n");
  }


  // sequence
  {
    ComLED test_obj;
    ASSERT_EQ(
      test_obj
      .blink(CommandElement::LEDType::A, 100, 100)
      .blink(CommandElement::LEDType::B, 50, 50).invert(CommandElement::LEDType::B)
      .blink(CommandElement::LEDType::BLKR, 50, 50)
      .ReturnMessage(), "!LED A 100 100 B 0 50 50 BLKR 50 50\n");
  }

  {
    ComLED test_obj;
    ASSERT_EQ(
      test_obj
      .blink(CommandElement::LEDType::A, 100, 100)
      .blink(CommandElement::LEDType::B, 50, 50)
      .invert(CommandElement::LEDType::A)
      .invert(CommandElement::LEDType::B, false)
      .ReturnMessage(), "!LED A 0 100 100 B 50 50\n");
  }

  {
    ComLED test_obj;
    ASSERT_EQ(
      test_obj
      .blink(CommandElement::LEDType::A, 100, 100)
      .blink(CommandElement::LEDType::B, 50, 50)
      .invert(CommandElement::LEDType::A)
      .invert(CommandElement::LEDType::B)
      .ReturnMessage(), "!LED A 0 100 100 B 0 50 50\n");
  }

  {
    ComLED test_obj;
    ASSERT_EQ(
      test_obj
      .blink(CommandElement::LEDType::A, 100, 100)
      .blink(CommandElement::LEDType::B, 50, 50)
      .invert(CommandElement::LEDType::A, false)
      .invert(CommandElement::LEDType::B, false)
      .ReturnMessage(), "!LED A 100 100 B 50 50\n");
  }
}

TEST(ComLED, parseAnswer) {
  ComLED test_obj;
  auto promise = std::promise<void>();
  test_obj.setFuture(promise.get_future());
  promise.set_value();
  test_obj.parseAnswer(":ok");
  ASSERT_TRUE(test_obj.success());
  ASSERT_EQ("", test_obj.errorMessage());
  ASSERT_EQ(0, test_obj.errorCode());

  test_obj.parseAnswer(":nooooooo");
  ASSERT_FALSE(test_obj.success());
  ASSERT_EQ("", test_obj.errorMessage());
  ASSERT_EQ(0, test_obj.errorCode());

  test_obj.parseAnswer(":ERR(666): Got empty result from Uc Board");
  ASSERT_FALSE(test_obj.success());
  ASSERT_EQ(" Got empty result from Uc Board", test_obj.errorMessage());
  ASSERT_EQ(666, test_obj.errorCode());
}

/**
 * @brief Tests VOUT Command ComVOUT
 */
TEST(ComVOUT, ReturnMessage) {
  ComVOUT test_obj;
  ASSERT_EQ(test_obj.ReturnMessage(), "!VOUT ON\n");
  ASSERT_EQ(test_obj.on().ReturnMessage(), "!VOUT ON\n");
  ASSERT_EQ(test_obj.off().ReturnMessage(), "!VOUT OFF\n");
}

TEST(ComVOUT, parseAnswer) {
  ComVOUT test_obj;
  auto promise = std::promise<void>();
  test_obj.setFuture(promise.get_future());
  promise.set_value();
  test_obj.on();
  test_obj.parseAnswer(":ON");
  ASSERT_TRUE(test_obj.success());
  ASSERT_EQ("", test_obj.errorMessage());
  ASSERT_EQ(0, test_obj.errorCode());

  test_obj.parseAnswer(":OFF");
  ASSERT_FALSE(test_obj.success());
  ASSERT_EQ("", test_obj.errorMessage());
  ASSERT_EQ(0, test_obj.errorCode());

  test_obj.off();
  test_obj.parseAnswer(":OFF");
  ASSERT_TRUE(test_obj.success());
  ASSERT_EQ("", test_obj.errorMessage());
  ASSERT_EQ(0, test_obj.errorCode());

  test_obj.parseAnswer(":ON");
  ASSERT_FALSE(test_obj.success());
  ASSERT_EQ("", test_obj.errorMessage());
  ASSERT_EQ(0, test_obj.errorCode());

  test_obj.parseAnswer(":ERR(666): Got empty result from Uc Board");
  ASSERT_FALSE(test_obj.success());
  ASSERT_EQ(" Got empty result from Uc Board", test_obj.errorMessage());
  ASSERT_EQ(666, test_obj.errorCode());
}


/**
 * @brief Tests US Command ComUS
 */
TEST(ComUS, ReturnMessage) {
  ComUS test_obj;
  ASSERT_EQ(test_obj.ReturnMessage(), "!US OPT ~GAIN=10 ~RANGE=100\n");
  ComUS test_obj0;
  ASSERT_EQ(test_obj0.on().ReturnMessage(), "!US ON\n");
  ComUS test_obj1;
  ASSERT_EQ(test_obj1.off().ReturnMessage(), "!US OFF\n");
  ComUS test_obj2;
  ASSERT_EQ(test_obj2.range(32).ReturnMessage(), "!US OPT ~RANGE=32\n");
  ComUS test_obj3;
  ASSERT_EQ(test_obj3.gain(44).ReturnMessage(), "!US OPT ~GAIN=44\n");
  ComUS test_obj4;
  ASSERT_EQ(test_obj4.range(44).gain(32).ReturnMessage(), "!US OPT ~GAIN=32 ~RANGE=44\n");
}

TEST(ComUS, parseAnswer) {
  ComUS test_obj;
  auto promise = std::promise<void>();
  test_obj.setFuture(promise.get_future());
  promise.set_value();
  test_obj.on();
  test_obj.parseAnswer(":ON");
  ASSERT_TRUE(test_obj.success());
  ASSERT_EQ("", test_obj.errorMessage());
  ASSERT_EQ(0, test_obj.errorCode());

  test_obj.parseAnswer(":OFF");
  ASSERT_FALSE(test_obj.success());
  ASSERT_EQ("", test_obj.errorMessage());
  ASSERT_EQ(0, test_obj.errorCode());

  test_obj.off();
  test_obj.parseAnswer(":OFF");
  ASSERT_TRUE(test_obj.success());
  ASSERT_EQ("", test_obj.errorMessage());
  ASSERT_EQ(0, test_obj.errorCode());

  test_obj.parseAnswer(":ON");
  ASSERT_FALSE(test_obj.success());
  ASSERT_EQ("", test_obj.errorMessage());
  ASSERT_EQ(0, test_obj.errorCode());

  ComUS test_obj1;
  auto promise1 = std::promise<void>();
  test_obj1.setFuture(promise1.get_future());
  promise1.set_value();
  test_obj1.parseAnswer(":~RANGE=100 ~GAIN=10");
  ASSERT_TRUE(test_obj1.success());
  ASSERT_EQ("", test_obj1.errorMessage());
  ASSERT_EQ(0, test_obj1.errorCode());

  ComUS test_obj2;
  auto promise2 = std::promise<void>();
  test_obj2.setFuture(promise2.get_future());
  promise2.set_value();
  test_obj2.gain(11);
  test_obj2.parseAnswer(":~GAIN=11");
  ASSERT_TRUE(test_obj2.success());
  ASSERT_EQ("", test_obj2.errorMessage());
  ASSERT_EQ(0, test_obj2.errorCode());

  ComUS test_obj3;
  auto promise3 = std::promise<void>();
  test_obj3.setFuture(promise3.get_future());
  promise3.set_value();
  test_obj2.parseAnswer(":~GAIN=11");
  test_obj3.parseAnswer(":~GAIN=10");
  ASSERT_FALSE(test_obj3.success());
  ASSERT_EQ("", test_obj3.errorMessage());
  ASSERT_EQ(0, test_obj3.errorCode());

  test_obj.parseAnswer(":ERR(666): Got empty result from Uc Board");
  ASSERT_FALSE(test_obj.success());
  ASSERT_EQ(" Got empty result from Uc Board", test_obj.errorMessage());
  ASSERT_EQ(666, test_obj.errorCode());
}

/**
 * @brief Tests IMU Command ComIMU
 */
TEST(ComIMU, ReturnMessage) {
  ComIMU test_obj;
  ASSERT_EQ(test_obj.ReturnMessage(), "!IMU OPT ~ARANGE=4 ~AFILT=0 ~GRANGE=500 ~GFILT=0\n");
  test_obj.parseAnswer(":~ARANGE=4 ~AFILT=0 ~GRANGE=500 ~GFILT=0");
  ComIMU test_obj1;
  ASSERT_EQ(
    test_obj1.arange(CommandElement::IMUAcceleration::ImuAcc16).afilt(
      CommandElement::IMUAFiltMode::AFILT1).ReturnMessage(), "!IMU OPT ~ARANGE=16 ~AFILT=1\n");
  ComIMU test_obj2;
  ASSERT_EQ(
    test_obj2.grange(CommandElement::IMURotationRate::ImuRot2000).gfilt(
      CommandElement::IMUGFiltMode::GFILT7).ReturnMessage(), "!IMU OPT ~GRANGE=2000 ~GFILT=7\n");
}

/**
 * @brief Tests MAG Command ComMAG
 */
TEST(ComMAG, ReturnMessage) {
  ComMAG test_obj;
  ASSERT_EQ(test_obj.ReturnMessage(), "!MAG OPT ~USEASA=1\n");
  ASSERT_EQ(test_obj.useasa(false).ReturnMessage(), "!MAG OPT ~USEASA=0\n");
  ASSERT_EQ(test_obj.useasa().ReturnMessage(), "!MAG OPT ~USEASA=1\n");
}

TEST(ComMAG, parseAnswer) {
  ComMAG test_obj;
  auto promise = std::promise<void>();
  test_obj.setFuture(promise.get_future());
  promise.set_value();
  test_obj.parseAnswer(":~USEASA=1");
  ASSERT_TRUE(test_obj.success());
  ASSERT_EQ("", test_obj.errorMessage());
  ASSERT_EQ(0, test_obj.errorCode());

  test_obj.parseAnswer("ksfjhsj");
  ASSERT_FALSE(test_obj.success());
  ASSERT_EQ("", test_obj.errorMessage());
  ASSERT_EQ(0, test_obj.errorCode());

  test_obj.useasa(false);
  test_obj.parseAnswer(":~USEASA=0");
  ASSERT_TRUE(test_obj.success());
  ASSERT_EQ("", test_obj.errorMessage());
  ASSERT_EQ(0, test_obj.errorCode());

  test_obj.parseAnswer("ksfjhsj");
  ASSERT_FALSE(test_obj.success());
  ASSERT_EQ("", test_obj.errorMessage());
  ASSERT_EQ(0, test_obj.errorCode());

  test_obj.parseAnswer(":ERR(666): Got empty result from Uc Board");
  ASSERT_FALSE(test_obj.success());
  ASSERT_EQ(" Got empty result from Uc Board", test_obj.errorMessage());
  ASSERT_EQ(666, test_obj.errorCode());
}

TEST(ComENC, ReturnMessage) {
  ComENC test_obj;
  test_obj.calibration(66);
  ASSERT_EQ(test_obj.ReturnMessage(), "!ENC 66\n");
}

TEST(ComENC, parseAnswer) {
  ComENC test_obj;
  auto promise = std::promise<void>();
  test_obj.setFuture(promise.get_future());
  promise.set_value();
  test_obj.parseAnswer(":ok");
  ASSERT_TRUE(test_obj.success());
  test_obj.parseAnswer(":ERR(666): Got empty result from Uc Board");
  ASSERT_FALSE(test_obj.success());
}
