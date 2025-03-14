#include <future>
#include "gtest/gtest.h"
#include "psaf_ucbridge/communication/message.hpp"
#include "psaf_ucbridge/communication/request.hpp"

/**
 * @brief Tests Version Request ReqVer
 */
TEST(test_Message_Request, test_ReqVer) {
  ReqVer test_obj;
  ASSERT_EQ(test_obj.ReturnMessage(), "?VER\n");
  auto promise = std::promise<void>();
  test_obj.parseAnswer(":4.3.6868");
  test_obj.setFuture(promise.get_future());
  promise.set_value();
  ASSERT_EQ(test_obj.major(), 4);
  ASSERT_EQ(test_obj.minor(), 3);
  ASSERT_EQ(test_obj.patch(), 6868);
  ASSERT_EQ(test_obj.success(), true);
  auto promise2 = std::promise<void>();
  test_obj.setFuture(promise2.get_future());
  test_obj.parseAnswer(":ERR(666): Got empty result from Uc Board");
  promise2.set_value();
  ASSERT_FALSE(test_obj.success());
  ASSERT_EQ(" Got empty result from Uc Board", test_obj.errorMessage());
  ASSERT_EQ(666, test_obj.errorCode());
}

/**
 * @brief Tests ID Request ReqID
 */
TEST(test_Message_Request, test_ReqID) {
  ReqID test_obj;
  auto promise = std::promise<void>();
  test_obj.setFuture(promise.get_future());
  ASSERT_EQ(test_obj.ReturnMessage(), "?ID\n");
  test_obj.parseAnswer(":1234567");
  promise.set_value();
  ASSERT_EQ(test_obj.success(), true);
  ASSERT_EQ(test_obj.id(), 1234567);
  auto promise2 = std::promise<void>();
  test_obj.setFuture(promise2.get_future());
  test_obj.parseAnswer(":ERR(666): Got empty result from Uc Board");
  promise2.set_value();
  ASSERT_FALSE(test_obj.success());
  ASSERT_EQ(" Got empty result from Uc Board", test_obj.errorMessage());
  ASSERT_EQ(666, test_obj.errorCode());
}

/**
 * @brief Tests SID Request ReqSID
 */
TEST(test_Message_Request, test_ReqSID) {
  ReqSID test_obj;
  auto promise = std::promise<void>();
  test_obj.setFuture(promise.get_future());
  ASSERT_EQ(test_obj.ReturnMessage(), "?SID\n");
  test_obj.parseAnswer(":1234567");
  promise.set_value();
  ASSERT_EQ(test_obj.id(), 1234567);
  ASSERT_EQ(test_obj.success(), true);
  test_obj.parseAnswer(":ERR(666): Got empty result from Uc Board");
  ASSERT_FALSE(test_obj.success());
  ASSERT_EQ(" Got empty result from Uc Board", test_obj.errorMessage());
  ASSERT_EQ(666, test_obj.errorCode());
}

/**
 * @brief Tests TICS Request ReqTICS
 */
TEST(test_Message_Request, test_ReqTICS) {
  ReqTICS test_obj;
  auto promise = std::promise<void>();
  test_obj.setFuture(promise.get_future());
  ASSERT_EQ(test_obj.ReturnMessage(), "?TICS\n");
  test_obj.parseAnswer(":1234567");
  promise.set_value();
  ASSERT_EQ(test_obj.tics(), 1234567);
  ASSERT_EQ(test_obj.success(), true);
  test_obj.parseAnswer(":ERR(666): Got empty result from Uc Board");
  ASSERT_FALSE(test_obj.success());
  ASSERT_EQ(" Got empty result from Uc Board", test_obj.errorMessage());
  ASSERT_EQ(666, test_obj.errorCode());
}

/**
 * @brief Tests STEER Request ReqSTEER
 */
TEST(test_Message_Request, test_ReqSTEER) {
  ReqSTEER test_obj;
  auto promise = std::promise<void>();
  test_obj.setFuture(promise.get_future());
  ASSERT_EQ(test_obj.ReturnMessage(), "?STEER\n");
  test_obj.parseAnswer(":1234567");
  ASSERT_EQ(test_obj.angle(), 1234567);
  test_obj.parseAnswer(":-1234567");
  ASSERT_EQ(test_obj.angle(), -1234567);
  promise.set_value();
  ASSERT_EQ(test_obj.success(), true);
  test_obj.parseAnswer(":ERR(666): Got empty result from Uc Board");
  ASSERT_FALSE(test_obj.success());
  ASSERT_EQ(" Got empty result from Uc Board", test_obj.errorMessage());
  ASSERT_EQ(666, test_obj.errorCode());
}

/**
 * @brief Tests DRV Request ReqDRV with asking speed.
 */
TEST(test_Message_Request, test_ReqDRVS) {
  ReqDRV test_obj;
  auto promise = std::promise<void>();
  test_obj.setFuture(promise.get_future());
  ASSERT_EQ(test_obj.ReturnMessage(), "?DRV\n");
  test_obj.parseAnswer(":F 300");
  promise.set_value();
  ASSERT_EQ(test_obj.aMode(), CommandElement::DRVMode::FORWARDS);
  ASSERT_EQ(test_obj.value(), 300);
  test_obj.parseAnswer(":B 200");
  ASSERT_EQ(test_obj.aMode(), CommandElement::DRVMode::BACKWARDS);
  ASSERT_EQ(test_obj.value(), 200);
}

/**
 * @brief Tests DRV Request ReqDRV with asking DMS.
 */
TEST(test_Message_Request, test_ReqDRVDMS) {
  ReqDRV test_obj;
  auto promise = std::promise<void>();
  test_obj.setFuture(promise.get_future());
  test_obj.dms();
  ASSERT_EQ(test_obj.ReturnMessage(), "?DRV ~DMS\n");
  test_obj.parseAnswer(":ON ~DMS=1000");
  promise.set_value();
  ASSERT_TRUE(test_obj.activate());
  ASSERT_EQ(test_obj.aMode(), CommandElement::DRVMode::DMS);
  ASSERT_EQ(test_obj.value(), 1000);
}

/**
 * @brief Tests DRV Request ReqDRV with asking impulse width.
 */
TEST(test_Message_Request, test_ReqDRVD) {
  ReqDRV test_obj;
  auto promise = std::promise<void>();
  test_obj.setFuture(promise.get_future());
  test_obj.direct();
  ASSERT_EQ(test_obj.ReturnMessage(), "?DRV D\n");
  test_obj.parseAnswer(":D -186");
  promise.set_value();
  ASSERT_EQ(test_obj.aMode(), CommandElement::DRVMode::DIRECT);
  ASSERT_EQ(test_obj.value(), -186);
}

/**
 * @brief Test DAQ Request with asking channels info.
 */
TEST(test_Message_Request, test_ReqDAQCH_CHS) {
  ReqDAQCH test_obj;
  auto promise = std::promise<void>();
  test_obj.setFuture(promise.get_future());
  test_obj.channels();
  ASSERT_EQ(test_obj.ReturnMessage(), "?DAQ CHS\n");
  test_obj.parseAnswer(
    ":2\nAX | acc. ahead  | opt-dep.! | 1  | I16\nAY | acc. left  | opt-dep.!  | 1 | I16");
  promise.set_value();
  ASSERT_EQ(test_obj.getChannelDescriptions().size(), 2);
  ASSERT_EQ(test_obj.getChannelDescriptions()[0].getName(), "AX");
  ASSERT_EQ(test_obj.getChannelDescriptions()[1].getName(), "AY");
  ASSERT_EQ(test_obj.getChannelDescriptions()[0].getDescription(), "acc. ahead");
  ASSERT_EQ(test_obj.getChannelDescriptions()[1].getDescription(), "acc. left");
  ASSERT_EQ(test_obj.getChannelDescriptions()[0].getUnit(), "opt-dep.!");
  ASSERT_EQ(test_obj.getChannelDescriptions()[1].getUnit(), "opt-dep.!");
  ASSERT_EQ(test_obj.getChannelDescriptions()[0].getScanTime(), "1");
  ASSERT_EQ(test_obj.getChannelDescriptions()[1].getScanTime(), "1");
  ASSERT_EQ(test_obj.getChannelDescriptions()[0].getRawType(), "I16");
  ASSERT_EQ(test_obj.getChannelDescriptions()[1].getRawType(), "I16");
}

/**
 * @brief Test DAQ Request with asking channel info.
 */
TEST(test_Message_Request, test_ReqDAQCH_CH) {
  ReqDAQCH test_obj;
  auto promise = std::promise<void>();
  test_obj.setFuture(promise.get_future());
  test_obj.name("USF");
  ASSERT_EQ(test_obj.ReturnMessage(), "?DAQ CH USF\n");
  test_obj.parseAnswer(
    ":USF        | ultrasonic front distance      | 0.1 mm          | undef | U16");
  promise.set_value();
  ASSERT_EQ(test_obj.getChannelDescriptions().size(), 1);
  ASSERT_EQ(test_obj.getChannelDescriptions()[0].getName(), "USF");
  ASSERT_EQ(test_obj.getChannelDescriptions()[0].getDescription(), "ultrasonic front distance");
  ASSERT_EQ(test_obj.getChannelDescriptions()[0].getUnit(), "0.1 mm");
  ASSERT_EQ(test_obj.getChannelDescriptions()[0].getScanTime(), "undef");
  ASSERT_EQ(test_obj.getChannelDescriptions()[0].getRawType(), "U16");
}

/**
 * @brief Test DAQ Request get
 */
TEST(test_Message_Request, test_ReqDAQ_GET_ReturnMessage) {
  ReqDAQGET test_obj;
  test_obj.addSensor("USF");
  test_obj.addSensor("USL");
  test_obj.addSensor("USR");
  ASSERT_EQ(test_obj.ReturnMessage(), "?DAQ GET USF USL USR\n");
  test_obj.age();
  ASSERT_EQ(test_obj.ReturnMessage(), "?DAQ GET ~AGE USF USL USR\n");
  test_obj.tics();
  ASSERT_EQ(test_obj.ReturnMessage(), "?DAQ GET ~AGE ~TICS USF USL USR\n");
}

TEST(test_Message_Request, test_ReqDAQ_GET_ParseAnswer_case1) {
  ReqDAQGET test_obj;
  auto promise = std::promise<void>();
  test_obj.setFuture(promise.get_future());
  test_obj.addSensor("USF");
  test_obj.parseAnswer(":1860");
  promise.set_value();
  ASSERT_TRUE(test_obj.success());
  ASSERT_EQ(test_obj.getMeasureValue()[0], 1860);
  ASSERT_TRUE(test_obj.getTicsValue().empty());
  ASSERT_TRUE(test_obj.getAgeValue().empty());
  test_obj.parseAnswer(":1860 75");
  ASSERT_FALSE(test_obj.success());
}

TEST(test_Message_Request, test_ReqDAQ_GET_ParseAnswer_case2) {
  ReqDAQGET test_obj;
  auto promise = std::promise<void>();
  test_obj.setFuture(promise.get_future());
  promise.set_value();
  test_obj.addSensor("USF");
  test_obj.age();
  test_obj.parseAnswer(":1860 68");
  ASSERT_TRUE(test_obj.success());
  ASSERT_TRUE(test_obj.getTicsValue().empty());
  ASSERT_EQ(test_obj.getAgeValue()[0], 68);
  ASSERT_EQ(test_obj.getMeasureValue()[0], 1860);
  test_obj.parseAnswer(":1860 68 | 12 3");
  ASSERT_FALSE(test_obj.success());
}

TEST(test_Message_Request, test_ReqDAQ_GET_ParseAnswer_case3) {
  ReqDAQGET test_obj;
  auto promise = std::promise<void>();
  test_obj.setFuture(promise.get_future());
  promise.set_value();
  test_obj.addSensor("USL");
  test_obj.addSensor("USF");
  test_obj.addSensor("USR");
  test_obj.age();
  test_obj.parseAnswer(":487 72 | 1860 75 | 4293 85");
  ASSERT_TRUE(test_obj.success());
  ASSERT_EQ(test_obj.getMeasureValue()[0], 487);
  ASSERT_EQ(test_obj.getMeasureValue()[1], 1860);
  ASSERT_EQ(test_obj.getMeasureValue()[2], 4293);
  ASSERT_EQ(test_obj.getAgeValue()[0], 72);
  ASSERT_EQ(test_obj.getAgeValue()[1], 75);
  ASSERT_EQ(test_obj.getAgeValue()[2], 85);
  ASSERT_TRUE(test_obj.getTicsValue().empty());
}

TEST(test_Message_Request, test_ReqDAQ_GET_ParseAnswer_case4) {
  ReqDAQGET test_obj;
  auto promise = std::promise<void>();
  test_obj.setFuture(promise.get_future());
  promise.set_value();
  test_obj.addSensor("USL");
  test_obj.addSensor("USF");
  test_obj.addSensor("USR");
  test_obj.age();
  test_obj.tics();
  test_obj.parseAnswer(":487 72 66| 1860 75 77| 4293 85 88");
  ASSERT_TRUE(test_obj.success());
  ASSERT_EQ(test_obj.getMeasureValue()[0], 487);
  ASSERT_EQ(test_obj.getMeasureValue()[1], 1860);
  ASSERT_EQ(test_obj.getMeasureValue()[2], 4293);
  ASSERT_EQ(test_obj.getAgeValue()[0], 72);
  ASSERT_EQ(test_obj.getAgeValue()[1], 75);
  ASSERT_EQ(test_obj.getAgeValue()[2], 85);
  ASSERT_EQ(test_obj.getTicsValue()[0], 66);
  ASSERT_EQ(test_obj.getTicsValue()[1], 77);
  ASSERT_EQ(test_obj.getTicsValue()[2], 88);
}

TEST(test_Message_Request, test_ReqDAQ_Grp) {
  {
    ReqDAQGRP test_obj;
    auto promise = std::promise<void>();
    test_obj.setFuture(promise.get_future());
    promise.set_value();
    test_obj.groups();
    ASSERT_EQ(test_obj.ReturnMessage(), "?DAQ GRPS\n");
    test_obj.setNum(1);
    ASSERT_EQ(test_obj.ReturnMessage(), "?DAQ GRP 1\n");
    test_obj.parseAnswer(":\n1  : USL USF USR   ~ENC=B64 ~ALL=10");
    ASSERT_EQ(test_obj.getInfo()[0], "1  : USL USF USR   ~ENC=B64 ~ALL=10");
  }
  {
    ReqDAQGRP test_obj;
    auto promise = std::promise<void>();
    test_obj.setFuture(promise.get_future());
    promise.set_value();
    test_obj.groups();
    ASSERT_EQ(test_obj.ReturnMessage(), "?DAQ GRPS\n");
    test_obj.parseAnswer(
      "\n1  : USL USF USR   ~ENC=B64 ~ALL=10 \n3  : AX AY GZ   ~ENC=B64 ~TS=10 ~AVG=10");
    ASSERT_EQ(test_obj.getInfo().size(), 2);
    ASSERT_EQ(test_obj.getInfo()[0], "1  : USL USF USR   ~ENC=B64 ~ALL=10");
    ASSERT_EQ(test_obj.getInfo()[1], "3  : AX AY GZ   ~ENC=B64 ~TS=10 ~AVG=10");
  }
}

TEST(test_Message_Request, test_Req_VOUT) {
  ReqVOUT test_obj;
  auto promise = std::promise<void>();
  test_obj.setFuture(promise.get_future());
  promise.set_value();
  ASSERT_EQ(test_obj.ReturnMessage(), "?VOUT\n");
  test_obj.parseAnswer(":ON");
  ASSERT_TRUE(test_obj.success());
  ASSERT_TRUE(test_obj.getStatus());
  test_obj.parseAnswer(":OFF");
  ASSERT_FALSE(test_obj.getStatus());
}

TEST(test_Message_Request, test_Req_US) {
  {
    ReqUS test_obj;
    auto promise = std::promise<void>();
    test_obj.setFuture(promise.get_future());
    promise.set_value();
    ASSERT_EQ(test_obj.ReturnMessage(), "?US\n");
    test_obj.parseAnswer(":ON");
    ASSERT_TRUE(test_obj.getStatus());
    ASSERT_TRUE(test_obj.success());
    test_obj.parseAnswer(":OFF");
    ASSERT_FALSE(test_obj.getStatus());
    ASSERT_TRUE(test_obj.success());
  }
  {
    ReqUS test_obj;
    auto promise = std::promise<void>();
    test_obj.setFuture(promise.get_future());
    promise.set_value();
    test_obj.opt();
    ASSERT_EQ(test_obj.ReturnMessage(), "?US OPT\n");
    test_obj.parseAnswer(":~RANGE=100 ~GAIN=10   [4343 mm, 25 ms]");
    ASSERT_TRUE(test_obj.success());
    ASSERT_EQ(test_obj.getGain(), 10);
    ASSERT_EQ(test_obj.getRange(), 100);
  }
}

TEST(test_Message_Request, test_Req_Imu) {
  {
    ReqIMU test_obj;
    auto promise = std::promise<void>();
    test_obj.setFuture(promise.get_future());
    promise.set_value();
    ASSERT_EQ(test_obj.ReturnMessage(), "?IMU OPT\n");
    test_obj.parseAnswer(":~ARANGE=8 ~AFILT=3 ~GRANGE=1000 ~GFILT=0   [A: 45 Hz, G: 250 Hz]");
    ASSERT_TRUE(test_obj.success());
    ASSERT_EQ(test_obj.getArange(), 8);
    ASSERT_EQ(test_obj.getAfilt(), 3);
    ASSERT_EQ(test_obj.getGrange(), 1000);
    ASSERT_EQ(test_obj.getGfilt(), 0);
  }
}

TEST(test_Message_Request, test_Req_Mag) {
  {
    ReqMAG test_obj;
    auto promise = std::promise<void>();
    test_obj.setFuture(promise.get_future());
    promise.set_value();
    test_obj.opt();
    ASSERT_EQ(test_obj.ReturnMessage(), "?MAG OPT\n");
    test_obj.parseAnswer(":~USEASA=1");
    ASSERT_TRUE(test_obj.success());
    ASSERT_TRUE(test_obj.getUseasa());
    test_obj.parseAnswer(":~USEASA=0");
    ASSERT_TRUE(test_obj.success());
    ASSERT_FALSE(test_obj.getUseasa());
    test_obj.asa();
    ASSERT_EQ(test_obj.ReturnMessage(), "?MAG ASA\n");
    test_obj.parseAnswer(":176 176 165");
    ASSERT_TRUE(test_obj.success());
    ASSERT_EQ(test_obj.getAsavalue()[0], 176);
    ASSERT_EQ(test_obj.getAsavalue()[1], 176);
    ASSERT_EQ(test_obj.getAsavalue()[2], 165);
  }
}

TEST(test_Message_Request, test_Req_Enc) {
  ReqENC test_obj;
  auto promise = std::promise<void>();
  test_obj.setFuture(promise.get_future());
  promise.set_value();
  ASSERT_EQ(test_obj.ReturnMessage(), "?ENC\n");
  test_obj.parseAnswer(":8888");
  ASSERT_TRUE(test_obj.success());
  ASSERT_EQ(test_obj.steps(), 8888);
}

TEST(test_Message_Request, test_Req_Rc) {
  ReqRc test_obj;
  auto promise = std::promise<void>();
  test_obj.setFuture(promise.get_future());
  promise.set_value();
  ASSERT_EQ(test_obj.ReturnMessage(), "?RCMODE\n");
  test_obj.parseAnswer(":RC_ON");
  ASSERT_EQ(test_obj.get_mode(), 1);
  test_obj.parseAnswer(":RC_OFF");
  ASSERT_EQ(test_obj.get_mode(), 0);
}
