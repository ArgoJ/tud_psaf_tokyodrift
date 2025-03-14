#include <string>
#include <chrono>
#include <utility>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "gtest/gtest.h"
#include "psaf_ucbridge/service_functions/service_functions.hpp"
#include "psaf_ucbridge/communication/message.hpp"


class MockCommunication : public ICommunication
{
public:
  void sendCommand(const Message::SharedPtr & cmd, std::chrono::microseconds) override
  {
    cmd->parseAnswer(s);
    auto a = std::promise<void>();
    cmd->setFuture(a.get_future());
    a.set_value();
    cmd->notify_one();
  }
  void setReceive(std::string input)
  {
    s.clear();
    s = std::move(input);
  }

private:
  std::string s;
};

TEST(test_service_functions, ReqVer) {
  rclcpp::init(0, nullptr);
  auto request = std::make_shared<psaf_ucbridge_msgs::srv::ReqVer::Request>();
  auto response = std::make_shared<psaf_ucbridge_msgs::srv::ReqVer::Response>();
  auto node = rclcpp::Node::make_shared("mock_server");
  auto com = std::make_shared<MockCommunication>();
  com->setReceive(":4.3.6868");
  ServiceFunctions::reqVer(request, response, com, node->get_logger());
  ASSERT_EQ(response->major, 4);
  ASSERT_EQ(response->minor, 3);
  ASSERT_EQ(response->patch, 6868);
  com->setReceive(":ERR(666): Got empty result from Uc Board");
  ServiceFunctions::reqVer(request, response, com, node->get_logger());
  ASSERT_FALSE(response->succeed);
  rclcpp::shutdown();
}

TEST(test_service_functions, ReqId) {
  rclcpp::init(0, nullptr);
  auto com = std::make_shared<MockCommunication>();
  auto node = rclcpp::Node::make_shared("mock_server");
  {
    auto request = std::make_shared<psaf_ucbridge_msgs::srv::ReqId::Request>();
    auto response = std::make_shared<psaf_ucbridge_msgs::srv::ReqId::Response>();


    com->setReceive(":4");
    ServiceFunctions::reqId(request, response, com, node->get_logger());
    ASSERT_EQ(response->id, 4);
  }
  {
    auto request = std::make_shared<psaf_ucbridge_msgs::srv::ReqId::Request>();
    auto response = std::make_shared<psaf_ucbridge_msgs::srv::ReqId::Response>();
    com->setReceive(":ERR(666): Got empty result from Uc Board");
    ServiceFunctions::reqId(request, response, com, node->get_logger());
    ASSERT_FALSE(response->succeed);
  }
  rclcpp::shutdown();
}

TEST(test_service_functions, ReqSid) {
  rclcpp::init(0, nullptr);
  auto node = rclcpp::Node::make_shared("mock_server");
  auto com = std::make_shared<MockCommunication>();
  {
    auto request = std::make_shared<psaf_ucbridge_msgs::srv::ReqSid::Request>();
    auto response = std::make_shared<psaf_ucbridge_msgs::srv::ReqSid::Response>();

    com->setReceive(":4");
    ServiceFunctions::reqSid(request, response, com, node->get_logger());
    ASSERT_EQ(response->sid, 4);
  }

  {
    auto request = std::make_shared<psaf_ucbridge_msgs::srv::ReqSid::Request>();
    auto response = std::make_shared<psaf_ucbridge_msgs::srv::ReqSid::Response>();
    com->setReceive(":ERR(666): Got empty result from Uc Board");
    ServiceFunctions::reqSid(request, response, com, node->get_logger());
    ASSERT_FALSE(response->succeed);
  }
  rclcpp::shutdown();
}

TEST(test_service_functions, ReqSteer) {
  rclcpp::init(0, nullptr);
  auto node = rclcpp::Node::make_shared("mock_server");
  auto com = std::make_shared<MockCommunication>();
  {
    auto request = std::make_shared<psaf_ucbridge_msgs::srv::ReqSteer::Request>();
    auto response = std::make_shared<psaf_ucbridge_msgs::srv::ReqSteer::Response>();
    com->setReceive(":200");
    ServiceFunctions::reqSteer(request, response, com, node->get_logger());
    ASSERT_EQ(response->angle, 200);
  }
  {
    auto request = std::make_shared<psaf_ucbridge_msgs::srv::ReqSteer::Request>();
    auto response = std::make_shared<psaf_ucbridge_msgs::srv::ReqSteer::Response>();
    com->setReceive(":ERR(666): Got empty result from Uc Board");
    ServiceFunctions::reqSteer(request, response, com, node->get_logger());
    ASSERT_FALSE(response->succeed);
  }
  rclcpp::shutdown();
}
TEST(test_service_functions, ReqDrv) {
  rclcpp::init(0, nullptr);
  auto node = rclcpp::Node::make_shared("mock_server");
  auto com = std::make_shared<MockCommunication>();
  {
    auto request = std::make_shared<psaf_ucbridge_msgs::srv::ReqDrv::Request>();
    auto response = std::make_shared<psaf_ucbridge_msgs::srv::ReqDrv::Response>();
    com->setReceive(":F 300");
    ServiceFunctions::reqDrv(request, response, com, node->get_logger());
    ASSERT_EQ(response->mode, "F");
    ASSERT_EQ(response->value, 300);
  }
  {
    auto request = std::make_shared<psaf_ucbridge_msgs::srv::ReqDrv::Request>();
    auto response = std::make_shared<psaf_ucbridge_msgs::srv::ReqDrv::Response>();
    request->mode = "D";
    com->setReceive(":D -186");
    ServiceFunctions::reqDrv(request, response, com, node->get_logger());
    ASSERT_EQ(response->mode, "D");
    ASSERT_EQ(response->value, -186);
  }

  {
    auto request = std::make_shared<psaf_ucbridge_msgs::srv::ReqDrv::Request>();
    auto response = std::make_shared<psaf_ucbridge_msgs::srv::ReqDrv::Response>();
    request->mode = "D";
    com->setReceive(":F -186");
    ServiceFunctions::reqDrv(request, response, com, node->get_logger());
    ASSERT_FALSE(response->succeed);
  }

  {
    auto request = std::make_shared<psaf_ucbridge_msgs::srv::ReqDrv::Request>();
    auto response = std::make_shared<psaf_ucbridge_msgs::srv::ReqDrv::Response>();
    com->setReceive(":ERR(666): Got empty result from Uc Board");
    ServiceFunctions::reqDrv(request, response, com, node->get_logger());
    ASSERT_FALSE(response->succeed);
  }
  rclcpp::shutdown();
}

TEST(test_service_functions, ReqDms) {
  rclcpp::init(0, nullptr);
  auto node = rclcpp::Node::make_shared("mock_server");
  auto com = std::make_shared<MockCommunication>();
  {
    auto request = std::make_shared<psaf_ucbridge_msgs::srv::ReqDms::Request>();
    auto response = std::make_shared<psaf_ucbridge_msgs::srv::ReqDms::Response>();
    com->setReceive(":OFF ~DMS=1000");
    ServiceFunctions::reqDms(request, response, com, node->get_logger());
    ASSERT_TRUE(response->succeed);
    ASSERT_FALSE(response->on);
    ASSERT_EQ(response->value, 1000);
  }
  {
    auto request = std::make_shared<psaf_ucbridge_msgs::srv::ReqDms::Request>();
    auto response = std::make_shared<psaf_ucbridge_msgs::srv::ReqDms::Response>();
    com->setReceive(":ON ~DMS=1000");
    ServiceFunctions::reqDms(request, response, com, node->get_logger());
    ASSERT_TRUE(response->succeed);
    ASSERT_TRUE(response->on);
    ASSERT_EQ(response->value, 1000);
  }
  {
    auto request = std::make_shared<psaf_ucbridge_msgs::srv::ReqDms::Request>();
    auto response = std::make_shared<psaf_ucbridge_msgs::srv::ReqDms::Response>();
    com->setReceive(":ERR(666): Got empty result from Uc Board");
    ServiceFunctions::reqDms(request, response, com, node->get_logger());
    ASSERT_FALSE(response->succeed);
  }
  rclcpp::shutdown();
}

TEST(test_service_functions, ReqChs) {
  rclcpp::init(0, nullptr);
  auto node = rclcpp::Node::make_shared("mock_server");
  auto com = std::make_shared<MockCommunication>();
  {
    auto request = std::make_shared<psaf_ucbridge_msgs::srv::ReqChs::Request>();
    auto response = std::make_shared<psaf_ucbridge_msgs::srv::ReqChs::Response>();
    com->setReceive(
      ":2\n AX | acc. ahead| opt-dep.!| 1 | I16 \n AY  | acc. left   | opt-dep.!  | 1 | I16");
    ServiceFunctions::reqChs(request, response, com, node->get_logger());
    ASSERT_TRUE(response->succeed);
    ASSERT_EQ(response->num, 2);
    ASSERT_EQ(response->name[0], "AX");
    ASSERT_EQ(response->description[0], "acc. ahead");
    ASSERT_EQ(response->unit[0], "opt-dep.!");
    ASSERT_EQ(response->scantime[0], "1");
    ASSERT_EQ(response->datatype[0], "int16");
    ASSERT_EQ(response->name[1], "AY");
    ASSERT_EQ(response->description[1], "acc. left");
    ASSERT_EQ(response->unit[1], "opt-dep.!");
    ASSERT_EQ(response->scantime[1], "1");
    ASSERT_EQ(response->datatype[1], "int16");
  }
  {
    auto request = std::make_shared<psaf_ucbridge_msgs::srv::ReqChs::Request>();
    request->channels = false;
    request->name = "USF";
    auto response = std::make_shared<psaf_ucbridge_msgs::srv::ReqChs::Response>();
    com->setReceive(":USF        | ultrasonic front distance      | 0.1 mm          | undef | U16");
    ServiceFunctions::reqChs(request, response, com, node->get_logger());
    ASSERT_TRUE(response->succeed);
    ASSERT_EQ(response->num, 1);
    ASSERT_EQ(response->name[0], "USF");
    ASSERT_EQ(response->description[0], "ultrasonic front distance");
    ASSERT_EQ(response->unit[0], "0.1 mm");
    ASSERT_EQ(response->scantime[0], "undef");
    ASSERT_EQ(response->datatype[0], "uint16");
  }
  {
    auto request = std::make_shared<psaf_ucbridge_msgs::srv::ReqChs::Request>();
    auto response = std::make_shared<psaf_ucbridge_msgs::srv::ReqChs::Response>();
    com->setReceive(":ERR(666): Got empty result from Uc Board");
    ServiceFunctions::reqChs(request, response, com, node->get_logger());
    ASSERT_FALSE(response->succeed);
  }
  rclcpp::shutdown();
}

TEST(test_service_functions, ReqGet) {
  rclcpp::init(0, nullptr);
  auto node = rclcpp::Node::make_shared("mock_server");
  auto com = std::make_shared<MockCommunication>();
  {
    auto request = std::make_shared<psaf_ucbridge_msgs::srv::ReqGet::Request>();
    request->names.emplace_back("USF");
    auto response = std::make_shared<psaf_ucbridge_msgs::srv::ReqGet::Response>();
    com->setReceive(":1860");
    ServiceFunctions::reqGet(request, response, com, node->get_logger());
    ASSERT_TRUE(response->succeed);
    ASSERT_EQ(response->value.size(), 1);
    ASSERT_EQ(response->value[0], 1860);
    ASSERT_EQ(response->age.size(), 0);
    ASSERT_EQ(response->ticks.size(), 0);
  }
  {
    auto request = std::make_shared<psaf_ucbridge_msgs::srv::ReqGet::Request>();
    request->names.emplace_back("USF");
    request->age = true;
    auto response = std::make_shared<psaf_ucbridge_msgs::srv::ReqGet::Response>();
    com->setReceive(":1860 68");
    ServiceFunctions::reqGet(request, response, com, node->get_logger());
    ASSERT_TRUE(response->succeed);
    ASSERT_EQ(response->value.size(), 1);
    ASSERT_EQ(response->value[0], 1860);
    ASSERT_EQ(response->age.size(), 1);
    ASSERT_EQ(response->age[0], 68);
    ASSERT_EQ(response->ticks.size(), 0);
  }
  {
    auto request = std::make_shared<psaf_ucbridge_msgs::srv::ReqGet::Request>();
    request->names.emplace_back("USF");
    request->names.emplace_back("UFR");
    request->age = true;
    request->ticks = true;
    auto response = std::make_shared<psaf_ucbridge_msgs::srv::ReqGet::Response>();
    com->setReceive(":1860 68 5 | 233 32 8");
    ServiceFunctions::reqGet(request, response, com, node->get_logger());
    ASSERT_TRUE(response->succeed);
    ASSERT_EQ(response->value.size(), 2);
    ASSERT_EQ(response->value[0], 1860);
    ASSERT_EQ(response->value[1], 233);
    ASSERT_EQ(response->age.size(), 2);
    ASSERT_EQ(response->age[0], 68);
    ASSERT_EQ(response->age[1], 32);
    ASSERT_EQ(response->ticks.size(), 2);
    ASSERT_EQ(response->ticks[0], 5);
    ASSERT_EQ(response->ticks[1], 8);
  }
  {
    auto request = std::make_shared<psaf_ucbridge_msgs::srv::ReqGet::Request>();
    request->names.emplace_back("USF");
    request->names.emplace_back("UFR");
    request->age = true;
    request->ticks = true;
    auto response = std::make_shared<psaf_ucbridge_msgs::srv::ReqGet::Response>();
    com->setReceive(":ERR(666): Got empty result from Uc Board");
    ServiceFunctions::reqGet(request, response, com, node->get_logger());
    ASSERT_FALSE(response->succeed);
  }
  rclcpp::shutdown();
}

TEST(test_services_functions, ReqGrp) {
  rclcpp::init(0, nullptr);
  auto node = rclcpp::Node::make_shared("mock_server");
  auto com = std::make_shared<MockCommunication>();
  {
    auto request = std::make_shared<psaf_ucbridge_msgs::srv::ReqGrp::Request>();
    request->num = 1;
    auto response = std::make_shared<psaf_ucbridge_msgs::srv::ReqGrp::Response>();
    com->setReceive(":\n1  : USL USF USR   ~ENC=B64 ~ALL=10");
    ServiceFunctions::reqGrp(request, response, com, node->get_logger());
    ASSERT_TRUE(response->succeed);
    ASSERT_EQ(response->info.size(), 1);
    ASSERT_EQ(response->info[0], "1  : USL USF USR   ~ENC=B64 ~ALL=10");
  }
  {
    auto request = std::make_shared<psaf_ucbridge_msgs::srv::ReqGrp::Request>();
    request->groups = true;
    auto response = std::make_shared<psaf_ucbridge_msgs::srv::ReqGrp::Response>();
    com->setReceive(
      "\n1  : USL USF USR   ~ENC=B64 ~ALL=10 \n3  : AX AY GZ   ~ENC=B64 ~TS=10 ~AVG=10");
    ServiceFunctions::reqGrp(request, response, com, node->get_logger());
    ASSERT_TRUE(response->succeed);
    ASSERT_EQ(response->info.size(), 2);
    ASSERT_EQ(response->info[0], "1  : USL USF USR   ~ENC=B64 ~ALL=10");
    ASSERT_EQ(response->info[1], "3  : AX AY GZ   ~ENC=B64 ~TS=10 ~AVG=10");
  }
  {
    auto request = std::make_shared<psaf_ucbridge_msgs::srv::ReqGrp::Request>();
    request->groups = true;
    auto response = std::make_shared<psaf_ucbridge_msgs::srv::ReqGrp::Response>();
    com->setReceive(":ERR(666): Got empty result from Uc Board");
    ServiceFunctions::reqGrp(request, response, com, node->get_logger());
    ASSERT_FALSE(response->succeed);
  }
  rclcpp::shutdown();
}

TEST(test_services_functions, ReqVout) {
  rclcpp::init(0, nullptr);
  auto node = rclcpp::Node::make_shared("mock_server");
  auto com = std::make_shared<MockCommunication>();
  {
    auto request = std::make_shared<psaf_ucbridge_msgs::srv::ReqVout::Request>();
    auto response = std::make_shared<psaf_ucbridge_msgs::srv::ReqVout::Response>();
    com->setReceive(":ON");
    ServiceFunctions::reqVout(request, response, com, node->get_logger());
    ASSERT_TRUE(response->succeed);
    ASSERT_TRUE(response->status);
  }
  {
    auto request = std::make_shared<psaf_ucbridge_msgs::srv::ReqVout::Request>();
    auto response = std::make_shared<psaf_ucbridge_msgs::srv::ReqVout::Response>();
    com->setReceive(":OFF");
    ServiceFunctions::reqVout(request, response, com, node->get_logger());
    ASSERT_TRUE(response->succeed);
    ASSERT_FALSE(response->status);
  }
  {
    auto request = std::make_shared<psaf_ucbridge_msgs::srv::ReqVout::Request>();
    auto response = std::make_shared<psaf_ucbridge_msgs::srv::ReqVout::Response>();
    com->setReceive(":ERR(666): Got empty result from Uc Board");
    ServiceFunctions::reqVout(request, response, com, node->get_logger());
    ASSERT_FALSE(response->succeed);
  }
  rclcpp::shutdown();
}

TEST(test_services_functions, ReqUS) {
  rclcpp::init(0, nullptr);
  auto node = rclcpp::Node::make_shared("mock_server");
  auto com = std::make_shared<MockCommunication>();
  {
    auto request = std::make_shared<psaf_ucbridge_msgs::srv::ReqUs::Request>();
    auto response = std::make_shared<psaf_ucbridge_msgs::srv::ReqUs::Response>();
    com->setReceive(":ON");
    ServiceFunctions::reqUs(request, response, com, node->get_logger());
    ASSERT_TRUE(response->succeed);
    ASSERT_TRUE(response->status);
  }
  {
    auto request = std::make_shared<psaf_ucbridge_msgs::srv::ReqUs::Request>();
    request->opt = true;
    auto response = std::make_shared<psaf_ucbridge_msgs::srv::ReqUs::Response>();
    com->setReceive(":~RANGE=100 ~GAIN=10   [4343 mm, 25 ms]");
    ServiceFunctions::reqUs(request, response, com, node->get_logger());
    ASSERT_TRUE(response->succeed);
    ASSERT_EQ(response->range, 100);
    ASSERT_EQ(response->gain, 10);
  }
  {
    auto request = std::make_shared<psaf_ucbridge_msgs::srv::ReqUs::Request>();
    auto response = std::make_shared<psaf_ucbridge_msgs::srv::ReqUs::Response>();
    com->setReceive(":OFF");
    ServiceFunctions::reqUs(request, response, com, node->get_logger());
    ASSERT_TRUE(response->succeed);
    ASSERT_FALSE(response->status);
  }
  {
    auto request = std::make_shared<psaf_ucbridge_msgs::srv::ReqUs::Request>();
    auto response = std::make_shared<psaf_ucbridge_msgs::srv::ReqUs::Response>();
    com->setReceive(":ERR(666): Got empty result from Uc Board");
    ServiceFunctions::reqUs(request, response, com, node->get_logger());
    ASSERT_FALSE(response->succeed);
  }
  rclcpp::shutdown();
}

TEST(test_services_functions, ReqImu) {
  rclcpp::init(0, nullptr);
  auto node = rclcpp::Node::make_shared("mock_server");
  auto com = std::make_shared<MockCommunication>();
  {
    auto request = std::make_shared<psaf_ucbridge_msgs::srv::ReqImu::Request>();
    auto response = std::make_shared<psaf_ucbridge_msgs::srv::ReqImu::Response>();
    com->setReceive(":~ARANGE=8 ~AFILT=3 ~GRANGE=1000 ~GFILT=0   [A: 45 Hz, G: 250 Hz]");
    ServiceFunctions::reqImu(request, response, com, node->get_logger());
    ASSERT_TRUE(response->succeed);
    ASSERT_EQ(response->arange, 8);
    ASSERT_EQ(response->afilt, 3);
    ASSERT_EQ(response->grange, 1000);
    ASSERT_EQ(response->gfilt, 0);
  }
  {
    auto request = std::make_shared<psaf_ucbridge_msgs::srv::ReqImu::Request>();
    auto response = std::make_shared<psaf_ucbridge_msgs::srv::ReqImu::Response>();
    com->setReceive(":ERR(666): Got empty result from Uc Board");
    ServiceFunctions::reqImu(request, response, com, node->get_logger());
    ASSERT_FALSE(response->succeed);
  }
  rclcpp::shutdown();
}

TEST(test_services_functions, ReqMag) {
  rclcpp::init(0, nullptr);
  auto node = rclcpp::Node::make_shared("mock_server");
  auto com = std::make_shared<MockCommunication>();
  {
    auto request = std::make_shared<psaf_ucbridge_msgs::srv::ReqMag::Request>();
    request->opt = true;
    auto response = std::make_shared<psaf_ucbridge_msgs::srv::ReqMag::Response>();
    com->setReceive(":~USEASA=1");
    ServiceFunctions::reqMag(request, response, com, node->get_logger());
    ASSERT_TRUE(response->succeed);
    ASSERT_TRUE(response->status);
  }
  {
    auto request = std::make_shared<psaf_ucbridge_msgs::srv::ReqMag::Request>();
    request->opt = true;
    auto response = std::make_shared<psaf_ucbridge_msgs::srv::ReqMag::Response>();
    com->setReceive(":~USEASA=0");
    ServiceFunctions::reqMag(request, response, com, node->get_logger());
    ASSERT_TRUE(response->succeed);
    ASSERT_FALSE(response->status);
  }
  {
    auto request = std::make_shared<psaf_ucbridge_msgs::srv::ReqMag::Request>();
    request->asa = true;
    auto response = std::make_shared<psaf_ucbridge_msgs::srv::ReqMag::Response>();
    com->setReceive(":176 176 165");
    ServiceFunctions::reqMag(request, response, com, node->get_logger());
    ASSERT_TRUE(response->succeed);
    ASSERT_EQ(response->asavalue[0], 176);
    ASSERT_EQ(response->asavalue[1], 176);
    ASSERT_EQ(response->asavalue[2], 165);
  }
  {
    auto request = std::make_shared<psaf_ucbridge_msgs::srv::ReqMag::Request>();
    request->asa = true;
    request->opt = true;
    auto response = std::make_shared<psaf_ucbridge_msgs::srv::ReqMag::Response>();
    com->setReceive(":176 176 165");
    ServiceFunctions::reqMag(request, response, com, node->get_logger());
    ASSERT_FALSE(response->succeed);
  }
  {
    auto request = std::make_shared<psaf_ucbridge_msgs::srv::ReqMag::Request>();
    request->asa = true;
    auto response = std::make_shared<psaf_ucbridge_msgs::srv::ReqMag::Response>();
    com->setReceive(":ERR(666): Got empty result from Uc Board");
    ServiceFunctions::reqMag(request, response, com, node->get_logger());
    ASSERT_FALSE(response->succeed);
  }
  rclcpp::shutdown();
}

TEST(test_services_functions, ReqEnc) {
  rclcpp::init(0, nullptr);
  auto node = rclcpp::Node::make_shared("mock_server");
  auto com = std::make_shared<MockCommunication>();
  {
    auto request = std::make_shared<psaf_ucbridge_msgs::srv::ReqEnc::Request>();
    auto response = std::make_shared<psaf_ucbridge_msgs::srv::ReqEnc::Response>();
    com->setReceive(":42");
    ServiceFunctions::reqEnc(request, response, com, node->get_logger());
    ASSERT_TRUE(response->succeed);
    ASSERT_EQ(response->steps, 42);
  }

  {
    auto request = std::make_shared<psaf_ucbridge_msgs::srv::ReqEnc::Request>();
    auto response = std::make_shared<psaf_ucbridge_msgs::srv::ReqEnc::Response>();
    com->setReceive(":ERR(666): Got empty result from Uc Board");
    ServiceFunctions::reqEnc(request, response, com, node->get_logger());
    ASSERT_FALSE(response->succeed);
  }
  rclcpp::shutdown();
}
