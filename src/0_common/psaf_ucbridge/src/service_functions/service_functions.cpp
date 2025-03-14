#include <memory>
#include "psaf_ucbridge/service_functions/service_functions.hpp"
#include "psaf_ucbridge/configuration/static_config.hpp"
#include "psaf_ucbridge/sensor_groups/channel_description.hpp"
void ServiceFunctions::reqVer(
  const std::shared_ptr<psaf_ucbridge_msgs::srv::ReqVer::Request> request,
  std::shared_ptr<psaf_ucbridge_msgs::srv::ReqVer::Response> response,
  const ICommunication::SharedPtr & com,
  const rclcpp::Logger & logger)
{
  RCLCPP_INFO(logger, "Received a version request! ");
  auto message = std::make_shared<ReqVer>();
  com->sendCommand(message, std::chrono::seconds(1));
  if (!message->success()) {
    RCLCPP_WARN(
      logger,
      "Version request failed: %s",
      message->errorMessage().c_str());
    response->succeed = false;
  } else {
    RCLCPP_INFO(logger, "Version request succeed.");
    response->succeed = true;
    response->major = message->major();
    response->minor = message->minor();
    response->patch = message->patch();
  }
}

void ServiceFunctions::reqId(
  std::shared_ptr<psaf_ucbridge_msgs::srv::ReqId::Request> request,
  std::shared_ptr<psaf_ucbridge_msgs::srv::ReqId::Response> response,
  const ICommunication::SharedPtr & com,
  const rclcpp::Logger & logger)
{
  RCLCPP_INFO(logger, "Received a ID request!");
  auto message = std::make_shared<ReqID>();
  com->sendCommand(message, std::chrono::seconds(1));
  if (!message->success()) {
    RCLCPP_WARN(
      logger,
      "ID request failed: %s",
      message->errorMessage().c_str());
    response->succeed = false;
  } else {
    RCLCPP_INFO(logger, "ID request succeed.");
    response->succeed = true;
    response->id = message->id();
  }
}

void ServiceFunctions::reqSid(
  std::shared_ptr<psaf_ucbridge_msgs::srv::ReqSid::Request> request,
  std::shared_ptr<psaf_ucbridge_msgs::srv::ReqSid::Response> response,
  const ICommunication::SharedPtr & com,
  const rclcpp::Logger & logger)
{
  RCLCPP_INFO(logger, "Received a SID request!");
  auto message = std::make_shared<ReqSID>();
  com->sendCommand(message, std::chrono::seconds(1));
  if (!message->success()) {
    RCLCPP_WARN(
      logger,
      "SID request failed: %s",
      message->errorMessage().c_str());
    response->succeed = false;
  } else {
    RCLCPP_INFO(logger, "SID request succeed.");
    response->succeed = true;
    response->sid = message->id();
  }
}

void ServiceFunctions::reqSteer(
  std::shared_ptr<psaf_ucbridge_msgs::srv::ReqSteer::Request> request,
  std::shared_ptr<psaf_ucbridge_msgs::srv::ReqSteer::Response> response,
  const ICommunication::SharedPtr & com,
  const rclcpp::Logger & logger)
{
  RCLCPP_INFO(logger, "Received a steer request!");
  auto message = std::make_shared<ReqSTEER>();
  com->sendCommand(message, std::chrono::seconds(1));
  if (!message->success()) {
    RCLCPP_WARN(
      logger,
      "Steer request failed: %s",
      message->errorMessage().c_str());
    response->succeed = false;
  } else {
    RCLCPP_INFO(logger, "Steer request succeed.");
    response->succeed = true;
    response->angle = message->angle();
  }
}

void ServiceFunctions::reqDrv(
  std::shared_ptr<psaf_ucbridge_msgs::srv::ReqDrv::Request> request,
  std::shared_ptr<psaf_ucbridge_msgs::srv::ReqDrv::Response> response,
  const ICommunication::SharedPtr & com,
  const rclcpp::Logger & logger)
{
  RCLCPP_INFO(logger, "Received a Drv request!");
  auto message = std::make_shared<ReqDRV>();
  if (request->mode == "D") {message->direct();} else {
    if (request->mode != "N") {
      RCLCPP_WARN(
        logger, "Invalid request mode(aka %s) in Drv request! Using default!",
        request->mode.c_str());
    }
  }
  com->sendCommand(message, std::chrono::seconds(1));
  if (!message->success()) {
    RCLCPP_WARN(
      logger,
      "Drv request failed: %s",
      message->errorMessage().c_str());
    response->succeed = false;
  } else {
    RCLCPP_INFO(logger, "Drv request succeed.");
    response->succeed = true;
    switch (message->aMode()) {
      case CommandElement::DRVMode::BACKWARDS:
        response->mode = "B";
        break;
      case CommandElement::DRVMode::FORWARDS:
        response->mode = "F";
        break;
      case CommandElement::DRVMode::DIRECT:
        response->mode = "D";
        break;
      default:
        response->mode = "N";
    }
    response->value = message->value();
  }
}

void ServiceFunctions::reqDms(
  std::shared_ptr<psaf_ucbridge_msgs::srv::ReqDms::Request> request,
  std::shared_ptr<psaf_ucbridge_msgs::srv::ReqDms::Response> response,
  const ICommunication::SharedPtr & com,
  const rclcpp::Logger & logger)
{
  RCLCPP_INFO(logger, "Received a Dms request!");
  auto message = std::make_shared<ReqDRV>();
  message->dms();
  com->sendCommand(message, std::chrono::seconds(1));
  if (!message->success()) {
    RCLCPP_WARN(
      logger,
      "Dms request failed: %s",
      message->errorMessage().c_str());
    response->succeed = false;
  } else {
    RCLCPP_INFO(logger, "Dms request succeed.");
    response->succeed = true;
    response->on = message->activate();
    response->value = message->value();
  }
}

void ServiceFunctions::reqChs(
  std::shared_ptr<psaf_ucbridge_msgs::srv::ReqChs::Request> request,
  std::shared_ptr<psaf_ucbridge_msgs::srv::ReqChs::Response> response,
  const ICommunication::SharedPtr & com,

  const rclcpp::Logger & logger)
{
  auto message = std::make_shared<ReqDAQCH>();
  if (request->channels) {
    RCLCPP_INFO(logger, "Received a channels request!");
    message->channels();
    com->sendCommand(message, std::chrono::seconds(1));
    if (!message->success()) {
      RCLCPP_WARN(
        logger,
        "Channels request failed: %s",
        message->errorMessage().c_str());
      response->succeed = false;
      return;
    } else {
      RCLCPP_INFO(logger, "Channels request succeed.");
      response->succeed = true;
    }
  } else {
    RCLCPP_INFO(logger, "Received a channel(%s) request!", request->name.c_str());
    message->name(request->name);
    com->sendCommand(message, std::chrono::seconds(1));
    if (!message->success()) {
      RCLCPP_WARN(
        logger,
        "Channel request failed: %s",
        message->errorMessage().c_str());
      response->succeed = false;
      return;
    } else {
      RCLCPP_INFO(logger, "Channel request succeed.");
      response->succeed = true;
    }
  }
  const auto & descriptions = message->getChannelDescriptions();
  for (const auto & item : descriptions) {
    response->num = static_cast<int>(descriptions.size());
    response->name.emplace_back(item.getName());
    response->description.emplace_back(item.getDescription());
    response->unit.emplace_back(item.getUnit());
    response->scantime.emplace_back(item.getScanTime());
    switch (item.getType()) {
      case ChannelDescription::DataType::Int16:
        response->datatype.emplace_back("int16");
        break;
      case ChannelDescription::DataType::Int8:
        response->datatype.emplace_back("int8");
        break;
      case ChannelDescription::DataType::UInt16:
        response->datatype.emplace_back("uint16");
        break;
      case ChannelDescription::DataType::UInt8:
        response->datatype.emplace_back("uint8");
        break;
      case ChannelDescription::DataType::UNKNOWN:
        response->datatype.emplace_back("unknown");
        break;
    }
  }
}

void ServiceFunctions::reqGet(
  std::shared_ptr<psaf_ucbridge_msgs::srv::ReqGet::Request> request,
  std::shared_ptr<psaf_ucbridge_msgs::srv::ReqGet::Response> response,
  const ICommunication::SharedPtr & com,
  const rclcpp::Logger & logger)
{
  RCLCPP_INFO(logger, "Received a Get request!");
  auto message = std::make_shared<ReqDAQGET>();
  if (request->names.empty()) {
    RCLCPP_WARN(logger, "The name of sensor should not be empty!");
    return;
  }
  for (const auto & item : request->names) {
    message->addSensor(item);
  }
  if (request->age) {message->age();}
  if (request->ticks) {message->tics();}
  com->sendCommand(message, std::chrono::seconds(1));
  if (!message->success()) {
    RCLCPP_WARN(
      logger,
      "Get request failed: %s",
      message->errorMessage().c_str());
    response->succeed = false;
  } else {
    RCLCPP_INFO(logger, "Get request succeed.");
    response->succeed = true;
    for (int i : message->getMeasureValue()) {
      response->value.emplace_back(i);
    }
    for (int i : message->getTicsValue()) {
      response->ticks.emplace_back(i);
    }
    for (int i : message->getAgeValue()) {
      response->age.emplace_back(i);
    }
  }
}

void ServiceFunctions::reqGrp(
  std::shared_ptr<psaf_ucbridge_msgs::srv::ReqGrp::Request> request,
  std::shared_ptr<psaf_ucbridge_msgs::srv::ReqGrp::Response> response,
  const ICommunication::SharedPtr & com,
  const rclcpp::Logger & logger)
{
  auto message = std::make_shared<ReqDAQGRP>();
  if (request->groups) {
    RCLCPP_INFO(logger, "Received a groups request!");
    message->groups();
  } else {
    RCLCPP_INFO(logger, "Received a group(%d) request!", request->num);
    message->setNum(request->num);
  }
  com->sendCommand(message, std::chrono::seconds(1));
  if (!message->success()) {
    if (request->groups) {
      RCLCPP_WARN(logger, "Groups request failed: %s", message->errorMessage().c_str());
    } else {
      RCLCPP_WARN(logger, "Group request failed: %s", message->errorMessage().c_str());
    }
    response->succeed = false;
  } else {
    response->succeed = true;
    if (request->groups) {
      RCLCPP_INFO(logger, "Groups request succeed!");
    } else {
      RCLCPP_INFO(logger, "Group request succeed!");
    }
    for (const auto & item : message->getInfo() ) {
      response->info.emplace_back(item);
    }
  }
}

void ServiceFunctions::reqVout(
  std::shared_ptr<psaf_ucbridge_msgs::srv::ReqVout::Request> request,
  std::shared_ptr<psaf_ucbridge_msgs::srv::ReqVout::Response> response,
  const ICommunication::SharedPtr & com,
  const rclcpp::Logger & logger)
{
  RCLCPP_INFO(logger, "Received a Vout request!");
  auto message = std::make_shared<ReqVOUT>();
  com->sendCommand(message, std::chrono::seconds(1));
  if (!message->success()) {
    RCLCPP_WARN(
      logger,
      "Vout request failed: %s",
      message->errorMessage().c_str());
    response->succeed = false;
  } else {
    RCLCPP_INFO(logger, "Vout request succeed.");
    response->succeed = true;
    response->status = message->getStatus();
  }
}

void ServiceFunctions::reqUs(
  std::shared_ptr<psaf_ucbridge_msgs::srv::ReqUs::Request> request,
  std::shared_ptr<psaf_ucbridge_msgs::srv::ReqUs::Response> response,
  const ICommunication::SharedPtr & com,
  const rclcpp::Logger & logger)
{
  auto message = std::make_shared<ReqUS>();
  if (request->opt) {
    RCLCPP_INFO(logger, "Received a US OPT request!");
    message->opt();
  } else {
    RCLCPP_INFO(logger, "Received a US request!");
  }
  com->sendCommand(message, std::chrono::seconds(1));
  if (!message->success()) {
    RCLCPP_WARN(
      logger,
      "US request failed: %s",
      message->errorMessage().c_str());
    response->succeed = false;
  } else {
    RCLCPP_INFO(logger, "US request succeed.");
    response->succeed = true;
    if (request->opt) {
      response->gain = message->getGain();
      response->range = message->getRange();
    } else {response->status = message->getStatus();}
  }
}

void ServiceFunctions::reqImu(
  std::shared_ptr<psaf_ucbridge_msgs::srv::ReqImu::Request> request,
  std::shared_ptr<psaf_ucbridge_msgs::srv::ReqImu::Response> response,
  const ICommunication::SharedPtr & com,
  const rclcpp::Logger & logger)
{
  RCLCPP_INFO(logger, "Received a IMU request!");
  auto message = std::make_shared<ReqIMU>();
  com->sendCommand(message, std::chrono::seconds(1));
  if (!message->success()) {
    RCLCPP_WARN(
      logger,
      "IMU request failed: %s",
      message->errorMessage().c_str());
    response->succeed = false;
    return;
  } else {
    RCLCPP_INFO(logger, "IMU request succeed.");
    response->succeed = true;
    response->arange = message->getArange();
    response->afilt = message->getAfilt();
    response->grange = message->getGrange();
    response->gfilt = message->getGfilt();
  }
}

void ServiceFunctions::reqMag(
  std::shared_ptr<psaf_ucbridge_msgs::srv::ReqMag::Request> request,
  std::shared_ptr<psaf_ucbridge_msgs::srv::ReqMag::Response> response,
  const ICommunication::SharedPtr & com,
  const rclcpp::Logger & logger)
{
  auto message = std::make_shared<ReqMAG>();
  if (request->opt && !request->asa) {
    RCLCPP_INFO(logger, "Received a Mag opt request!");
    message->opt();
  } else if (request->asa && !request->opt) {
    RCLCPP_INFO(logger, "Received a Mag asa request!");
    message->asa();
  } else {
    RCLCPP_WARN(logger, "In Mag request one and only one of the opt and asa should be true!");
    response->succeed = false;
    return;
  }
  com->sendCommand(message, std::chrono::seconds(1));
  if (!message->success()) {
    RCLCPP_WARN(
      logger,
      "MAG request failed: %s",
      message->errorMessage().c_str());
    response->succeed = false;
    return;
  }
  RCLCPP_INFO(logger, "Mag request succeed.");
  response->succeed = true;
  if (request->opt) {response->status = message->getUseasa();}
  if (request->asa) {
    for (const auto & item : message->getAsavalue() ) {
      response->asavalue.emplace_back(item);
    }
  }
}

void ServiceFunctions::reqEnc(
  std::shared_ptr<psaf_ucbridge_msgs::srv::ReqEnc::Request> request,
  std::shared_ptr<psaf_ucbridge_msgs::srv::ReqEnc::Response> response,
  const ICommunication::SharedPtr & com,
  const rclcpp::Logger & logger)
{
  RCLCPP_INFO(logger, "Received a Enc request!");
  auto message = std::make_shared<ReqENC>();
  com->sendCommand(message, std::chrono::seconds(1));
  if (!message->success()) {
    RCLCPP_WARN(
      logger,
      "Enc request failed: %s",
      message->errorMessage().c_str());
    response->succeed = false;
  } else {
    RCLCPP_INFO(logger, "Enc request succeed.");
    response->succeed = true;
    response->steps = message->steps();
  }
}

void ServiceFunctions::reqRaw(
  std::shared_ptr<psaf_ucbridge_msgs::srv::ReqRaw::Request> request,
  std::shared_ptr<psaf_ucbridge_msgs::srv::ReqRaw::Response> response,
  const ICommunication::SharedPtr & com,
  const rclcpp::Logger & logger)
{
  RCLCPP_INFO(logger, "Received a raw communication request!");
  auto message = std::make_shared<RawMessage>(request->raw);
  if (request->type == "ANSYC") {
    message->setType(CommandElement::ASYNC);
  } else if (request->type == "DISCARD") {message->setType(CommandElement::DISCARD);} else {
    message->setType(CommandElement::WAIT);
  }
  com->sendCommand(message, std::chrono::milliseconds(request->timeout));
  if (!message->success()) {
    RCLCPP_WARN(
      logger,
      "Raw communication gets error: %s",
      message->errorMessage().c_str());
    response->succeed = false;
    response->answer = "";
  } else {
    response->succeed = true;
    response->answer = message->getAnswer();
  }
}
