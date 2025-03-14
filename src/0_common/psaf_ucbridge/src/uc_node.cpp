#include <string>
#include <memory>
#include <utility>
#include <map>
#include <chrono>
#include "psaf_ucbridge/configuration/configuration.hpp"
#include "psaf_ucbridge/uc_node.hpp"
#include "psaf_ucbridge/sensor_groups/sensor_group.hpp"
#include "psaf_ucbridge/logging/logger_factory.hpp"
#include "psaf_ucbridge_msgs/msg/raw_message.hpp"
#include "psaf_ucbridge_msgs/srv/req_ver.hpp"

using namespace std::placeholders;
UcNode::~UcNode()
{
  receiver->unregisterSensorGroups();
  auto message1 = std::make_shared<ComDAQ>();
  message1->stop();
  auto message2 = std::make_shared<ComUS>();
  message2->off();
//  com->sendCommand(std::shared_ptr<ComDAQ>(&ComDAQ().stop()), std::chrono::microseconds(100000));
//  com->sendCommand(std::shared_ptr<ComUS>(&ComUS().off()), std::chrono::microseconds(100000));
  com->sendCommand(message1, std::chrono::microseconds(100000));
  com->sendCommand(message2, std::chrono::microseconds(100000));
  message1->success();
  message2->success();
  if (Configuration::instance().reset_end()) {resetUcBoard();}
  reader->stopThread();
  receiver->stopThread();
  com->stopThread();
  LoggerFactory::instance().unregisterLoggerBackend();
  RCLCPP_INFO(this->get_logger(), "uc_node was deleted");
}

UcNode::UcNode()
: Node("uc_bridge"), QOS(rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile())
{
  RCLCPP_INFO(this->get_logger(), "Initializing uc_node");

  RCLCPP_INFO(this->get_logger(), "Read configuration from file");
  Configuration & conf = Configuration::instance();
  conf.readConfigurationFromNode(*this);

  RCLCPP_INFO(this->get_logger(), "Initialize logger framework");
  initializeLogger();

  RCLCPP_INFO(this->get_logger(), conf.serialPort().c_str());

  // message queue for message answers
  MessageQueue<std::string>::SharedPtr commandResult =
    std::make_shared<MessageQueue<std::string>>();

  // message queue for data answers
  MessageQueue<std::string>::SharedPtr dataResult =
    std::make_shared<MessageQueue<std::string>>();

  // Board communication
  boardCommunication = BoardCommunication::create(conf);
  // reading thread
  reader = std::make_unique<ReadingThread>(dataResult, commandResult, boardCommunication);

  receiver = std::make_unique<DataReceiver>(dataResult);
  com = std::make_shared<Communication>(commandResult, boardCommunication);
  com->startThread();

  /// service relevant
  /// req Version
  registerService
  <rclcpp::Service<psaf_ucbridge_msgs::srv::ReqVer>::SharedPtr,
    psaf_ucbridge_msgs::srv::ReqVer,
    std::shared_ptr<psaf_ucbridge_msgs::srv::ReqVer::Request>,
    std::shared_ptr<psaf_ucbridge_msgs::srv::ReqVer::Response>>(
    service_ver,
    &ServiceFunctions::reqVer,
    conf.kServiceReqVer);

  /// req ID
  registerService
  <rclcpp::Service<psaf_ucbridge_msgs::srv::ReqId>::SharedPtr,
    psaf_ucbridge_msgs::srv::ReqId,
    std::shared_ptr<psaf_ucbridge_msgs::srv::ReqId::Request>,
    std::shared_ptr<psaf_ucbridge_msgs::srv::ReqId::Response>>(
    service_id, &ServiceFunctions::reqId,
    conf.kServiceReqId);

  /// req SID
  registerService
  <rclcpp::Service<psaf_ucbridge_msgs::srv::ReqSid>::SharedPtr,
    psaf_ucbridge_msgs::srv::ReqSid,
    std::shared_ptr<psaf_ucbridge_msgs::srv::ReqSid::Request>,
    std::shared_ptr<psaf_ucbridge_msgs::srv::ReqSid::Response>>(
    service_sid,
    &ServiceFunctions::reqSid,
    conf.kServiceReqSid);

  /// req Steer
  registerService
  <rclcpp::Service<psaf_ucbridge_msgs::srv::ReqSteer>::SharedPtr,
    psaf_ucbridge_msgs::srv::ReqSteer,
    std::shared_ptr<psaf_ucbridge_msgs::srv::ReqSteer::Request>,
    std::shared_ptr<psaf_ucbridge_msgs::srv::ReqSteer::Response>>(
    service_steer,
    &ServiceFunctions::reqSteer,
    conf.kServiceReqSteer);

  /// req Drv
  registerService
  <rclcpp::Service<psaf_ucbridge_msgs::srv::ReqDrv>::SharedPtr,
    psaf_ucbridge_msgs::srv::ReqDrv,
    std::shared_ptr<psaf_ucbridge_msgs::srv::ReqDrv::Request>,
    std::shared_ptr<psaf_ucbridge_msgs::srv::ReqDrv::Response>>(
    service_drv,
    &ServiceFunctions::reqDrv,
    conf.kServiceReqDrv);

  /// req Dms
  registerService
  <rclcpp::Service<psaf_ucbridge_msgs::srv::ReqDms>::SharedPtr,
    psaf_ucbridge_msgs::srv::ReqDms,
    std::shared_ptr<psaf_ucbridge_msgs::srv::ReqDms::Request>,
    std::shared_ptr<psaf_ucbridge_msgs::srv::ReqDms::Response>>(
    service_dms,
    &ServiceFunctions::reqDms,
    conf.kServiceReqDms);

  /// req Channel or channels
  registerService
  <rclcpp::Service<psaf_ucbridge_msgs::srv::ReqChs>::SharedPtr,
    psaf_ucbridge_msgs::srv::ReqChs,
    std::shared_ptr<psaf_ucbridge_msgs::srv::ReqChs::Request>,
    std::shared_ptr<psaf_ucbridge_msgs::srv::ReqChs::Response>>(
    service_chs,
    &ServiceFunctions::reqChs,
    conf.kServiceReqChs);

  /// req get measure value
  registerService
  <rclcpp::Service<psaf_ucbridge_msgs::srv::ReqGet>::SharedPtr,
    psaf_ucbridge_msgs::srv::ReqGet,
    std::shared_ptr<psaf_ucbridge_msgs::srv::ReqGet::Request>,
    std::shared_ptr<psaf_ucbridge_msgs::srv::ReqGet::Response>>(
    service_get,
    &ServiceFunctions::reqGet,
    conf.kServiceReqGet);

  /// req grp
  registerService
  <rclcpp::Service<psaf_ucbridge_msgs::srv::ReqGrp>::SharedPtr,
    psaf_ucbridge_msgs::srv::ReqGrp,
    std::shared_ptr<psaf_ucbridge_msgs::srv::ReqGrp::Request>,
    std::shared_ptr<psaf_ucbridge_msgs::srv::ReqGrp::Response>>(
    service_grp,
    &ServiceFunctions::reqGrp,
    conf.kServiceReqGrp);

  /// req vout
  registerService
  <rclcpp::Service<psaf_ucbridge_msgs::srv::ReqVout>::SharedPtr,
    psaf_ucbridge_msgs::srv::ReqVout,
    std::shared_ptr<psaf_ucbridge_msgs::srv::ReqVout::Request>,
    std::shared_ptr<psaf_ucbridge_msgs::srv::ReqVout::Response>>(
    service_vout,
    &ServiceFunctions::reqVout,
    conf.kServiceReqVout);

  /// req us
  registerService
  <rclcpp::Service<psaf_ucbridge_msgs::srv::ReqUs>::SharedPtr,
    psaf_ucbridge_msgs::srv::ReqUs,
    std::shared_ptr<psaf_ucbridge_msgs::srv::ReqUs::Request>,
    std::shared_ptr<psaf_ucbridge_msgs::srv::ReqUs::Response>>(
    service_us, &ServiceFunctions::reqUs,
    conf.kServiceReqUs);

  /// req imu
  registerService
  <rclcpp::Service<psaf_ucbridge_msgs::srv::ReqImu>::SharedPtr,
    psaf_ucbridge_msgs::srv::ReqImu,
    std::shared_ptr<psaf_ucbridge_msgs::srv::ReqImu::Request>,
    std::shared_ptr<psaf_ucbridge_msgs::srv::ReqImu::Response>>(
    service_imu,
    &ServiceFunctions::reqImu,
    conf.kServiceReqImu);

  /// req mag
  registerService
  <rclcpp::Service<psaf_ucbridge_msgs::srv::ReqMag>::SharedPtr,
    psaf_ucbridge_msgs::srv::ReqMag,
    std::shared_ptr<psaf_ucbridge_msgs::srv::ReqMag::Request>,
    std::shared_ptr<psaf_ucbridge_msgs::srv::ReqMag::Response>>(
    service_mag,
    &ServiceFunctions::reqMag,
    conf.kServiceReqMag);

  /// req enc
  if (conf.board_version() == 2) {
    registerService
    <rclcpp::Service<psaf_ucbridge_msgs::srv::ReqEnc>::SharedPtr,
      psaf_ucbridge_msgs::srv::ReqEnc,
      std::shared_ptr<psaf_ucbridge_msgs::srv::ReqEnc::Request>,
      std::shared_ptr<psaf_ucbridge_msgs::srv::ReqEnc::Response>>(
      service_enc,
      &ServiceFunctions::reqEnc,
      conf.kServiceReqEnc);
  }


  /// raw communication
  registerService
  <rclcpp::Service<psaf_ucbridge_msgs::srv::ReqRaw>::SharedPtr,
    psaf_ucbridge_msgs::srv::ReqRaw,
    std::shared_ptr<psaf_ucbridge_msgs::srv::ReqRaw::Request>,
    std::shared_ptr<psaf_ucbridge_msgs::srv::ReqRaw::Response>>(
    service_raw,
    &ServiceFunctions::reqRaw,
    conf.kServiceReqRaw
    );

  // register callbacks
  subscriptions.push_back(
    this->create_subscription<std_msgs::msg::Int16>(
      conf.kTopicSetMotorLevelForward,
      QOS,
      [this](const std_msgs::msg::Int16::SharedPtr msg) {
        set_motor_level_forwards_callback(msg);
      }
  ));

  subscriptions.push_back(
    this->create_subscription<std_msgs::msg::Int16>(
      conf.kTopicSetMotorLevelBackward,
      QOS,
      [this](const std_msgs::msg::Int16::SharedPtr msg) {
        set_motor_level_backwards_callback(msg);
      }
  ));

  subscriptions.push_back(
    this->create_subscription<std_msgs::msg::Int16>(
      conf.kTopicSetSteeringAngle,
      QOS,
      [this](const std_msgs::msg::Int16::SharedPtr msg) {
        set_steer_angle_callback(msg);
      }
  ));

  subscriptions.push_back(
    this->create_subscription<std_msgs::msg::Int8>(
      conf.kTopicSetLed,
      QOS,
      [this](const std_msgs::msg::Int8::SharedPtr msg) {set_led_callback(msg);}));

  if (conf.reset_start()) {resetUcBoard();}


  reader->startThread();
  receiver->startThread();

  publisher_speed = this->create_publisher<std_msgs::msg::Int16>(conf.kTopicSpeed, QOS);
  publisher_steering = this->create_publisher<std_msgs::msg::Int16>(conf.kTopicSteer, QOS);
  publisher_button = this->create_publisher<std_msgs::msg::Int8>(conf.kTopicButton, QOS);


  if (conf.activateTxtDisplay()) {
    publisher_dis = this->create_publisher<psaf_ucbridge_msgs::msg::Display>(
      conf.kTopicTxtDisplay,
      QOS);
    createDisplayTxt();
  } else {
    RCLCPP_INFO(this->get_logger(), "Do not display txt message");
  }

  getChannels();
  setDeadMan();
  if (conf.activateImuSensorGroup()) {
    // init publisher
    publisher_imu = this->create_publisher<sensor_msgs::msg::Imu>(conf.kTopicImuData, QOS);
    createImuGroup();
  } else {
    RCLCPP_INFO(this->get_logger(), "Do not register Imu sensor group");
  }

  if (conf.activateUSSensorGroup()) {
    createUSGroup();
  } else {
    RCLCPP_INFO(this->get_logger(), "Do not register US sensor group");
  }

  timer2_ = this->create_wall_timer(
    std::chrono::milliseconds(conf.steer_frequency()),
    [this]() {send_steering_callback();});

  if (conf.board_version() == 1) {
    RCLCPP_INFO(this->get_logger(), "Board version is 1, using hall sensor.");
    if (conf.activateMagSensorGroup()) {
      publisher_mag =
        this->create_publisher<sensor_msgs::msg::MagneticField>(conf.kTopicMagData, QOS);
      createMagGroup();
    } else {
      RCLCPP_INFO(this->get_logger(), "Do not register Mag sensor group");
    }

    if (conf.activateHallDtSensorGroup()) {
      publisher_dt = this->create_publisher<std_msgs::msg::Float32>(
        conf.kTopicDtData, QOS);
      publisher_cnt = this->create_publisher<std_msgs::msg::UInt8>(conf.kTopicHallCntData, QOS);
      createDtGroup();
    } else {
      RCLCPP_INFO(this->get_logger(), "Do not register hall/dt sensor group");
    }

    if (conf.activateHallDt8SensorGroup()) {
      publisher_dt8 = this->create_publisher<std_msgs::msg::Float32>(
        conf.kTopicDt8Data, QOS);
      createDt8Group();
    } else {
      RCLCPP_INFO(this->get_logger(), "Do not register dt8 sensor group");
    }
  } else if (conf.board_version() == 2) {
    RCLCPP_INFO(this->get_logger(), "Board version is 2, using enc sensor.");
    if (conf.activateEncStepSensorGroup()) {
      publisher_enc_steps = this->create_publisher<psaf_ucbridge_msgs::msg::EncStep>(
        conf.kTopicEncStepData, QOS);
      createEncStepGroup();
    } else {
      RCLCPP_INFO(this->get_logger(), "Do not register enc step sensor group");
    }

    if (conf.activateEncCmSensorGroup()) {
      publisher_enc_cms = this->create_publisher<psaf_ucbridge_msgs::msg::EncCm>(
        conf.kTopicEncCmData, QOS);
      createEncCmGroup();
    } else {
      RCLCPP_INFO(this->get_logger(), "Do not register enc cm sensor group");
    }

    if (conf.activateRc()) {
      RCLCPP_INFO(this->get_logger(), "Create manual signal!");
      publisher_rc = this->create_publisher<std_msgs::msg::UInt8>(conf.kTopicRc, QOS);
      timer1_ = this->create_wall_timer(
        std::chrono::milliseconds(conf.rc_frequency()),
        [this]() {send_rc_callback();});
    } else {
      RCLCPP_INFO(this->get_logger(), "Do not register RCMODE");
    }
  } else {
    RCLCPP_WARN(this->get_logger(), "Board version is not support! No speed info!");
  }

  if (conf.activatePbSensorGroup()) {
    publisher_pb = this->create_publisher<psaf_ucbridge_msgs::msg::Pbs>(
      conf.kTopicPbData, QOS);
    createPbGroup();
  } else {
    RCLCPP_INFO(this->get_logger(), "Do not register pb sensor group");
  }

  if (conf.activateVbatSensorGroup()) {
    publisher_vbat = this->create_publisher<psaf_ucbridge_msgs::msg::Vbat>(
      conf.kTopicVbatData, QOS);
    createVbatGroup();
  } else {
    RCLCPP_INFO(this->get_logger(), "Do not register vbat sensor group");
  }

//  if (conf.activateRawCommunication()) {
//    subscriptions.push_back(
//      this->create_subscription<psaf_ucbridge_msgs::msg::RawMessage>(
//        conf.kTopicRawCommunication,
//        QOS,
//        [this](psaf_ucbridge_msgs::msg::RawMessage::SharedPtr msg) {
//          raw_communication_callback(msg);
//        }
//      )
//    );
//  }

  if (conf.activateUnkGrp()) {
    subscriptions.push_back(
      this->create_subscription<psaf_ucbridge_msgs::msg::UnkDaq>(
        conf.kTopicSetUnkGrp,
        QOS,
        [this](psaf_ucbridge_msgs::msg::UnkDaq::SharedPtr msg) {
          unknown_channel_callback(msg);
        }
      )
    );
    publisher_unk = this->create_publisher<std_msgs::msg::String>(
      conf.kTopicUnkGrp, QOS
    );
  }
  startDAQ();

  // this is for the case that nothing is queued in rclcpp and the program has to be terminated
  timer_ = this->create_wall_timer(node_interval, []() {});

  RCLCPP_INFO(this->get_logger(), "uc_node was initialized");
}

void UcNode::set_motor_level_forwards_callback(const std_msgs::msg::Int16::SharedPtr & msg)
{
  auto message = std::make_shared<ComDRV>();
  message->setType(CommandElement::ASYNC);
  message->forwards(msg->data);
  com->sendCommand(message, std::chrono::microseconds(100000));
  if (!message->success()) {
    RCLCPP_WARN(
      this->get_logger(), "Was not able to set motor level (forwards) %s",
      message->errorMessage().c_str());
  }
}

void UcNode::set_motor_level_backwards_callback(const std_msgs::msg::Int16::SharedPtr & msg)
{
  auto message = std::make_shared<ComDRV>();
//  ComDRV message;
  message->backwards(msg->data);
  com->sendCommand(message, std::chrono::microseconds(100000));
  if (!message->success()) {
    RCLCPP_WARN(
      this->get_logger(),
      "Was not able to set motor level (backwards): %s",
      message->errorMessage().c_str());
  }
}

void UcNode::set_steer_angle_callback(const std_msgs::msg::Int16::SharedPtr & msg)
{
  auto message = std::make_shared<ComSTEER>();
  int16_t data = 0;
  if (Configuration::instance().board_version() == 1) {
    data = (msg->data > 300) ? 300 : msg->data;
    data = data * 10 / 3;
  } else {
    data = (msg->data > 450) ? 450 : msg->data;
    data = data * 100 / 45;
  }
  // RCLCPP_INFO(this->get_logger(), "%d", data);
  message->steer(data).repeat(true);
  com->sendCommand(message, std::chrono::microseconds(100000));
  if (!message->success()) {
    RCLCPP_WARN(
      this->get_logger(),
      "Was not able to set steering angle: %s",
      message->errorMessage().c_str());
  }
}

void UcNode::set_led_callback(const std_msgs::msg::Int8::SharedPtr & msg)
{
  auto message = std::make_shared<ComLED>();
  switch (msg->data) {
    case 0:
      message->off(CommandElement::A)
      .off(CommandElement::B)
      .off(CommandElement::BRMS)
      .off(CommandElement::BLKR)
      .off(CommandElement::BLKL);
      break;
    case 1: message->on(CommandElement::BRMS);
      break;
    case 2: message->off(CommandElement::BRMS);
      break;
    case 5: message->blink(CommandElement::BLKL, 25, 25);
      break;
    case 6: message->off(CommandElement::BLKL);
      break;
    case 7: message->blink(CommandElement::BLKR, 25, 25);
      break;
    case 8: message->off(CommandElement::BLKR);
      break;
    case 9: message->blink(CommandElement::BLKR, 25, 25)
      .blink(CommandElement::BLKL, 25, 25);
      break;
    case 10: message->off(CommandElement::BLKL)
      .off(CommandElement::BLKR);
      break;
    default: break;
  }
  com->sendCommand(message, std::chrono::microseconds(100000));
  if (!message->success()) {
    RCLCPP_WARN(this->get_logger(), "Was not able to set LEDs");
  }
}

void UcNode::createHeader(std_msgs::msg::Header & header, rclcpp::Time & t)
{
  header.stamp.sec = t.seconds();
  header.stamp.nanosec = t.nanoseconds();
}

void UcNode::createImuGroup()
{
  RCLCPP_INFO(this->get_logger(), "Register IMU sensor group");
  Configuration & conf = Configuration::instance();
  auto message = std::make_shared<ComDAQ>();
  // configuration for imu group
//  ComDAQ message;
  message->group(Configuration::instance().kImuSensorGroupNo)
  .channel(CommandElement::AX)
  .channel(CommandElement::AY)
  .channel(CommandElement::AZ)
  .channel(CommandElement::GX)
  .channel(CommandElement::GY)
  .channel(CommandElement::GZ)
  .ts(conf.imuTs())
  .avg();
  SensorGroup::UniquePtr group = std::make_unique<SensorGroup>(*message);
  // bind callback to sensor group
  group->registerCallback([this](SensorGroup & group) {sendImuData(group);});
  // register sensor group in DataReceiver object
  receiver->registerSensorGroup(std::move(group));
  com->sendCommand(message, std::chrono::seconds(1));
  if (!message->success()) {
    RCLCPP_WARN(
      this->get_logger(),
      "Could not register Imu Sensor Group: %s", message->errorMessage().c_str());
  }
}

void UcNode::createUSGroup()
{
  RCLCPP_INFO(this->get_logger(), "Activate US sensors");
  auto activationCom = std::make_shared<ComUS>();
  activationCom->on();
//  ComUS activationCom;
  com->sendCommand(activationCom, std::chrono::seconds(1));
  if (!activationCom->success()) {
    RCLCPP_WARN(
      this->get_logger(),
      "Could not activate US: %s",
      activationCom->errorMessage().c_str());
  }

  RCLCPP_INFO(this->get_logger(), "Register US sensor group");
  // configuration for us group
  auto message = std::make_shared<ComDAQ>();
  message->group(Configuration::instance().kUSSensorGroupNo);
  range_publishers.clear();
  for (ChannelDescription & d : channelDescriptions) {
    if (CommandElement::command_element().kDaqUsChannels.end() !=
      CommandElement::command_element().kDaqUsChannels.find(d.getName()))
    {
      RCLCPP_DEBUG(this->get_logger(), "Adding channel %s to US sensor group", d.getName().c_str());
      message->channel(CommandElement::command_element().kDaqUsChannels.at(d.getName()));
      range_publishers.push_back(
        this->create_publisher<sensor_msgs::msg::Range>(
          Configuration::instance().kTopicRange.at(d.getName()), QOS));
    }
  }

  message->all();

  SensorGroup::UniquePtr group = std::make_unique<SensorGroup>(*message);
  group->registerCallback([this](SensorGroup & group) {sendUSData(group);});
  receiver->registerSensorGroup(std::move(group));
  com->sendCommand(message, std::chrono::seconds(1));
  if (!message->success()) {
    RCLCPP_WARN(
      this->get_logger(),
      "Could not register US Sensor Group: %s", message->errorMessage().c_str());
  }
}

void UcNode::createMagGroup()
{
  RCLCPP_INFO(this->get_logger(), "Use correction data from magnetometer");
  auto useAsaCommand = std::make_shared<ComMAG>();
  useAsaCommand->useasa();
  com->sendCommand(useAsaCommand, std::chrono::seconds(1));
  if (!useAsaCommand->success()) {
    RCLCPP_WARN(
      this->get_logger(),
      "Could not set correction data: %s", useAsaCommand->errorMessage().c_str());
  }

  RCLCPP_INFO(this->get_logger(), "Register Mag sensor group");
  // configuration for mag group
  auto message = std::make_shared<ComDAQ>();
  message->group(Configuration::instance().kMagSensorGroupNo)
  .channel(CommandElement::MX)
  .channel(CommandElement::MY)
  .channel(CommandElement::MZ);
  SensorGroup::UniquePtr group = std::make_unique<SensorGroup>(*message);
  group->registerCallback([this](SensorGroup & group) {sendMagData(group);});
  receiver->registerSensorGroup(std::move(group));
  com->sendCommand(message, std::chrono::seconds(1));
  if (!message->success()) {
    RCLCPP_WARN(
      this->get_logger(),
      "Could not register Mag Sensor Group: %s", message->errorMessage().c_str());
  }
}

void UcNode::createDtGroup()
{
  RCLCPP_INFO(this->get_logger(), "Register dt/cnt sensor group");
  auto message = std::make_shared<ComDAQ>();
  // configuration for dt group
  message->group(Configuration::instance().kHallDtSensorGroupNo)
  .channel(CommandElement::HALL_CNT)
  .channel(CommandElement::HALL_DT);
  SensorGroup::UniquePtr group = std::make_unique<SensorGroup>(*message);
  group->registerCallback([this](SensorGroup & grp) {sendHallDtData(grp);});
  receiver->registerSensorGroup(std::move(group));
  com->sendCommand(message, std::chrono::seconds(1));
  if (!message->success()) {
    RCLCPP_WARN(
      this->get_logger(),
      "Could not register dt/cnt Sensor Group: %s", message->errorMessage().c_str());
  }
}

void UcNode::createDt8Group()
{
  RCLCPP_INFO(this->get_logger(), "Register dt8 sensor group");
  // configuration for dt8 group
  auto message = std::make_shared<ComDAQ>();
  message->group(Configuration::instance().kHallDt8SensorGroupNo)
  .channel(CommandElement::HALL_DT8);
  SensorGroup::UniquePtr group = std::make_unique<SensorGroup>(*message);
  group->registerCallback([this](SensorGroup & grp) {sendHallDt8Data(grp);});
  receiver->registerSensorGroup(std::move(group));
  com->sendCommand(message, std::chrono::seconds(1));
  if (!message->success()) {
    RCLCPP_WARN(
      this->get_logger(),
      "Could not register dt8 Sensor Group: %s", message->errorMessage().c_str());
  }
}

void UcNode::createEncStepGroup()
{
  RCLCPP_INFO(this->get_logger(), "Register enc step sensor group");
  // configuration for enc step group
  auto message = std::make_shared<ComDAQ>();
  message->group(Configuration::instance().kEncStepSensorGroupNo)
  .channel(CommandElement::ENC_STEPS)
  .channel(CommandElement::TICS)
  .skip(Configuration::instance().encStepSkip());
  message->tics();
  SensorGroup::UniquePtr group = std::make_unique<SensorGroup>(*message);
  std::cout << group->numberOfChannels();
  group->registerCallback([this](SensorGroup & grp) {sendEncStepData(grp);});
  receiver->registerSensorGroup(std::move(group));
  com->sendCommand(message, std::chrono::seconds(1));
  if (!message->success()) {
    RCLCPP_WARN(
      this->get_logger(),
      "Could not register enc_step Sensor Group: %s", message->errorMessage().c_str());
  }
}

void UcNode::createEncCmGroup()
{
  RCLCPP_INFO(this->get_logger(), "Register enc cm sensor group");
  // configuration for enc cm group
  auto message = std::make_shared<ComDAQ>();
  message->group(Configuration::instance().kEncCmSensorGroupNo).channel(CommandElement::ENC_CMS)
  .skip(Configuration::instance().encCmSkip());
  SensorGroup::UniquePtr group = std::make_unique<SensorGroup>(*message);
  group->registerCallback([this](SensorGroup & grp) {sendEncCmData(grp);});
  receiver->registerSensorGroup(std::move(group));
  com->sendCommand(message, std::chrono::seconds(1));
  if (!message->success()) {
    RCLCPP_WARN(
      this->get_logger(),
      "Could not register enc_cm Sensor Group: %s", message->errorMessage().c_str());
  }
}

void UcNode::createPbGroup()
{
  RCLCPP_INFO(this->get_logger(), "Register PB sensor group");
  auto message = std::make_shared<ComDAQ>();
  message->group(Configuration::instance().kButtonsSensorGroupNo)
  .channel(CommandElement::PBA)
  .channel(CommandElement::PBB)
  .channel(CommandElement::PBC);
  SensorGroup::UniquePtr group = std::make_unique<SensorGroup>(*message);
  group->registerCallback([this](SensorGroup & grp) {sendPbData(grp);});
  receiver->registerSensorGroup(std::move(group));
  com->sendCommand(message, std::chrono::seconds(1));
  if (!message->success()) {
    RCLCPP_WARN(
      this->get_logger(),
      "Could not register PB Sensor Group: %s", message->errorMessage().c_str());
  }
}

void UcNode::createVbatGroup()
{
  RCLCPP_INFO(this->get_logger(), "Register VBAT sensor group");
  auto message = std::make_shared<ComDAQ>();
  message->group(Configuration::instance().kBatteryVoltageSensorGroupNo)
  .channel(CommandElement::VSBAT)
  .channel(CommandElement::VDBAT);
  SensorGroup::UniquePtr group = std::make_unique<SensorGroup>(*message);
  group->registerCallback([this](SensorGroup & grp) {sendVbatData(grp);});
  receiver->registerSensorGroup(std::move(group));
  com->sendCommand(message, std::chrono::seconds(1));
  if (!message->success()) {
    RCLCPP_WARN(
      this->get_logger(),
      "Could not register VBAT Sensor Group: %s", message->errorMessage().c_str());
  }
}

void UcNode::createDisplayTxt()
{
  RCLCPP_INFO(this->get_logger(), "Using Txt display!");
  DisplayTxt::UniquePtr dis = std::make_unique<DisplayTxt>();
  dis->registerCallback([this](DisplayTxt & display) {sendDisplay(display);});
  if (!dis->valid()) {
    RCLCPP_WARN(
      this->get_logger(),
      "Txt display failed!");
  }
  receiver->registerDisplayTxt(std::move(dis));
}

void UcNode::startDAQ()
{
  RCLCPP_INFO(this->get_logger(), "Start DAQ");
  auto cmd = std::make_shared<ComDAQ>();
  cmd->start();
  com->sendCommand(cmd, std::chrono::seconds(1));
  if (!cmd->success()) {
    RCLCPP_WARN(this->get_logger(), "Could not start DAQ: %s", cmd->errorMessage().c_str());
  }
}

bool UcNode::resetUcBoard()
{
  RCLCPP_INFO(this->get_logger(), "Wait 3s for Board reset..");
  auto cmd = std::make_shared<ComReset>();
  com->sendCommand(cmd, std::chrono::seconds(3));
  return cmd->success();
}

void UcNode::sendUSData(SensorGroup & us_group)
{
  // create message
  static sensor_msgs::msg::Range message;

  // erhead
  rclcpp::Time t = now();
  createHeader(message.header, t);

  for (unsigned int i = 0; i < us_group.numberOfChannels(); i++) {
    message.range = us_group[i];
    if (i == range_publishers.size()) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Try to publish channel %d of US group but publisher is missing",
        i);
      break;
    }
    this->range_publishers[i]->publish(message);
  }
}

bool UcNode::getChannels()
{
  RCLCPP_INFO(this->get_logger(), "Wait 2s for Channels..");
  auto req = std::make_shared<ReqDAQCH>();
  req->channels();
  com->sendCommand(req, std::chrono::seconds(2));

  if (req->success()) {
    channelDescriptions = req->getChannelDescriptions();
    for (const ChannelDescription & d : channelDescriptions) {
      RCLCPP_DEBUG(
        this->get_logger(),
        "Description for: %s : %s : %d",
        d.getName().c_str(),
        d.getDescription().c_str(),
        d.getType());
    }
    return true;
  } else {
    RCLCPP_ERROR(
      this->get_logger(),
      "Within time limitation got no answer for channel descriptions");
    return false;
  }
}

void UcNode::sendImuData(SensorGroup & imu_group)
{
  // create message
  static sensor_msgs::msg::Imu imu_message;

  geometry_msgs::msg::Vector3 & linear_acceleration(imu_message.linear_acceleration);
  geometry_msgs::msg::Vector3 & angular_velocity(imu_message.angular_velocity);

  // header
  rclcpp::Time t = now();
  createHeader(imu_message.header, t);

  // linear acceleration
  linear_acceleration.x = imu_group[0];
  linear_acceleration.y = imu_group[1];
  linear_acceleration.z = imu_group[2];

  // angular velocity
  angular_velocity.x = imu_group[3];
  angular_velocity.y = imu_group[4];
  angular_velocity.z = imu_group[5];

  publisher_imu->publish(imu_message);
}

void UcNode::sendMagData(SensorGroup & mag_group)
{
  // create message
  static sensor_msgs::msg::MagneticField mag_message;

  geometry_msgs::msg::Vector3 & magnetic_field(mag_message.magnetic_field);

  // header
  rclcpp::Time t = now();
  createHeader(mag_message.header, t);

  magnetic_field.x = mag_group[0];
  magnetic_field.y = mag_group[1];
  magnetic_field.z = mag_group[2];

  publisher_mag->publish(mag_message);
}

void UcNode::sendHallDtData(SensorGroup & hall_group)
{
  // create message
  static std_msgs::msg::Float32 dt_message;
  static std_msgs::msg::UInt8 cnt_message;

  dt_message.data = hall_group[1];
  cnt_message.data = static_cast<uint8_t>(hall_group(0));

  publisher_dt->publish(dt_message);
  publisher_cnt->publish(cnt_message);
}

void UcNode::sendHallDt8Data(SensorGroup & hall_group)
{
  // create message
  static std_msgs::msg::Float32 dt_message;
  static std_msgs::msg::Int16 speed_message;
  static float speed;

  dt_message.data = hall_group[0];
  speed = 3.3f * 2 * 3.14f / hall_group[0];
  speed_message.data = static_cast<int16_t>(speed);
  publisher_dt8->publish(dt_message);
  publisher_speed->publish(speed_message);
}

void UcNode::sendEncStepData(SensorGroup & enc_group)
{
  // create message
  static psaf_ucbridge_msgs::msg::EncStep step_message;
  // header
  rclcpp::Time t = now();
  createHeader(step_message.header, t);
  step_message.steps = static_cast<uint32_t>(enc_group[0]);
  step_message.tics = static_cast<uint32_t>(enc_group[1]);
  publisher_enc_steps->publish(step_message);
}

void UcNode::sendEncCmData(SensorGroup & enc_group)
{
  // create message
  static psaf_ucbridge_msgs::msg::EncCm cm_message;
  static std_msgs::msg::Int16 speed_message;
// header
  rclcpp::Time t = now();
  createHeader(cm_message.header, t);
  cm_message.cms = static_cast<uint16_t>(enc_group[0]);
  speed_message.data = static_cast<int16_t>(enc_group[0]);
  publisher_enc_cms->publish(cm_message);
  publisher_speed->publish(speed_message);
}

void UcNode::sendPbData(SensorGroup & pb_group)
{
  static psaf_ucbridge_msgs::msg::Pbs pb_message;
  static bool flag = false;
  static std_msgs::msg::Int8 button_msg;
  if (!flag) {
    flag = true;
    button_msg.data = -1;
  } else {
    if (static_cast<uint8_t>(pb_group[0]) != pb_message.pba) {button_msg.data = 1;}
    if (static_cast<uint8_t>(pb_group[1]) != pb_message.pbb) {button_msg.data = 2;}
    if (static_cast<uint8_t>(pb_group[2]) != pb_message.pbc) {button_msg.data = 3;}
  }
  pb_message.pba = static_cast<uint8_t>(pb_group[0]);
  pb_message.pbb = static_cast<uint8_t>(pb_group[1]);
  pb_message.pbc = static_cast<uint8_t>(pb_group[2]);
  publisher_pb->publish(pb_message);
  publisher_button->publish(button_msg);
}

void UcNode::sendRawDaq(SensorGroup & raw_grp)
{
  static std_msgs::msg::String raw_msg;
  std::stringstream s;
  s << "Grp Num:" << raw_grp.getSensorGroupNo();
  for (auto i = 0; i < raw_grp.numberOfChannels(); i++) {
    s << "| " << raw_grp[i] << " ";
  }
  s << "\n";
  raw_msg.data = s.str();
  publisher_unk->publish(raw_msg);
}

void UcNode::sendVbatData(SensorGroup & vbat_group)
{
  static psaf_ucbridge_msgs::msg::Vbat vbat_message;
  vbat_message.vsbat = static_cast<uint16_t>(vbat_group[0]);
  vbat_message.vdbat = static_cast<uint16_t>(vbat_group[1]);
  publisher_vbat->publish(vbat_message);
}

void UcNode::sendDisplay(DisplayTxt & dis)
{
  static psaf_ucbridge_msgs::msg::Display dis_message;
  dis_message.message = dis.get_data();
  publisher_dis->publish(dis_message);
}

// void UcNode::raw_communication_callback(
// const psaf_ucbridge_msgs::msg::RawMessage::SharedPtr & msg)
// {
//  RCLCPP_DEBUG(get_logger(), "Communicate RAW");
//  auto cmd = std::make_shared<ComRaw>();
//  cmd->raw(msg->message);
//  cmd->requireAnswer(msg->requires_answer);
//  cmd->answer(msg->answer);
//  com->sendCommand(cmd, std::chrono::microseconds(100000));
//  if (!cmd->success()) {
//    RCLCPP_WARN(this->get_logger(), "Could not communicate raw: %s", cmd->errorMessage().c_str());
//  }
// }

void UcNode::send_rc_callback()
{
  auto message = std::make_shared<ReqRc>();
  com->sendCommand(message, std::chrono::microseconds(100000));
  if (!message->success()) {
    RCLCPP_WARN(this->get_logger(), "Could not publish rc! %s", message->errorMessage().c_str());
    return;
  }
  static std_msgs::msg::UInt8 pub_msg;
  pub_msg.data = static_cast<uint8_t>(message->get_mode());
  publisher_rc->publish(pub_msg);
}

void UcNode::send_steering_callback()
{
  auto req = std::make_shared<ReqSTEER>();
  com->sendCommand(req, std::chrono::microseconds(100000));
  if (!req->success()) {
    RCLCPP_WARN(this->get_logger(), "Could not publish steer! %s", req->errorMessage().c_str());
    return;
  }
  static std_msgs::msg::Int16 pub_msg;
  pub_msg.data = static_cast<int16_t>(req->angle());
  publisher_steering->publish(pub_msg);
}

void UcNode::unknown_channel_callback(const psaf_ucbridge_msgs::msg::UnkDaq::SharedPtr & msg)
{
  static std::map<uint8_t, DAQChannelElement::ChannelEnum> m {{0, DAQChannelElement::UNK0},
    {1, DAQChannelElement::UNK1},
    {2, DAQChannelElement::UNK2},
    {3, DAQChannelElement::UNK3},
    {4, DAQChannelElement::UNK4},
    {5, DAQChannelElement::UNK5}};
  // delete group in ucbridge
  if (msg->delete_grp) {
    bool status = receiver->unregisterSensorGroup(msg->grp_no);
    if (status) {
      RCLCPP_INFO(this->get_logger(), "Delete group %d successfully!", msg->grp_no);
    } else {
      RCLCPP_WARN(
        this->get_logger(), "Could not delete %d! There is no such a group.",
        msg->grp_no);
    }
  }
  {
    auto cmd = std::make_shared<ComDAQ>();
    cmd->stop();
    com->sendCommand(cmd, std::chrono::microseconds(100000));
    if (!cmd->success()) {
      RCLCPP_WARN(this->get_logger(), "Could not stop daq: %s", cmd->errorMessage().c_str());
      return;
    } else {
      RCLCPP_INFO(this->get_logger(), "DAQ stopped because of adding new group.");
    }
  }
  auto cmd = std::make_shared<ComDAQ>();
  // delete group in ucboard
  if (msg->delete_grp) {
    cmd->group(msg->grp_no);
    cmd->remove();
  } else {
    if (msg->channel_num > 3 || msg->channel_num < 1) {
      RCLCPP_WARN(this->get_logger(), "Unknown channel num should in range [1, 3]");
      startDAQ();
      return;
    }
    if (msg->grp_no > 20 || msg->grp_no < 1) {
      RCLCPP_WARN(this->get_logger(), "grp_no should within [1, 20]! got: %d", msg->grp_no);
      startDAQ();
      return;
    }
    cmd->group(msg->grp_no);
    if (msg->sign) {
      for (auto i = 0; i < msg->channel_num; i++) {
        cmd->channel(m[i + 3]);
      }
    } else {
      for (auto i = 0; i < msg->channel_num; i++) {
        cmd->channel(m[i]);
      }
    }
    cmd->otherOpt(msg->options);
  }

  com->sendCommand(cmd, std::chrono::seconds(1));

  if (!cmd->success()) {
    RCLCPP_WARN(
      this->get_logger(),
      "Could not register Unknown Sensor Group: %s", cmd->errorMessage().c_str());
  } else {
    // create group
    SensorGroup::UniquePtr group = std::make_unique<SensorGroup>(*cmd);
    group->registerCallback([this](SensorGroup & grp) {sendRawDaq(grp);});
    receiver->registerSensorGroup(std::move(group));
  }
  startDAQ();
}

void UcNode::initializeLogger()
{
  Configuration & conf = Configuration::instance();
  RCLCPP_INFO(get_logger(), "LogLevel: %d", conf.logLevel());
  // rcutils_logging_set_default_logger_level(conf.logLevel());
  auto result = rcutils_logging_set_logger_level(get_logger().get_name(), conf.logLevel());
  if (result != RCUTILS_RET_OK) {
    RCLCPP_WARN(
      get_logger(),
      "Unable to set log level %d. Got return code %d",
      conf.logLevel(),
      result);
  }
  LoggerFactory & f = LoggerFactory::instance();
  f.registerLoggerBackend(this);
}

bool UcNode::setDeadMan()
{
  Configuration & conf = Configuration::instance();
  auto cmd = std::make_shared<ComDRV>();
  cmd->dms(conf.deadMan());
  com->sendCommand(cmd, std::chrono::microseconds(100000));
  if (!cmd->success()) {
    RCLCPP_WARN(this->get_logger(), "Could not set dead man as %d!", conf.deadMan());
  } else {
    RCLCPP_INFO(this->get_logger(), "Set dead man as %d.", conf.deadMan());
  }
  return cmd->success();
}

void logFatal(const rclcpp::Logger & logger, std::string & message)
{
  RCLCPP_FATAL(logger, message.c_str());
}

LoggerBackend::LoggerCallbackType UcNode::getFatalLogger(std::string & name)
{
  if (rcutils_logging_logger_is_enabled_for(
      get_logger().get_name(),
      RCUTILS_LOG_SEVERITY_FATAL))
  {
    auto logger = getConfiguredLogger(name);
    return [logger](std::string & message) {
             logFatal(logger, message);
           };
  } else {
    return nullptr;
  }
}

void logError(const rclcpp::Logger & logger, std::string & message)
{
  RCLCPP_ERROR(logger, message.c_str());
}

LoggerBackend::LoggerCallbackType UcNode::getErrorLogger(std::string & name)
{
  if (rcutils_logging_logger_is_enabled_for(
      get_logger().get_name(),
      RCUTILS_LOG_SEVERITY_ERROR))
  {
    auto logger = getConfiguredLogger(name);
    return [logger](std::string & message) {
             logError(logger, message);
           };
  } else {
    return nullptr;
  }
}

void logWarn(const rclcpp::Logger & logger, std::string & message)
{
  RCLCPP_WARN(logger, message.c_str());
}

LoggerBackend::LoggerCallbackType UcNode::getWarnLogger(std::string & name)
{
  if (rcutils_logging_logger_is_enabled_for(
      get_logger().get_name(),
      RCUTILS_LOG_SEVERITY_WARN))
  {
    auto logger = getConfiguredLogger(name);
    return [logger](std::string & message) {
             logWarn(logger, message);
           };
  } else {
    return nullptr;
  }
}

void logInfo(const rclcpp::Logger & logger, std::string & message)
{
  RCLCPP_INFO(logger, message.c_str());
}

LoggerBackend::LoggerCallbackType UcNode::getInfoLogger(std::string & name)
{
  if (rcutils_logging_logger_is_enabled_for(
      get_logger().get_name(),
      RCUTILS_LOG_SEVERITY_INFO))
  {
    auto logger = getConfiguredLogger(name);
    RCLCPP_INFO(get_logger(), "Get logger for %s", name.c_str());
    return [logger](std::string & message) {
             logInfo(logger, message);
           };
  } else {
    return nullptr;
  }
}

void logDebug(const rclcpp::Logger & logger, std::string & message)
{
  RCLCPP_DEBUG(logger, message.c_str());
}

LoggerBackend::LoggerCallbackType UcNode::getDebugLogger(std::string & name)
{
  if (rcutils_logging_logger_is_enabled_for(
      get_logger().get_name(),
      RCUTILS_LOG_SEVERITY_DEBUG))
  {
    auto logger = getConfiguredLogger(name);
    return [logger](std::string & message) {
             logDebug(logger, message);
           };
  } else {
    return nullptr;
  }
}
rclcpp::Logger UcNode::getConfiguredLogger(const std::string & name)
{
  rclcpp::Logger logger = this->get_logger().get_child(name);
  return logger;
}
