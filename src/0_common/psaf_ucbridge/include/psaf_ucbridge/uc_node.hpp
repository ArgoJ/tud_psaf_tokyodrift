#ifndef PSAF_UCBRIDGE_UC_BRIDGE_H
#define PSAF_UCBRIDGE_UC_BRIDGE_H

#include <memory>
#include <vector>
#include <string>
#include <unordered_map>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include "psaf_ucbridge_msgs/msg/led.hpp"
#include "psaf_ucbridge_msgs/msg/raw_message.hpp"
#include "psaf_ucbridge_msgs/msg/unk_daq.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "psaf_ucbridge/communication/command.hpp"
#include "psaf_ucbridge/communication/request.hpp"
#include "psaf_ucbridge/communication/communication.hpp"
#include "psaf_ucbridge/communication/message_queue.hpp"
#include "psaf_ucbridge/communication/reading_thread.hpp"
#include "psaf_ucbridge/communication/data_receiver.hpp"
#include "psaf_ucbridge/logging/logger_backend.hpp"
#include "psaf_ucbridge/sensor_groups/channel_description.hpp"
#include "psaf_ucbridge/sensor_groups/display_txt.hpp"
#include "psaf_ucbridge/service_functions/service_functions.hpp"
#include "psaf_ucbridge_msgs/msg/enc_cm.hpp"
#include "psaf_ucbridge_msgs/msg/enc_step.hpp"
#include "psaf_ucbridge_msgs/msg/vbat.hpp"
#include "psaf_ucbridge_msgs/msg/pbs.hpp"
#include "psaf_ucbridge_msgs/msg/display.hpp"
#include "psaf_ucbridge_msgs/srv/req_ver.hpp"
#include "psaf_ucbridge_msgs/srv/req_id.hpp"
#include "psaf_ucbridge_msgs/srv/req_sid.hpp"
#include "psaf_ucbridge_msgs/srv/req_steer.hpp"
#include "psaf_ucbridge_msgs/srv/req_drv.hpp"
#include "psaf_ucbridge_msgs/srv/req_dms.hpp"
#include "psaf_ucbridge_msgs/srv/req_chs.hpp"
#include "psaf_ucbridge_msgs/srv/req_get.hpp"
#include "psaf_ucbridge_msgs/srv/req_grp.hpp"
#include "psaf_ucbridge_msgs/srv/req_vout.hpp"
#include "psaf_ucbridge_msgs/srv/req_us.hpp"
#include "psaf_ucbridge_msgs/srv/req_imu.hpp"
#include "psaf_ucbridge_msgs/srv/req_mag.hpp"
#include "psaf_ucbridge_msgs/srv/req_enc.hpp"
#include "psaf_ucbridge_msgs/srv/req_raw.hpp"

class UcNode : public rclcpp::Node, public LoggerBackend
{
public:
  ~UcNode() override;
  UcNode();

  LoggerCallbackType getFatalLogger(std::string & name) final;
  LoggerCallbackType getErrorLogger(std::string & name) final;
  LoggerCallbackType getWarnLogger(std::string & name) final;
  LoggerCallbackType getInfoLogger(std::string & name) final;
  LoggerCallbackType getDebugLogger(std::string & name) final;

private:
  rclcpp::QoS QOS;

  std::vector<ChannelDescription> channelDescriptions;

  std::unique_ptr<ReadingThread> reader;

  BoardCommunication::SharedPtr boardCommunication;

  std::shared_ptr<Communication> com;

  std::unique_ptr<DataReceiver> receiver;

  void initializeLogger();

  void set_motor_level_forwards_callback(const std_msgs::msg::Int16::SharedPtr & msg);

  void set_motor_level_backwards_callback(const std_msgs::msg::Int16::SharedPtr & msg);

  void set_steer_angle_callback(const std_msgs::msg::Int16::SharedPtr & msg);

  void set_led_callback(const std_msgs::msg::Int8::SharedPtr & msg);

//  void raw_communication_callback(const psaf_ucbridge_msgs::msg::RawMessage::SharedPtr & msg);

  void unknown_channel_callback(const psaf_ucbridge_msgs::msg::UnkDaq::SharedPtr & msg);

  void send_rc_callback();

  void send_steering_callback();

  void sendRawDaq(SensorGroup & raw_grp);

  void sendImuData(SensorGroup & imu_group);

  void sendUSData(SensorGroup & us_group);

  void sendMagData(SensorGroup & mag_group);

  void sendHallDtData(SensorGroup & mag_group);

  void sendHallDt8Data(SensorGroup & mag_group);

  void sendEncStepData(SensorGroup & enc_group);

  void sendEncCmData(SensorGroup & enc_group);

  void sendPbData(SensorGroup & pb_group);

  void sendVbatData(SensorGroup & vbat_group);

  void sendDisplay(DisplayTxt & display);

  static void createHeader(std_msgs::msg::Header & header, rclcpp::Time & t);

  rclcpp::Logger getConfiguredLogger(const std::string & name);
  /**
 * The timer for the node
 */
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr timer1_;
  rclcpp::TimerBase::SharedPtr timer2_;

  static constexpr auto node_interval = std::chrono::seconds(1);

  void createImuGroup();

  void createUSGroup();

  void createMagGroup();

  void createDtGroup();

  void createDt8Group();

  void createEncStepGroup();

  void createEncCmGroup();

  void createPbGroup();

  void  createVbatGroup();

  void createDisplayTxt();

  void startDAQ();

  bool setDeadMan();

  bool resetUcBoard();

  bool getChannels();

  /**
   * Function used for service register
   * @tparam T Service ptr
   * @tparam T0 Service type
   * @tparam T1 request ptr
   * @tparam T2 response ptr
   * @param ptr Service ptr
   * @param f Service function ptr
   * @param name Service name
   */
  template<class T, class T0, class T1, class T2>
  void registerService(
    T & ptr, void (* f)(
      T1, T2, const ICommunication::SharedPtr &,
      const rclcpp::Logger &), const std::string & name)
  {
    std::function<void(T1, T2)> func;
    func = std::bind(f, std::placeholders::_1, std::placeholders::_2, com, this->get_logger());
    ptr = this->create_service<T0>(name, func);
  }

  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions;

  /**
   * The publisher for speed
   */
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr publisher_speed;

  /**
   * The publisher for steering
   */
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr publisher_steering;

  /**
   * The publisher for button
   */
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr publisher_button;

  /**
   * The publisher for rc mode
   */
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr publisher_rc;

  /**
   * The publisher for Unknown channel
   */
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_unk;

  /**
 * The publisher for IMU data.
 */
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_imu;

  /**
   * Publishers for the range from the ultrasonic sensors
   */
  std::vector<rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr> range_publishers;

  /**
   * The publisher for the magnetometer
   */
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr publisher_mag;

  /**
   * The publisher for hall_dt (1/8 of wheel turn) in s
   */
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_dt;

  /**
   * The publisher for hall_dt8 (1 wheel turn) in s
   */
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_dt8;

  /**
   * The publisher for hall_cnt (pulses%256)
   */
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr publisher_cnt;

  /**
   * The publisher for enc_steps
   */
  rclcpp::Publisher<psaf_ucbridge_msgs::msg::EncStep>::SharedPtr publisher_enc_steps;

  /**
   * The publisher for enc_cms
   */
  rclcpp::Publisher<psaf_ucbridge_msgs::msg::EncCm>::SharedPtr publisher_enc_cms;

  /**
   * The publisher for Battery Voltage
   */
  rclcpp::Publisher<psaf_ucbridge_msgs::msg::Vbat>::SharedPtr publisher_vbat;

  /**
   * The publisher for PB
   */
  rclcpp::Publisher<psaf_ucbridge_msgs::msg::Pbs>::SharedPtr publisher_pb;

  /**
   * The publisher for txt data
   */
  rclcpp::Publisher<psaf_ucbridge_msgs::msg::Display>::SharedPtr publisher_dis;

  /**
   * The services for raw communication
   */
  rclcpp::Service<psaf_ucbridge_msgs::srv::ReqRaw>::SharedPtr service_raw;

  /**
   * The services for req Version.
   */
  rclcpp::Service<psaf_ucbridge_msgs::srv::ReqVer>::SharedPtr service_ver;

  /**
   * The service for req ID.
   */
  rclcpp::Service<psaf_ucbridge_msgs::srv::ReqId>::SharedPtr service_id;

  /**
   * The service for req SID
   */
  rclcpp::Service<psaf_ucbridge_msgs::srv::ReqSid>::SharedPtr service_sid;

  /**
   * The service for req Steer
   */
  rclcpp::Service<psaf_ucbridge_msgs::srv::ReqSteer>::SharedPtr service_steer;

  /**
   * The service for req Drv
   */
  rclcpp::Service<psaf_ucbridge_msgs::srv::ReqDrv>::SharedPtr service_drv;
  /**
   * The service for req Dms
   */
  rclcpp::Service<psaf_ucbridge_msgs::srv::ReqDms>::SharedPtr service_dms;
  /**
   * The service for req channel or channels
   */
  rclcpp::Service<psaf_ucbridge_msgs::srv::ReqChs>::SharedPtr service_chs;
  /**
   * The service for req sensor measure value.
   */
  rclcpp::Service<psaf_ucbridge_msgs::srv::ReqGet>::SharedPtr service_get;
  /**
   * The service for req group(s) information.
   */
  rclcpp::Service<psaf_ucbridge_msgs::srv::ReqGrp>::SharedPtr service_grp;
  /**
   * The service for req Vout.
   */
  rclcpp::Service<psaf_ucbridge_msgs::srv::ReqVout>::SharedPtr service_vout;
  /**
   * The service for US or US OPT
   */
  rclcpp::Service<psaf_ucbridge_msgs::srv::ReqUs>::SharedPtr service_us;
  /**
   * The service for IMU OPT
   */
  rclcpp::Service<psaf_ucbridge_msgs::srv::ReqImu>::SharedPtr service_imu;
  /**
   * The service for mag opt or asa.
   */
  rclcpp::Service<psaf_ucbridge_msgs::srv::ReqMag>::SharedPtr service_mag;
  /**
   * The service for enc
   */
  rclcpp::Service<psaf_ucbridge_msgs::srv::ReqEnc>::SharedPtr service_enc;
};

#endif  // PSAF_UCBRIDGE_UC_BRIDGE_H
