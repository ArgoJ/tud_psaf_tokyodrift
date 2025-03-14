#ifndef PSAF_UCBRIDGE__SERVICE_FUNCTIONS_HPP_
#define PSAF_UCBRIDGE__SERVICE_FUNCTIONS_HPP_

#include <string>
#include <memory>
#include "psaf_ucbridge/communication/request.hpp"
#include "psaf_ucbridge/communication/raw_message.hpp"
#include "rclcpp/rclcpp.hpp"
#include "psaf_ucbridge/communication/communication_interface.hpp"
#include "psaf_ucbridge/uc_node.hpp"
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

namespace ServiceFunctions
{

void reqVer(
  std::shared_ptr<psaf_ucbridge_msgs::srv::ReqVer::Request> request,
  std::shared_ptr<psaf_ucbridge_msgs::srv::ReqVer::Response> response,
  const ICommunication::SharedPtr & com,
  const rclcpp::Logger & logger);

void reqId(
  std::shared_ptr<psaf_ucbridge_msgs::srv::ReqId::Request> request,
  std::shared_ptr<psaf_ucbridge_msgs::srv::ReqId::Response> response,
  const ICommunication::SharedPtr & com,
  const rclcpp::Logger & logger);

void reqSid(
  std::shared_ptr<psaf_ucbridge_msgs::srv::ReqSid::Request> request,
  std::shared_ptr<psaf_ucbridge_msgs::srv::ReqSid::Response> response,
  const ICommunication::SharedPtr & com,
  const rclcpp::Logger & logger);

void reqSteer(
  std::shared_ptr<psaf_ucbridge_msgs::srv::ReqSteer::Request> request,
  std::shared_ptr<psaf_ucbridge_msgs::srv::ReqSteer::Response> response,
  const ICommunication::SharedPtr & com,
  const rclcpp::Logger & logger);

void reqDrv(
  std::shared_ptr<psaf_ucbridge_msgs::srv::ReqDrv::Request> request,
  std::shared_ptr<psaf_ucbridge_msgs::srv::ReqDrv::Response> response,
  const ICommunication::SharedPtr & com,
  const rclcpp::Logger & logger);

void reqDms(
  std::shared_ptr<psaf_ucbridge_msgs::srv::ReqDms::Request> request,
  std::shared_ptr<psaf_ucbridge_msgs::srv::ReqDms::Response> response,
  const ICommunication::SharedPtr & com,
  const rclcpp::Logger & logger);

void reqChs(
  std::shared_ptr<psaf_ucbridge_msgs::srv::ReqChs::Request> request,
  std::shared_ptr<psaf_ucbridge_msgs::srv::ReqChs::Response> response,
  const ICommunication::SharedPtr & com,
  const rclcpp::Logger & logger);

void reqGet(
  std::shared_ptr<psaf_ucbridge_msgs::srv::ReqGet::Request> request,
  std::shared_ptr<psaf_ucbridge_msgs::srv::ReqGet::Response> response,
  const ICommunication::SharedPtr & com,
  const rclcpp::Logger & logger);

void reqGrp(
  std::shared_ptr<psaf_ucbridge_msgs::srv::ReqGrp::Request> request,
  std::shared_ptr<psaf_ucbridge_msgs::srv::ReqGrp::Response> response,
  const ICommunication::SharedPtr & com,
  const rclcpp::Logger & logger);

void reqVout(
  std::shared_ptr<psaf_ucbridge_msgs::srv::ReqVout::Request> request,
  std::shared_ptr<psaf_ucbridge_msgs::srv::ReqVout::Response> response,
  const ICommunication::SharedPtr & com,
  const rclcpp::Logger & logger);

void reqUs(
  std::shared_ptr<psaf_ucbridge_msgs::srv::ReqUs::Request> request,
  std::shared_ptr<psaf_ucbridge_msgs::srv::ReqUs::Response> response,
  const ICommunication::SharedPtr & com,
  const rclcpp::Logger & logger);

void reqImu(
  std::shared_ptr<psaf_ucbridge_msgs::srv::ReqImu::Request> request,
  std::shared_ptr<psaf_ucbridge_msgs::srv::ReqImu::Response> response,
  const ICommunication::SharedPtr & com,
  const rclcpp::Logger & logger);

void reqMag(
  std::shared_ptr<psaf_ucbridge_msgs::srv::ReqMag::Request> request,
  std::shared_ptr<psaf_ucbridge_msgs::srv::ReqMag::Response> response,
  const ICommunication::SharedPtr & com,
  const rclcpp::Logger & logger);

void reqEnc(
  std::shared_ptr<psaf_ucbridge_msgs::srv::ReqEnc::Request> request,
  std::shared_ptr<psaf_ucbridge_msgs::srv::ReqEnc::Response> response,
  const ICommunication::SharedPtr & com,
  const rclcpp::Logger & logger);

void reqRaw(
  std::shared_ptr<psaf_ucbridge_msgs::srv::ReqRaw::Request> request,
  std::shared_ptr<psaf_ucbridge_msgs::srv::ReqRaw::Response> response,
  const ICommunication::SharedPtr & com,
  const rclcpp::Logger & logger);
}  // namespace ServiceFunctions

#endif  // PSAF_UCBRIDGE__SERVICE_FUNCTIONS_HPP_
