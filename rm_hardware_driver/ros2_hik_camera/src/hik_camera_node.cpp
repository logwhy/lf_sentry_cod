#include "MvCameraControl.h"
// ROS
#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <atomic>

namespace hik_camera
{
class HikCameraNode : public rclcpp::Node
{
public:
  explicit HikCameraNode(const rclcpp::NodeOptions & options) : Node("hik_camera", options)
  {
    RCLCPP_INFO(this->get_logger(), "Starting HikCameraNode!");

    // Declare reconnect parameters
    reconnect_max_attempts_ = this->declare_parameter("reconnect_max_attempts", 30);
    reconnect_interval_ms_ = this->declare_parameter("reconnect_interval_ms", 500);

    if (!initCamera()) {
      RCLCPP_FATAL(this->get_logger(), "Initial camera connection failed!");
      return;
    }

    bool use_sensor_data_qos = this->declare_parameter("use_sensor_data_qos", true);
    auto qos = use_sensor_data_qos ? rmw_qos_profile_sensor_data : rmw_qos_profile_default;
    camera_pub_ = image_transport::create_camera_publisher(this, "image_raw", qos);

    declareParameters();

    MV_CC_StartGrabbing(camera_handle_);

    // Load camera info
    camera_name_ = this->declare_parameter("camera_name", "narrow_stereo");
    camera_info_manager_ =
      std::make_unique<camera_info_manager::CameraInfoManager>(this, camera_name_);
    auto camera_info_url =
      this->declare_parameter("camera_info_url", "package://hik_camera/config/camera_info.yaml");
    if (camera_info_manager_->validateURL(camera_info_url)) {
      camera_info_manager_->loadCameraInfo(camera_info_url);
      camera_info_msg_ = camera_info_manager_->getCameraInfo();
    } else {
      RCLCPP_WARN(this->get_logger(), "Invalid camera info URL: %s", camera_info_url.c_str());
    }

    camera_connected_ = true;

    params_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&HikCameraNode::parametersCallback, this, std::placeholders::_1));

    capture_thread_ = std::thread{[this]() -> void {
      MV_FRAME_OUT out_frame;

      RCLCPP_INFO(this->get_logger(), "Publishing image!");

      image_msg_.header.frame_id = "camera_optical_frame";
      image_msg_.encoding = "rgb8";

      while (rclcpp::ok()) {
        nRet = MV_CC_GetImageBuffer(camera_handle_, &out_frame, 1000);
        if (MV_OK == nRet) {
          convert_param_.pDstBuffer = image_msg_.data.data();
          convert_param_.nDstBufferSize = image_msg_.data.size();
          convert_param_.pSrcData = out_frame.pBufAddr;
          convert_param_.nSrcDataLen = out_frame.stFrameInfo.nFrameLen;
          convert_param_.enSrcPixelType = out_frame.stFrameInfo.enPixelType;

          MV_CC_ConvertPixelType(camera_handle_, &convert_param_);

          image_msg_.header.stamp = this->now();
          image_msg_.height = out_frame.stFrameInfo.nHeight;
          image_msg_.width = out_frame.stFrameInfo.nWidth;
          image_msg_.step = out_frame.stFrameInfo.nWidth * 3;
          image_msg_.data.resize(image_msg_.width * image_msg_.height * 3);

          camera_info_msg_.header = image_msg_.header;
          camera_pub_.publish(image_msg_, camera_info_msg_);

          MV_CC_FreeImageBuffer(camera_handle_, &out_frame);
          fail_count_ = 0;
        } else {
          RCLCPP_WARN(this->get_logger(), "Get buffer failed! nRet: [%x]", nRet);
          fail_count_++;

          bool need_full_reconnect = false;

          // Check if USB-layer error (0x80000300 - 0x800003FF)
          if ((nRet & 0xFFFFFF00) == 0x80000300) {
            need_full_reconnect = true;
          }

          // Check if device is still connected
          if (!MV_CC_IsDeviceConnected(camera_handle_)) {
            need_full_reconnect = true;
          }

          // Too many consecutive failures
          if (fail_count_ > 5) {
            need_full_reconnect = true;
          }

          if (need_full_reconnect) {
            RCLCPP_ERROR(
              this->get_logger(), "Camera connection lost (fail_count=%d, nRet=0x%x), "
              "attempting reconnect...", fail_count_, nRet);

            if (reconnectCamera()) {
              RCLCPP_INFO(this->get_logger(), "Camera reconnected successfully!");
              fail_count_ = 0;
            } else {
              RCLCPP_FATAL(
                this->get_logger(), "Camera reconnect failed after %d attempts, "
                "capture thread exiting.", reconnect_max_attempts_);
              break;
            }
          } else {
            // Lightweight recovery: restart grabbing
            MV_CC_StopGrabbing(camera_handle_);
            MV_CC_StartGrabbing(camera_handle_);
          }
        }
      }
    }};
  }

  ~HikCameraNode() override
  {
    if (capture_thread_.joinable()) {
      capture_thread_.join();
    }
    cleanupCamera();
    RCLCPP_INFO(this->get_logger(), "HikCameraNode destroyed!");
  }

private:
  // blocking=true: 启动时死等相机出现; blocking=false: 重连时只试一次立即返回
  bool initCamera(bool blocking = true)
  {
    MV_CC_DEVICE_INFO_LIST device_list;
    nRet = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
    RCLCPP_INFO(this->get_logger(), "Found camera count = %d", device_list.nDeviceNum);

    if (blocking) {
      while (device_list.nDeviceNum == 0 && rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "No camera found!");
        RCLCPP_INFO(this->get_logger(), "Enum state: [%x]", nRet);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        nRet = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
      }
      if (!rclcpp::ok()) {
        return false;
      }
    } else if (device_list.nDeviceNum == 0) {
      return false;
    }

    nRet = MV_CC_CreateHandle(&camera_handle_, device_list.pDeviceInfo[0]);
    if (MV_OK != nRet) {
      RCLCPP_ERROR(this->get_logger(), "MV_CC_CreateHandle failed: [%x]", nRet);
      return false;
    }

    nRet = MV_CC_OpenDevice(camera_handle_);
    if (MV_OK != nRet) {
      RCLCPP_ERROR(this->get_logger(), "MV_CC_OpenDevice failed: [%x]", nRet);
      MV_CC_DestroyHandle(&camera_handle_);
      camera_handle_ = nullptr;
      return false;
    }

    // 像素格式
    nRet = MV_CC_SetPixelFormat(camera_handle_, 0x01080009);
    if (nRet != MV_OK)
      RCLCPP_ERROR(this->get_logger(), "MV_CC_SetPixelFormat failed:[%x]", nRet);
    // Bayer像素质量
    nRet = MV_CC_SetBayerCvtQuality(camera_handle_, 2);
    if (nRet != MV_OK)
      RCLCPP_ERROR(this->get_logger(), "SetBayerCvtQuality failed:[%x]", nRet);

    // Get camera information
    MV_CC_GetImageInfo(camera_handle_, &img_info_);
    image_msg_.data.reserve(img_info_.nHeightMax * img_info_.nWidthMax * 3);

    // Init convert param
    convert_param_.nWidth = img_info_.nWidthValue;
    convert_param_.nHeight = img_info_.nHeightValue;
    convert_param_.enDstPixelType = PixelType_Gvsp_RGB8_Packed;

    return true;
  }

  void cleanupCamera()
  {
    camera_connected_ = false;
    if (camera_handle_) {
      MV_CC_StopGrabbing(camera_handle_);
      MV_CC_CloseDevice(camera_handle_);
      MV_CC_DestroyHandle(&camera_handle_);
      camera_handle_ = nullptr;
    }
  }

  void applyParameters()
  {
    double exposure_time = this->get_parameter("exposure_time").as_int();
    double gain = this->get_parameter("gain").as_double();
    MV_CC_SetFloatValue(camera_handle_, "ExposureTime", exposure_time);
    MV_CC_SetFloatValue(camera_handle_, "Gain", gain);
    RCLCPP_INFO(
      this->get_logger(), "Reapplied parameters: exposure_time=%.0f, gain=%.1f",
      exposure_time, gain);
  }

  bool reconnectCamera()
  {
    camera_connected_ = false;
    cleanupCamera();

    for (int attempt = 1; attempt <= reconnect_max_attempts_; attempt++) {
      RCLCPP_INFO(
        this->get_logger(), "Reconnect attempt %d/%d...", attempt, reconnect_max_attempts_);

      std::this_thread::sleep_for(std::chrono::milliseconds(reconnect_interval_ms_));

      if (!rclcpp::ok()) {
        return false;
      }

      if (initCamera(false)) {
        applyParameters();
        nRet = MV_CC_StartGrabbing(camera_handle_);
        if (MV_OK == nRet) {
          camera_connected_ = true;
          return true;
        }
        RCLCPP_WARN(this->get_logger(), "StartGrabbing failed: [%x]", nRet);
        cleanupCamera();
      }
    }

    return false;
  }

  void declareParameters()
  {
    rcl_interfaces::msg::ParameterDescriptor param_desc;
    MVCC_FLOATVALUE f_value;
    param_desc.integer_range.resize(1);
    param_desc.integer_range[0].step = 1;
    // Exposure time
    param_desc.description = "Exposure time in microseconds";
    MV_CC_GetFloatValue(camera_handle_, "ExposureTime", &f_value);
    param_desc.integer_range[0].from_value = f_value.fMin;
    param_desc.integer_range[0].to_value = f_value.fMax;
    double exposure_time = this->declare_parameter("exposure_time", 12000, param_desc);
    MV_CC_SetFloatValue(camera_handle_, "ExposureTime", exposure_time);
    RCLCPP_INFO(this->get_logger(), "Exposure time: %f", exposure_time);

    // Gain
    param_desc.description = "Gain";
    MV_CC_GetFloatValue(camera_handle_, "Gain", &f_value);
    param_desc.integer_range[0].from_value = f_value.fMin;
    param_desc.integer_range[0].to_value = f_value.fMax;
    double gain = this->declare_parameter("gain", f_value.fCurValue, param_desc);
    MV_CC_SetFloatValue(camera_handle_, "Gain", gain);
    RCLCPP_INFO(this->get_logger(), "Gain: %f", gain);
  }

  rcl_interfaces::msg::SetParametersResult parametersCallback(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    if (!camera_connected_) {
      result.successful = false;
      result.reason = "Camera is not connected, parameter change rejected";
      return result;
    }

    for (const auto & param : parameters) {
      if (param.get_name() == "exposure_time") {
        int status = MV_CC_SetFloatValue(camera_handle_, "ExposureTime", param.as_int());
        if (MV_OK != status) {
          result.successful = false;
          result.reason = "Failed to set exposure time, status = " + std::to_string(status);
        }
      } else if (param.get_name() == "gain") {
        int status = MV_CC_SetFloatValue(camera_handle_, "Gain", param.as_double());
        if (MV_OK != status) {
          result.successful = false;
          result.reason = "Failed to set gain, status = " + std::to_string(status);
        }
      } else {
        result.successful = false;
        result.reason = "Unknown parameter: " + param.get_name();
      }
    }
    return result;
  }

  sensor_msgs::msg::Image image_msg_;

  image_transport::CameraPublisher camera_pub_;

  int nRet = MV_OK;
  void * camera_handle_ = nullptr;
  MV_IMAGE_BASIC_INFO img_info_;

  MV_CC_PIXEL_CONVERT_PARAM convert_param_;

  std::string camera_name_;
  std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
  sensor_msgs::msg::CameraInfo camera_info_msg_;

  int fail_count_ = 0;
  std::atomic<bool> camera_connected_{false};
  std::thread capture_thread_;

  int reconnect_max_attempts_;
  int reconnect_interval_ms_;

  OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;
};
}  // namespace hik_camera

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(hik_camera::HikCameraNode)
