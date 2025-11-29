#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <opencv2/opencv.hpp>
#include "CameraApi.h"  // Huateng Vision SDK
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include <mutex>
#include <queue>
#include <thread>
#include <atomic>
#include "huateng_camera_node/srv/set_float32.hpp"

using SetFloatSrv = huateng_camera_node::srv::SetFloat32;

using namespace std::chrono_literals;


class HuatengCameraNode : public rclcpp::Node
{
public:
    HuatengCameraNode() : Node("huateng_camera_node"), running_(true)
    {
    }

    void init() {
    // ---------------------------------------------------------
    // 1. DECLARAÇÃO DE PARÂMETROS (ROS 2 Parameters)
    // ---------------------------------------------------------
    camera_id_ = declare_parameter<int>("camera_id", 0);
    fps_ = declare_parameter<int>("fps", 30);
    output_format_ = declare_parameter<std::string>("output_format", "bgr8");
    param_save_path_ = declare_parameter<std::string>("param_save_path", "/tmp/huateng_params.bin");
    
    // Parâmetros de Exposição
    exposure_time_ = declare_parameter<double>("exposure_time", 10000.0);
    auto_exposure_ = declare_parameter<bool>("exposure_auto", false);

    // Parâmetros de Ajuste de Imagem (ISP Hardware)
    // Valores padrão baseados na nossa discussão para destacar faixas
    this->declare_parameter("black_level", 0);   // Tente manter baixo (0-10)
    this->declare_parameter("contrast", 80);     // Alto para binarizar (0-100)
    this->declare_parameter("gamma", 140);       // Ajuste fino de tons médios (0-255, padrão ~100)
    this->declare_parameter("sharpness", 50);    // Nitidez para bordas (0-100)

    // ---------------------------------------------------------
    // 2. CONFIGURAÇÃO DE PUBLISHERS E SUBSCRIBERS
    // ---------------------------------------------------------
    image_transport::ImageTransport it(shared_from_this());
    image_pub_ = it.advertise("image_raw", 1);
    camera_info_man_ = std::make_shared<camera_info_manager::CameraInfoManager>(this);
    camera_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", 1);
    status_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("camera_status", 1);

    // Subscribers (Legado/Auxiliar)
    sub_exposure_ = create_subscription<std_msgs::msg::Float32>(
        "set_exposure", 10, std::bind(&HuatengCameraNode::onExposureTopic, this, std::placeholders::_1));
    sub_gain_ = create_subscription<std_msgs::msg::Float32>(
        "set_gain", 10, std::bind(&HuatengCameraNode::onGainTopic, this, std::placeholders::_1));
    sub_white_balance_ = create_subscription<std_msgs::msg::Bool>(
        "do_white_balance", 10, std::bind(&HuatengCameraNode::onWhiteBalanceTopic, this, std::placeholders::_1));

    // Services
    srv_set_exposure_ = create_service<SetFloatSrv>(
        "set_exposure_srv",
        [this](const std::shared_ptr<SetFloatSrv::Request> req, std::shared_ptr<SetFloatSrv::Response> res) {
            res->success = setExposure(req->data);
        });
    srv_set_gain_ = create_service<SetFloatSrv>(
        "set_gain_srv",
        [this](const std::shared_ptr<SetFloatSrv::Request> req, std::shared_ptr<SetFloatSrv::Response> res) {
            res->success = setGain(req->data);
        });

    // ---------------------------------------------------------
    // 3. INICIALIZAÇÃO DA SDK E CÂMERA
    // ---------------------------------------------------------
    if (CameraSdkInit(0) != CAMERA_STATUS_SUCCESS) {
        RCLCPP_ERROR(get_logger(), "Failed to initialize Huateng SDK");
        rclcpp::shutdown();
        return;
    }

    tSdkCameraDevInfo devs[10];
    int num = 10;
    if (CameraEnumerateDevice(devs, &num) != CAMERA_STATUS_SUCCESS || num <= 0) {
        RCLCPP_ERROR(get_logger(), "No cameras detected");
        rclcpp::shutdown();
        return;
    }
    if (CameraInit(&devs[camera_id_], -1, -1, &hCamera_) != CAMERA_STATUS_SUCCESS) {
        RCLCPP_ERROR(get_logger(), "Failed to open camera %d", camera_id_);
        rclcpp::shutdown();
        return;
    }

    CameraGetCapability(hCamera_, &capability_);
    CameraPlay(hCamera_);
    CameraSetIspOutFormat(hCamera_, CAMERA_MEDIA_TYPE_BGR8);

    // ---------------------------------------------------------
    // 4. CONFIGURAÇÃO DE MELHORIA DE IMAGEM (ISP)
    // ---------------------------------------------------------
    
    // ATENÇÃO: Ativa o modo LUT Dinâmico. Essencial para Gamma e Contraste funcionarem.
    // 1 geralmente mapeia para LUTMODE_PARAM_GEN na SDK.
    CameraSetLutMode(hCamera_, 1); 

    // Aplica os valores iniciais definidos nos parâmetros
    int bl = this->get_parameter("black_level").as_int();
    int ct = this->get_parameter("contrast").as_int();
    int gm = this->get_parameter("gamma").as_int();
    int sh = this->get_parameter("sharpness").as_int();

    CameraSetBlackLevel(hCamera_, bl);
    CameraSetContrast(hCamera_, ct);
    CameraSetGamma(hCamera_, gm);
    CameraSetSharpness(hCamera_, sh);

    RCLCPP_INFO(get_logger(), "ISP Configurado -> BL:%d | Contrast:%d | Gamma:%d | Sharp:%d", bl, ct, gm, sh);

    // *** CALLBACK DINÂMICO ***
    // Isso permite que você mude os valores acima em tempo real via terminal/rqt
    params_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&HuatengCameraNode::parametersCallback, this, std::placeholders::_1));


    // ---------------------------------------------------------
    // 5. EXPOSIÇÃO E AUTO-EXPOSIÇÃO
    // ---------------------------------------------------------
    if (auto_exposure_) {
        CameraSetAeState(hCamera_, TRUE);
        RCLCPP_INFO(get_logger(), "Auto Exposure: ON");
    } else {
        CameraSetAeState(hCamera_, FALSE);
        CameraSetExposureTime(hCamera_, exposure_time_);
        RCLCPP_INFO(get_logger(), "Auto Exposure: OFF | Time set to: %.2f µs", exposure_time_);
    }

    // Capability Logging
    int exp_min = capability_.sExposeDesc.uiExposeTimeMin;
    int exp_max = capability_.sExposeDesc.uiExposeTimeMax;
    int gain_min = (int)((capability_.sRgbGainRange.iRGainMin + capability_.sRgbGainRange.iGGainMin + capability_.sRgbGainRange.iBGainMin)/3);
    int gain_max = (int)((capability_.sRgbGainRange.iRGainMax + capability_.sRgbGainRange.iGGainMax + capability_.sRgbGainRange.iBGainMax)/3);

    RCLCPP_INFO(get_logger(), "Exposure range %d–%d µs | Gain range %d–%d",
                exp_min, exp_max, gain_min, gain_max);

    // Register frame callback
    CameraSetCallbackFunction(hCamera_, &HuatengCameraNode::GrabImageCallback, this, nullptr);

    // Start threads
    pub_thread_ = std::thread([this]() { publishLoop(); });
    status_timer_ = create_wall_timer(1s, std::bind(&HuatengCameraNode::publishStatus, this));

    RCLCPP_INFO(get_logger(), "Huateng camera initialized and streaming");
}

    ~HuatengCameraNode()
    {
        running_ = false;
        if (pub_thread_.joinable()) pub_thread_.join();
        if (hCamera_) {
            CameraSetCallbackFunction(hCamera_, nullptr, nullptr, nullptr);
            CameraUnInit(hCamera_);
        }
    }

private:
    // Handle para o callback de parâmetros
    rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;

    // A função que vai ser chamada quando você mudar algo no terminal/rqt
    rcl_interfaces::msg::SetParametersResult parametersCallback(
        const std::vector<rclcpp::Parameter> &parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "success";

        for (const auto &param : parameters)
        {
            if (param.get_name() == "black_level")
            {
                int val = param.as_int();
                CameraSetBlackLevel(hCamera_, val);
                RCLCPP_INFO(get_logger(), "Black Level atualizado: %d", val);
            }
            else if (param.get_name() == "gamma")
            {
                int val = param.as_int();
                CameraSetGamma(hCamera_, val);
                RCLCPP_INFO(get_logger(), "Gamma atualizado: %d", val);
            }
            else if (param.get_name() == "contrast")
            {
                int val = param.as_int();
                CameraSetContrast(hCamera_, val);
                RCLCPP_INFO(get_logger(), "Contrast atualizado: %d", val);
            }
        }
        return result;
    }
    // Frame callback from SDK
    static void GrabImageCallback(CameraHandle h, BYTE* buf, tSdkFrameHead* head, PVOID ctx)
    {
        reinterpret_cast<HuatengCameraNode*>(ctx)->onFrame(h, buf, head);
    }

    void onFrame(CameraHandle h, BYTE* pBuf, tSdkFrameHead* head)
    {
        std::lock_guard<std::mutex> lock(queue_mtx_);
        cv::Mat img(cv::Size(head->iWidth, head->iHeight), CV_8UC3);
        CameraImageProcess(h, pBuf, img.data, head);
        frame_queue_.push(std::move(img));
        CameraReleaseImageBuffer(h, pBuf);
    }

    // Frame publishing thread
    void publishLoop()
    {
        while (running_ && rclcpp::ok()) {
            cv::Mat img;
            {
                std::lock_guard<std::mutex> lock(queue_mtx_);
                if (!frame_queue_.empty()) {
                    img = std::move(frame_queue_.front());
                    frame_queue_.pop();
                }
            }
            if (!img.empty()) {
                std_msgs::msg::Header hdr;
                hdr.stamp = now();
                auto msg = cv_bridge::CvImage(hdr, output_format_, img).toImageMsg();
                image_pub_.publish(msg);
                frames_published_++;
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
        }
    }

    // Diagnostics publisher
    void publishStatus()
    {
        diagnostic_msgs::msg::DiagnosticArray arr;
        diagnostic_msgs::msg::DiagnosticStatus s;
        s.name = "huateng_camera_status";
        s.hardware_id = "cam_" + std::to_string(camera_id_);

        double exp_us = 0.0;
        int gain = 0;
        BOOL ae = 0;
        CameraGetExposureTime(hCamera_, &exp_us);
        CameraGetAnalogGain(hCamera_, &gain);
        CameraGetAeState(hCamera_, &ae);

        s.message = "Exp:" + std::to_string(exp_us) + " Gain:" + std::to_string(gain) +
                    " AE:" + std::string(ae ? "ON" : "OFF");
        s.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        arr.status.push_back(s);
        status_pub_->publish(arr);
    }

    // Topic callbacks
    void onExposureTopic(const std_msgs::msg::Float32::SharedPtr msg) { setExposure(msg->data); }
    void onGainTopic(const std_msgs::msg::Float32::SharedPtr msg) { setGain(msg->data); }
    void onWhiteBalanceTopic(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data) CameraSetOnceWB(hCamera_);
    }

    // SDK setters
    bool setExposure(float exp_us)
    {
        CameraSetAeState(hCamera_, FALSE);
        if (CameraSetExposureTime(hCamera_, exp_us) == CAMERA_STATUS_SUCCESS) {
            RCLCPP_INFO(get_logger(), "Exposure set to %.2f µs", exp_us);
            return true;
        }
        RCLCPP_WARN(get_logger(), "Failed to set exposure");
        return false;
    }

    bool setGain(float gain)
    {
        CameraSetAeState(hCamera_, FALSE);
        if (CameraSetAnalogGain(hCamera_, static_cast<int>(gain)) == CAMERA_STATUS_SUCCESS) {
            RCLCPP_INFO(get_logger(), "Gain set to %.2f dB", gain);
            return true;
        }
        RCLCPP_WARN(get_logger(), "Failed to set gain");
        return false;
    }

    /// Internal members
    CameraHandle hCamera_ = -1;
    tSdkCameraCapbility capability_;
    std::atomic<bool> running_;
    std::mutex queue_mtx_;
    std::queue<cv::Mat> frame_queue_;
    std::thread pub_thread_;
    int camera_id_{0}, fps_{30}, frames_published_{0};
    double exposure_time_{10000.0}; 
    bool auto_exposure_{false};

    std::string output_format_, param_save_path_;

    image_transport::Publisher image_pub_;
    std::shared_ptr<camera_info_manager::CameraInfoManager> camera_info_man_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr status_pub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_exposure_, sub_gain_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_white_balance_;
    rclcpp::Service<SetFloatSrv>::SharedPtr srv_set_exposure_, srv_set_gain_;
    rclcpp::TimerBase::SharedPtr status_timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HuatengCameraNode>();
    node->init(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
