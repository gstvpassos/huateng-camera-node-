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

    ~HuatengCameraNode()
    {
        running_ = false;
        if (pub_thread_.joinable()) pub_thread_.join();
        if (hCamera_ > 0) {
            CameraSetCallbackFunction(hCamera_, nullptr, nullptr, nullptr);
            CameraUnInit(hCamera_);
        }
    }

    void init() {
        // 1. Declara todos os parâmetros do ROS
        declareNodeParameters();

        // 2. Configura Publishers, Subscribers e Services
        setupRosInterfaces();

        // 3. Inicializa a SDK e abre a câmera física
        if (!initializeCameraHardware()) {
            rclcpp::shutdown();
            return;
        }

        // 4. Aplica as configurações iniciais (Contraste, ROI, Cor, etc)
        applyInitialConfiguration();

        // 5. Inicia as threads de captura e publicação
        startProcessing();
    }

private:

    // Declaracao dos parametros da camera
    void declareNodeParameters() {
        // Sistema
        camera_id_ = declare_parameter<int>("camera_id", 0);
        fps_ = declare_parameter<int>("fps", 30);
        output_format_ = declare_parameter<std::string>("output_format", "bgr8");
        param_save_path_ = declare_parameter<std::string>("param_save_path", "/tmp/huateng_params.bin");
        
        // Exposição
        exposure_time_ = declare_parameter<double>("exposure_time", 10000.0);
        auto_exposure_ = declare_parameter<bool>("exposure_auto", false);

        // ISP (Processamento de Imagem)
        declare_parameter("black_level", 5);
        declare_parameter("contrast", 100);
        declare_parameter("gamma", 120);
        declare_parameter("sharpness", 50);

        // Balanço de Branco / Cor
        declare_parameter("white_balance_auto", true);
        declare_parameter("gain_r", 120);
        declare_parameter("gain_g", 100);
        declare_parameter("gain_b", 80);

        // ROI de Exposição (Auto Exposure Window)
        declare_parameter("ae_roi_x", 0);
        declare_parameter("ae_roi_y", 250);
        declare_parameter("ae_roi_w", 1024);
        declare_parameter("ae_roi_h", 350);
        declare_parameter("ae_roi_draw", true);

        // ROI Proporcional (Ratios)
        declare_parameter("ae_roi_top_skip", 0.35);
        declare_parameter("ae_roi_bottom_skip", 0.17);
    }

    // Configuracao da interface do ROS
    void setupRosInterfaces() {
        image_transport::ImageTransport it(shared_from_this());
        image_pub_ = it.advertise("image_raw", 1);
        camera_info_man_ = std::make_shared<camera_info_manager::CameraInfoManager>(this);
        camera_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", 1);
        status_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("camera_status", 1);

        // Subscribers Legados
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
    }

    // Inicializacao do SDK e da camera
    bool initializeCameraHardware() {
        if (CameraSdkInit(0) != CAMERA_STATUS_SUCCESS) {
            RCLCPP_ERROR(get_logger(), "Failed to initialize Huateng SDK");
            return false;
        }

        tSdkCameraDevInfo devs[10];
        int num = 10;
        if (CameraEnumerateDevice(devs, &num) != CAMERA_STATUS_SUCCESS || num <= 0) {
            RCLCPP_ERROR(get_logger(), "No cameras detected");
            return false;
        }

        if (CameraInit(&devs[camera_id_], -1, -1, &hCamera_) != CAMERA_STATUS_SUCCESS) {
            RCLCPP_ERROR(get_logger(), "Failed to open camera %d", camera_id_);
            return false;
        }

        CameraGetCapability(hCamera_, &capability_);
        CameraPlay(hCamera_);
        CameraSetIspOutFormat(hCamera_, CAMERA_MEDIA_TYPE_BGR8);
        
        // Log capability info
        int exp_min = capability_.sExposeDesc.uiExposeTimeMin;
        int exp_max = capability_.sExposeDesc.uiExposeTimeMax;
        RCLCPP_INFO(get_logger(), "Camera Init OK. Exposure Range: %d-%d µs", exp_min, exp_max);
        
        return true;
    }

    // Aplicacao das configuracoes iniciais
    void applyInitialConfiguration() {
        // Ativa Processamento Dinâmico (LUT)
        CameraSetLutMode(hCamera_, 1); // 1 = LUTMODE_PARAM_GEN

        // Aplica ISP (Contraste, Gamma, etc)
        CameraSetBlackLevel(hCamera_, get_parameter("black_level").as_int());
        CameraSetContrast(hCamera_, get_parameter("contrast").as_int());
        CameraSetGamma(hCamera_, get_parameter("gamma").as_int());
        CameraSetSharpness(hCamera_, get_parameter("sharpness").as_int());

        // Aplica Balanço de Branco
        bool auto_wb = get_parameter("white_balance_auto").as_bool();
        CameraSetWbMode(hCamera_, auto_wb ? TRUE : FALSE);
        if (!auto_wb) {
            CameraSetGain(hCamera_, 
                get_parameter("gain_r").as_int(),
                get_parameter("gain_g").as_int(),
                get_parameter("gain_b").as_int());
        }

        // Aplica Exposição
        if (auto_exposure_) {
            CameraSetAeState(hCamera_, TRUE);
        } else {
            CameraSetAeState(hCamera_, FALSE);
            CameraSetExposureTime(hCamera_, exposure_time_);
        }

        // Aplica ROI Automático (Calcula baseado nos ratios)
        updateAutoExposureROI();

        // Registra o Callback para mudanças futuras (via RQT/Terminal)
        params_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&HuatengCameraNode::parametersCallback, this, std::placeholders::_1));
            
        RCLCPP_INFO(get_logger(), "Settings Applied. Ready for LKAS.");
    }

    // Atualiza o ROI de acordo com a resolucao da camera
    void updateAutoExposureROI() {
        tSdkImageResolution res;
        if (CameraGetImageResolution(hCamera_, &res) != CAMERA_STATUS_SUCCESS) return;
        
        int w = res.iWidth;
        int h = res.iHeight;
        double top_ratio = get_parameter("ae_roi_top_skip").as_double();
        double bottom_ratio = get_parameter("ae_roi_bottom_skip").as_double();
        bool draw = get_parameter("ae_roi_draw").as_bool();

        int y_start = static_cast<int>(h * top_ratio);
        int roi_h = h - y_start - static_cast<int>(h * bottom_ratio);
        if (roi_h < 10) roi_h = 10;

        CameraSetAeWindow(hCamera_, 0, y_start, w, roi_h);
        CameraSetAeWinVisible(hCamera_, draw ? TRUE : FALSE);
        
        RCLCPP_INFO(get_logger(), "ROI Updated: Skip Top %dpx | Skip Bottom %dpx | WinH %dpx", 
                    y_start, static_cast<int>(h * bottom_ratio), roi_h);
    }

    // Defincao dos callback e thread
    void startProcessing() {
        CameraSetCallbackFunction(hCamera_, &HuatengCameraNode::GrabImageCallback, this, nullptr);
        pub_thread_ = std::thread([this]() { publishLoop(); });
        status_timer_ = create_wall_timer(1s, std::bind(&HuatengCameraNode::publishStatus, this));
        RCLCPP_INFO(get_logger(), "Streaming Started.");
    }

    // Callback dinâmico para RQT/Param Set
    rcl_interfaces::msg::SetParametersResult parametersCallback(
        const std::vector<rclcpp::Parameter> &parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "success";
        
        bool roi_changed = false;

        for (const auto &param : parameters)
        {
            // ISP
            if (param.get_name() == "black_level") CameraSetBlackLevel(hCamera_, param.as_int());
            else if (param.get_name() == "contrast") CameraSetContrast(hCamera_, param.as_int());
            else if (param.get_name() == "gamma") CameraSetGamma(hCamera_, param.as_int());
            else if (param.get_name() == "sharpness") CameraSetSharpness(hCamera_, param.as_int());
            
            // Exposure
            else if (param.get_name() == "exposure_time") CameraSetExposureTime(hCamera_, param.as_double());
            
            // Color / WB
            else if (param.get_name() == "white_balance_auto") {
                bool auto_wb = param.as_bool();
                CameraSetWbMode(hCamera_, auto_wb ? TRUE : FALSE);
                if (!auto_wb) { // Re-apply manual gains if switching to manual
                    CameraSetGain(hCamera_, get_parameter("gain_r").as_int(), get_parameter("gain_g").as_int(), get_parameter("gain_b").as_int());
                }
            }
            else if (param.get_name() == "gain_r" || param.get_name() == "gain_g" || param.get_name() == "gain_b") {
                if (!get_parameter("white_balance_auto").as_bool()) {
                    int r = (param.get_name() == "gain_r") ? param.as_int() : get_parameter("gain_r").as_int();
                    int g = (param.get_name() == "gain_g") ? param.as_int() : get_parameter("gain_g").as_int();
                    int b = (param.get_name() == "gain_b") ? param.as_int() : get_parameter("gain_b").as_int();
                    CameraSetGain(hCamera_, r, g, b);
                }
            }

            // ROI Check
            else if (param.get_name() == "ae_roi_top_skip" || param.get_name() == "ae_roi_bottom_skip" || param.get_name() == "ae_roi_draw") {
                roi_changed = true;
            }
        }

        if (roi_changed) updateAutoExposureROI();
        return result;
    }

    static void GrabImageCallback(CameraHandle h, BYTE* buf, tSdkFrameHead* head, PVOID ctx) {
        reinterpret_cast<HuatengCameraNode*>(ctx)->onFrame(h, buf, head);
    }

    void onFrame(CameraHandle h, BYTE* pBuf, tSdkFrameHead* head) {
        std::lock_guard<std::mutex> lock(queue_mtx_);
        // Aqui a imagem já vem processada pelo hardware (gamma, contraste, etc)
        cv::Mat img(cv::Size(head->iWidth, head->iHeight), CV_8UC3);
        CameraImageProcess(h, pBuf, img.data, head);
        frame_queue_.push(std::move(img));
        CameraReleaseImageBuffer(h, pBuf);
    }

    void publishLoop() {
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

    void publishStatus() { /* (O código antigo de diagnóstico permanece o mesmo aqui) */ 
        diagnostic_msgs::msg::DiagnosticArray arr;
        diagnostic_msgs::msg::DiagnosticStatus s;
        s.name = "huateng_camera_status";
        s.hardware_id = "cam_" + std::to_string(camera_id_);
        double exp_us = 0.0; int gain = 0; BOOL ae = 0;
        CameraGetExposureTime(hCamera_, &exp_us);
        CameraGetAnalogGain(hCamera_, &gain);
        CameraGetAeState(hCamera_, &ae);
        s.message = "Exp:" + std::to_string(exp_us) + " Gain:" + std::to_string(gain) + " AE:" + std::string(ae ? "ON" : "OFF");
        s.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        arr.status.push_back(s);
        status_pub_->publish(arr);
    }

    // Callbacks Legados
    void onExposureTopic(const std_msgs::msg::Float32::SharedPtr msg) { setExposure(msg->data); }
    void onGainTopic(const std_msgs::msg::Float32::SharedPtr msg) { setGain(msg->data); }
    void onWhiteBalanceTopic(const std_msgs::msg::Bool::SharedPtr msg) { if (msg->data) CameraSetOnceWB(hCamera_); }

    bool setExposure(float exp_us) {
        CameraSetAeState(hCamera_, FALSE);
        return (CameraSetExposureTime(hCamera_, exp_us) == CAMERA_STATUS_SUCCESS);
    }
    bool setGain(float gain) {
        CameraSetAeState(hCamera_, FALSE);
        return (CameraSetAnalogGain(hCamera_, static_cast<int>(gain)) == CAMERA_STATUS_SUCCESS);
    }

    // Membros Internos
    rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;
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

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HuatengCameraNode>();
    node->init(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}