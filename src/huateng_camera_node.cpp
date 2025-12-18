#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <opencv2/opencv.hpp>
#include "CameraApi.h" 

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
    HuatengCameraNode() : Node("huateng_camera_node"), running_(true) {}

    ~HuatengCameraNode() {
        running_ = false;
        if (pub_thread_.joinable()) pub_thread_.join();
        if (hCamera_ > 0) {
            CameraSetCallbackFunction(hCamera_, nullptr, nullptr, nullptr);
            CameraUnInit(hCamera_);
        }
    }

    void init() {
        declareNodeParameters();
        setupRosInterfaces();

        if (!initializeCameraHardware()) {
            rclcpp::shutdown(); return;
        }

        applyInitialConfiguration();

        // Timer de Monitoramento (1Hz)
        timer_ = create_wall_timer(
            std::chrono::seconds(1), 
            std::bind(&HuatengCameraNode::publishCameraStatus, this)
        );

        startProcessing();
    }

private:
    // --- 1. DECLARAÇÃO DE PARÂMETROS ---
    void declareNodeParameters() {
        camera_id_ = declare_parameter<int>("camera_id", 0);
        fps_ = declare_parameter<int>("fps", 30);
        output_format_ = declare_parameter<std::string>("output_format", "bgr8");
        
        declare_parameter("exposure_time", 10000.0);
        declare_parameter("auto_exposure", true); // Padrao Auto
        
        declare_parameter("black_level", 5);
        declare_parameter("contrast", 100);
        declare_parameter("gamma", 120);
        declare_parameter("sharpness", 50);
        
        declare_parameter("white_balance_auto", true);
        declare_parameter("gain_r", 120);
        declare_parameter("gain_g", 100);
        declare_parameter("gain_b", 80);

        declare_parameter("ae_roi_top_skip", 0.35);
        declare_parameter("ae_roi_bottom_skip", 0.17);
        
        roi_top_ratio_ = get_parameter("ae_roi_top_skip").as_double();
        roi_bottom_ratio_ = get_parameter("ae_roi_bottom_skip").as_double();
    }

    // --- 2. CONFIGURAÇÃO DOS TÓPICOS ---
    void setupRosInterfaces() {
        image_transport::ImageTransport it(shared_from_this());
        image_pub_ = it.advertise("image_raw", 1);
        status_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("camera_status", 1);

        // --- CONTROLE DE EXPOSIÇÃO ---
        
        // 1. Auto Exposição (Faltava este!)
        sub_auto_exp_ = create_subscription<std_msgs::msg::Bool>("set_auto_exposure", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                CameraSetAeState(hCamera_, msg->data ? TRUE : FALSE);
                RCLCPP_INFO(get_logger(), "Auto Exposure: %s", msg->data ? "ON" : "OFF");
            });

        // 2. Exposição Manual
        sub_exposure_ = create_subscription<std_msgs::msg::Float32>("set_exposure", 10, 
            [this](const std_msgs::msg::Float32::SharedPtr msg) { setExposure(msg->data); });
        
        // 3. Ganho Analógico
        sub_gain_ = create_subscription<std_msgs::msg::Float32>("set_gain", 10, 
            [this](const std_msgs::msg::Float32::SharedPtr msg) { setGain(msg->data); });
        
        // --- CONTROLE DE IMAGEM (ISP) ---
        sub_black_level_ = create_subscription<std_msgs::msg::Float32>("set_black_level", 10, 
            [this](const std_msgs::msg::Float32::SharedPtr msg) { CameraSetBlackLevel(hCamera_, (int)msg->data); });

        sub_contrast_ = create_subscription<std_msgs::msg::Float32>("set_contrast", 10, 
            [this](const std_msgs::msg::Float32::SharedPtr msg) { CameraSetContrast(hCamera_, (int)msg->data); });

        sub_gamma_ = create_subscription<std_msgs::msg::Float32>("set_gamma", 10, 
            [this](const std_msgs::msg::Float32::SharedPtr msg) { CameraSetGamma(hCamera_, (int)msg->data); });

        sub_sharpness_ = create_subscription<std_msgs::msg::Float32>("set_sharpness", 10, 
            [this](const std_msgs::msg::Float32::SharedPtr msg) { CameraSetSharpness(hCamera_, (int)msg->data); });

        // --- CONTROLE DE COR ---
        // Aqui está a correção para o WB funcionar com o Checkbox
        sub_wb_auto_ = create_subscription<std_msgs::msg::Bool>("set_white_balance_auto", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                CameraSetWbMode(hCamera_, msg->data ? TRUE : FALSE);
                RCLCPP_INFO(get_logger(), "Auto WB: %s", msg->data ? "ON" : "OFF");
            });
            
        sub_gain_r_ = create_subscription<std_msgs::msg::Float32>("set_gain_r", 10, 
            [this](const std_msgs::msg::Float32::SharedPtr msg) { updateRGBGain(0, (int)msg->data); });
        sub_gain_g_ = create_subscription<std_msgs::msg::Float32>("set_gain_g", 10, 
            [this](const std_msgs::msg::Float32::SharedPtr msg) { updateRGBGain(1, (int)msg->data); });
        sub_gain_b_ = create_subscription<std_msgs::msg::Float32>("set_gain_b", 10, 
            [this](const std_msgs::msg::Float32::SharedPtr msg) { updateRGBGain(2, (int)msg->data); });

        // --- ROI ---
        sub_roi_top_ = create_subscription<std_msgs::msg::Float32>("set_ae_roi_top_skip", 10,
            [this](const std_msgs::msg::Float32::SharedPtr msg) {
                roi_top_ratio_ = msg->data;
                updateAutoExposureROI();
            });
        sub_roi_bottom_ = create_subscription<std_msgs::msg::Float32>("set_ae_roi_bottom_skip", 10,
            [this](const std_msgs::msg::Float32::SharedPtr msg) {
                roi_bottom_ratio_ = msg->data;
                updateAutoExposureROI();
            });
    }

    // --- 3. HARDWARE ---
    bool initializeCameraHardware() {
        if (CameraSdkInit(0) != CAMERA_STATUS_SUCCESS) return false;
        tSdkCameraDevInfo devs[10]; int num = 10;
        if (CameraEnumerateDevice(devs, &num) != CAMERA_STATUS_SUCCESS || num <= 0) return false;
        if (CameraInit(&devs[camera_id_], -1, -1, &hCamera_) != CAMERA_STATUS_SUCCESS) return false;
        
        CameraGetCapability(hCamera_, &capability_);
        CameraPlay(hCamera_);
        CameraSetIspOutFormat(hCamera_, CAMERA_MEDIA_TYPE_BGR8);
        return true;
    }

    // --- 4. CONFIGURAÇÃO INICIAL ---
    void applyInitialConfiguration() {
        CameraSetLutMode(hCamera_, 1); 
        CameraSetBlackLevel(hCamera_, get_parameter("black_level").as_int());
        CameraSetContrast(hCamera_, get_parameter("contrast").as_int());
        CameraSetGamma(hCamera_, get_parameter("gamma").as_int());
        CameraSetSharpness(hCamera_, get_parameter("sharpness").as_int());

        // Configura Auto Exposure Inicial
        bool auto_exp = get_parameter("auto_exposure").as_bool();
        CameraSetAeState(hCamera_, auto_exp ? TRUE : FALSE);

        // Configura WB
        bool auto_wb = get_parameter("white_balance_auto").as_bool();
        CameraSetWbMode(hCamera_, auto_wb ? TRUE : FALSE);
        if (!auto_wb) {
            CameraSetGain(hCamera_, 
                get_parameter("gain_r").as_int(),
                get_parameter("gain_g").as_int(),
                get_parameter("gain_b").as_int());
        }

        updateAutoExposureROI();
        RCLCPP_INFO(get_logger(), "Configuração Inicial Aplicada.");
    }

    // --- 5. FUNÇÕES AUXILIARES ---
    void publishCameraStatus() {
        if (hCamera_ <= 0) return;

        diagnostic_msgs::msg::DiagnosticArray msg;
        msg.header.stamp = now();

        diagnostic_msgs::msg::DiagnosticStatus status;
        status.name = "Huateng Camera Parameters";
        status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        status.hardware_id = std::to_string(camera_id_);

        auto add_kv = [&](std::string k, std::string v) {
            diagnostic_msgs::msg::KeyValue kv;
            kv.key = k;
            kv.value = v;
            status.values.push_back(kv);
        };

        double exposure;
        if (CameraGetExposureTime(hCamera_, &exposure) == CAMERA_STATUS_SUCCESS) {
            add_kv("Exposure (us)", std::to_string((int)exposure));
        }

        int gain, r, g, b;
        if (CameraGetAnalogGain(hCamera_, &gain) == CAMERA_STATUS_SUCCESS) {
            add_kv("Analog Gain", std::to_string(gain));
        }
        
        BOOL ae_state;
        if (CameraGetAeState(hCamera_, &ae_state) == CAMERA_STATUS_SUCCESS) {
            add_kv("Auto Exposure", ae_state ? "ON" : "OFF");
        }

        if (CameraGetGain(hCamera_, &r, &g, &b) == CAMERA_STATUS_SUCCESS) {
            add_kv("Gain R", std::to_string(r));
            add_kv("Gain G", std::to_string(g));
            add_kv("Gain B", std::to_string(b));
        }

        int val;
        CameraGetContrast(hCamera_, &val); add_kv("Contrast", std::to_string(val));
        CameraGetGamma(hCamera_, &val);    add_kv("Gamma", std::to_string(val));
        
        add_kv("ROI Top Skip", std::to_string(roi_top_ratio_));
        add_kv("ROI Bottom Skip", std::to_string(roi_bottom_ratio_));

        msg.status.push_back(status);
        status_pub_->publish(msg);
    }

    void updateAutoExposureROI() {
        tSdkImageResolution res;
        if (CameraGetImageResolution(hCamera_, &res) != CAMERA_STATUS_SUCCESS) return;
        
        int h = res.iHeight;
        int y_start = static_cast<int>(h * roi_top_ratio_);
        int roi_h = h - y_start - static_cast<int>(h * roi_bottom_ratio_);
        if (roi_h < 10) roi_h = 10;

        CameraSetAeWindow(hCamera_, 0, y_start, res.iWidth, roi_h);
        CameraSetAeWinVisible(hCamera_, TRUE);
    }

    void updateRGBGain(int channel, int value) {
        int r, g, b;
        CameraGetGain(hCamera_, &r, &g, &b);
        if (channel == 0) r = value;
        else if (channel == 1) g = value;
        else if (channel == 2) b = value;
        
        // Desativa Auto WB se tentar setar ganho manual
        CameraSetWbMode(hCamera_, FALSE); 
        CameraSetGain(hCamera_, r, g, b);
    }

    bool setExposure(float exp_us) {
        CameraSetAeState(hCamera_, FALSE); // Força manual ao receber comando
        return (CameraSetExposureTime(hCamera_, exp_us) == CAMERA_STATUS_SUCCESS);
    }
    bool setGain(float gain) {
        CameraSetAeState(hCamera_, FALSE);
        return (CameraSetAnalogGain(hCamera_, static_cast<int>(gain)) == CAMERA_STATUS_SUCCESS);
    }

    void startProcessing() {
        CameraSetCallbackFunction(hCamera_, &HuatengCameraNode::GrabImageCallback, this, nullptr);
        pub_thread_ = std::thread([this]() { publishLoop(); });
    }

    static void GrabImageCallback(CameraHandle h, BYTE* buf, tSdkFrameHead* head, PVOID ctx) {
        reinterpret_cast<HuatengCameraNode*>(ctx)->onFrame(h, buf, head);
    }

    void onFrame(CameraHandle h, BYTE* pBuf, tSdkFrameHead* head) {
        std::lock_guard<std::mutex> lock(queue_mtx_);
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
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
        }
    }

    // Variáveis
    CameraHandle hCamera_ = -1;
    tSdkCameraCapbility capability_;
    std::atomic<bool> running_;
    std::mutex queue_mtx_;
    std::queue<cv::Mat> frame_queue_;
    std::thread pub_thread_;
    int camera_id_{0}, fps_{30};
    std::string output_format_;
    
    double roi_top_ratio_ = 0.0;
    double roi_bottom_ratio_ = 0.0;

    image_transport::Publisher image_pub_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr status_pub_;
    rclcpp::TimerBase::SharedPtr timer_; // Timer declarado corretamente aqui
    
    // Todos os Subscribers necessários
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_exposure_, sub_gain_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_auto_exp_; // NOVO
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_black_level_, sub_contrast_, sub_gamma_, sub_sharpness_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_roi_top_, sub_roi_bottom_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_gain_r_, sub_gain_g_, sub_gain_b_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_wb_auto_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HuatengCameraNode>();
    node->init(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}