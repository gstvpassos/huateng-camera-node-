#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp> 
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
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
        // 1. Declara parâmetros (Apenas para valor inicial no boot)
        declareNodeParameters();

        // 2. Cria os Tópicos (Subscribers) para controle dinâmico
        setupRosInterfaces();

        // 3. Abre a Câmera
        if (!initializeCameraHardware()) {
            rclcpp::shutdown(); return;
        }

        // 4. Aplica valores iniciais
        applyInitialConfiguration();

        // 5. Inicia Threads
        startProcessing();
    }

private:
    // --- 1. DECLARAÇÃO DE PARÂMETROS (Configuração Inicial) ---
    void declareNodeParameters() {
        // Sistema
        camera_id_ = declare_parameter<int>("camera_id", 0);
        fps_ = declare_parameter<int>("fps", 30);
        output_format_ = declare_parameter<std::string>("output_format", "bgr8");
        
        // Valores Iniciais de Imagem
        declare_parameter("exposure_time", 10000.0);
        declare_parameter("black_level", 5);
        declare_parameter("contrast", 100);
        declare_parameter("gamma", 120);
        declare_parameter("sharpness", 50);
        
        // Cores
        declare_parameter("white_balance_auto", true);
        declare_parameter("gain_r", 120);
        declare_parameter("gain_g", 100);
        declare_parameter("gain_b", 80);

        // ROI (Proporcional)
        declare_parameter("ae_roi_top_skip", 0.35);    // Pula 35% do topo
        declare_parameter("ae_roi_bottom_skip", 0.17); // Pula 17% do fundo
        
        // Estado interno inicial das variáveis de ROI
        roi_top_ratio_ = get_parameter("ae_roi_top_skip").as_double();
        roi_bottom_ratio_ = get_parameter("ae_roi_bottom_skip").as_double();
    }

    // --- 2. CONFIGURAÇÃO DOS TÓPICOS (SUBSCRIBERS) ---
    void setupRosInterfaces() {
        // Publishers
        image_transport::ImageTransport it(shared_from_this());
        image_pub_ = it.advertise("image_raw", 1);
        status_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("camera_status", 1);

        // --- SUBSCRIBERS (CONTROLE DE EXECUÇÃO) ---
        
        // Exposição e Ganho (Legado)
        sub_exposure_ = create_subscription<std_msgs::msg::Float32>("set_exposure", 10, 
            [this](const std_msgs::msg::Float32::SharedPtr msg) { setExposure(msg->data); });
        sub_gain_ = create_subscription<std_msgs::msg::Float32>("set_gain", 10, 
            [this](const std_msgs::msg::Float32::SharedPtr msg) { setGain(msg->data); });
        
        // ISP (Novos Controles) - Usando Float32 para manter padrão, cast para Int dentro
        sub_black_level_ = create_subscription<std_msgs::msg::Float32>("set_black_level", 10, 
            [this](const std_msgs::msg::Float32::SharedPtr msg) { 
                CameraSetBlackLevel(hCamera_, (int)msg->data); 
                RCLCPP_INFO(get_logger(), "Topico: BlackLevel -> %d", (int)msg->data);
            });

        sub_contrast_ = create_subscription<std_msgs::msg::Float32>("set_contrast", 10, 
            [this](const std_msgs::msg::Float32::SharedPtr msg) { 
                CameraSetContrast(hCamera_, (int)msg->data); 
                RCLCPP_INFO(get_logger(), "Topico: Contrast -> %d", (int)msg->data);
            });

        sub_gamma_ = create_subscription<std_msgs::msg::Float32>("set_gamma", 10, 
            [this](const std_msgs::msg::Float32::SharedPtr msg) { 
                CameraSetGamma(hCamera_, (int)msg->data); 
                RCLCPP_INFO(get_logger(), "Topico: Gamma -> %d", (int)msg->data);
            });

        sub_sharpness_ = create_subscription<std_msgs::msg::Float32>("set_sharpness", 10, 
            [this](const std_msgs::msg::Float32::SharedPtr msg) { 
                CameraSetSharpness(hCamera_, (int)msg->data); 
                RCLCPP_INFO(get_logger(), "Topico: Sharpness -> %d", (int)msg->data);
            });

        // Cor / White Balance
        sub_wb_auto_ = create_subscription<std_msgs::msg::Bool>("set_white_balance_auto", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                CameraSetWbMode(hCamera_, msg->data ? TRUE : FALSE);
                RCLCPP_INFO(get_logger(), "Topico: WB Auto -> %s", msg->data ? "ON" : "OFF");
            });
            
        // Ganhos Manuais de Cor (R, G, B)
        // Dica: Para ajustar via topico, envie "{data: 120.0}" etc.
        sub_gain_r_ = create_subscription<std_msgs::msg::Float32>("set_gain_r", 10, 
            [this](const std_msgs::msg::Float32::SharedPtr msg) { updateRGBGain(0, (int)msg->data); });
        sub_gain_g_ = create_subscription<std_msgs::msg::Float32>("set_gain_g", 10, 
            [this](const std_msgs::msg::Float32::SharedPtr msg) { updateRGBGain(1, (int)msg->data); });
        sub_gain_b_ = create_subscription<std_msgs::msg::Float32>("set_gain_b", 10, 
            [this](const std_msgs::msg::Float32::SharedPtr msg) { updateRGBGain(2, (int)msg->data); });

        // ROI Dinâmico (Top Skip / Bottom Skip) - 0.0 a 1.0
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

    // --- 4. APLICAÇÃO INICIAL ---
    void applyInitialConfiguration() {
        // Ativa LUT
        CameraSetLutMode(hCamera_, 1); 

        // Aplica valores do YAML
        CameraSetBlackLevel(hCamera_, get_parameter("black_level").as_int());
        CameraSetContrast(hCamera_, get_parameter("contrast").as_int());
        CameraSetGamma(hCamera_, get_parameter("gamma").as_int());
        CameraSetSharpness(hCamera_, get_parameter("sharpness").as_int());

        // Configura Cor
        bool auto_wb = get_parameter("white_balance_auto").as_bool();
        CameraSetWbMode(hCamera_, auto_wb ? TRUE : FALSE);
        if (!auto_wb) {
            CameraSetGain(hCamera_, 
                get_parameter("gain_r").as_int(),
                get_parameter("gain_g").as_int(),
                get_parameter("gain_b").as_int());
        }

        // Aplica ROI Inicial
        updateAutoExposureROI();
        RCLCPP_INFO(get_logger(), "Configuração Inicial Aplicada.");
    }

    // --- 5. LÓGICA DE EXECUÇÃO E AUXILIARES ---
    
    // Função auxiliar para atualizar ROI baseada nas variáveis membro
    void updateAutoExposureROI() {
        tSdkImageResolution res;
        if (CameraGetImageResolution(hCamera_, &res) != CAMERA_STATUS_SUCCESS) return;
        
        int h = res.iHeight;
        int y_start = static_cast<int>(h * roi_top_ratio_);
        int roi_h = h - y_start - static_cast<int>(h * roi_bottom_ratio_);
        if (roi_h < 10) roi_h = 10;

        CameraSetAeWindow(hCamera_, 0, y_start, res.iWidth, roi_h);
        CameraSetAeWinVisible(hCamera_, TRUE); // Sempre desenha se estiver mexendo
        
        RCLCPP_INFO(get_logger(), "ROI Atualizado: TopSkip %.2f | BottomSkip %.2f", roi_top_ratio_, roi_bottom_ratio_);
    }

    // Função auxiliar para atualizar apenas um canal de cor sem perder os outros
    void updateRGBGain(int channel, int value) {
        int r, g, b;
        CameraGetGain(hCamera_, &r, &g, &b);
        if (channel == 0) r = value;
        else if (channel == 1) g = value;
        else if (channel == 2) b = value;
        
        // Se estiver em Auto WB, desativa temporariamente para aplicar manual?
        // Ou assume que o usuário já desativou via topico /set_white_balance_auto
        CameraSetGain(hCamera_, r, g, b);
        RCLCPP_INFO(get_logger(), "Ganhos RGB: %d, %d, %d", r, g, b);
    }

    bool setExposure(float exp_us) {
        CameraSetAeState(hCamera_, FALSE);
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
    
    // Estado do ROI
    double roi_top_ratio_ = 0.0;
    double roi_bottom_ratio_ = 0.0;

    image_transport::Publisher image_pub_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr status_pub_;
    
    // Subscribers de Controle
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_exposure_, sub_gain_;
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