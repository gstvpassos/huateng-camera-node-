# 🚀 Huateng USB3 Camera ROS2 node

ROS Humble node for USB3 camera data configuration and acquisition.
---
## Pré-requisitos
| Item | Versão testada | Observações |
|------|----------------|------------|
| **Distribuição** | Ubuntu 22.04 LTS | |
| **ROS 2** | Humble Hawksbill | Siga a [instalação oficial](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html). |
|SDK da Câmera para Linux | HuaTeng Vision SDK for Linux_V2.1.0.49(250108) | https://huatengvision.com/download/1/ |

---
  ## 1) SDK da câmera:  
  Descompactar o arquivo (linuxSDK_V2.1.0.49(250108).tar.gz) e instalar: 
```bash
  sudo ./install.sh)
```
 Caso install.sh não seja reconhecido como arquivo executável:
```bash
	sudo chmod +x install.sh
```

## 2) Dependências ROS para a câmera:
	sudo apt install ros-humble-camera-info-manager

## 3) Para compilar a aplicação:
```bash
cd ~/ros2_ws
colcon build --packages-select huateng_camera_node --symlink-install
#colcon build --symlink-install
source install/setup.bash
ros2 run huateng_camera_node huateng_camera_node_exec \
  --ros-args --params-file ~/ros2_ws/src/huateng_camera_node/config/camera_params.yaml
```

## 4) Para ajustar parâmetros dinamicamente (publicar em tópicos):
```bash
# setar exposição para 20000 µs
ros2 topic pub --once /set_exposure std_msgs/msg/Float32 "{data: 20000.0}"

# setar ganho raw (veja capability para step)
ros2 topic pub --once /set_gain std_msgs/msg/Float32 "{data: 8.0}"

# ativa auto exposure
ros2 topic pub --once /set_auto_exposure std_msgs/msg/Bool "{data: true}"

# executar white balance uma vez
ros2 topic pub --once /do_white_balance std_msgs/msg/Bool "{data: true}"

# salvar parâmetros do SDK para arquivo
ros2 topic pub --once /save_parameters std_msgs/msg/Bool "{data: true}"

# carregar parâmetros do SDK de arquivo
ros2 topic pub --once /load_parameters std_msgs/msg/Bool "{data: true}"

# 0 (sem contraste) a 100+ (binarizado)
ros2 topic pub --once /set_contrast std_msgs/msg/Float32 "{data: 100.0}"

# 100 (neutro). Tente 140 para separar faixa do asfalto
ros2 topic pub --once /set_gamma std_msgs/msg/Float32 "{data: 140.0}"

# 0 a 10
ros2 topic pub --once /set_black_level std_msgs/msg/Float32 "{data: 5.0}"

# Pular 35% do topo (Céu)
ros2 topic pub --once /set_ae_roi_top_skip std_msgs/msg/Float32 "{data: 0.35}"

# Pular 20% do fundo (Capô)
ros2 topic pub --once /set_ae_roi_bottom_skip std_msgs/msg/Float32 "{data: 0.20}"

# Primeiro desativa o Auto
ros2 topic pub --once /set_white_balance_auto std_msgs/msg/Bool "{data: false}"

# Aumenta Vermelho, Diminui Azul
ros2 topic pub --once /set_gain_r std_msgs/msg/Float32 "{data: 130.0}"
ros2 topic pub --once /set_gain_b std_msgs/msg/Float32 "{data: 80.0}"
```

## 5) Ajustar via serviços:
```bash
ros2 service call /set_exposure_srv example_interfaces/srv/SetFloat32 "{data: 18000.0}"
ros2 service call /set_gain_srv example_interfaces/srv/SetFloat32 "{data: 10.0}"
```

## 6) Monitorar status:
```bash
ros2 topic echo /camera_status
```

## 7) Visualizar as imagens:
    • RVIZ:
  ```bash
      ros2 run rviz2 rviz2
  ```
    • Rqt:
  ```bash
      ros2 run rqt_image_view rqt_image_view
      # selecione /image_raw (ou use image_transport/compressed se ativado)
  ```


## 8) Recursos implementados:
| Função | Tipo | ROS 2 interface |
|------|----------------|------------|
| **Captura de imagem** | Publisher | /image_raw |
| **Informações intrínsecas** | Publisher | /camera_info |
| **Status diagnóstico** | Publisher | /camera_status |
| **Exposição** | Topic & Service | /set_exposure, /set_exposure_srv |
| **Ganho** | Topic & Service | /set_gain, /set_gain_srv |
| **Auto WB** | Topic | /do_white_balance |
| **Parâmetros do SDK** | YAML e CameraGetCapability | logs + ranges |

