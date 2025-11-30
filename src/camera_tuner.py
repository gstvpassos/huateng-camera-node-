import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
import tkinter as tk
from tkinter import ttk
import threading

class CameraTunerGUI(Node):
    def __init__(self):
        super().__init__('camera_tuner_gui')

        # --- Publishers (Conectam com os Subscribers do seu nó C++) ---
        self.pub_contrast = self.create_publisher(Float32, '/set_contrast', 10)
        self.pub_gamma = self.create_publisher(Float32, '/set_gamma', 10)
        self.pub_black = self.create_publisher(Float32, '/set_black_level', 10)
        self.pub_sharp = self.create_publisher(Float32, '/set_sharpness', 10)
        self.pub_exposure = self.create_publisher(Float32, '/set_exposure', 10)
        
        self.pub_roi_top = self.create_publisher(Float32, '/set_ae_roi_top_skip', 10)
        self.pub_roi_bot = self.create_publisher(Float32, '/set_ae_roi_bottom_skip', 10)
        
        self.pub_wb_auto = self.create_publisher(Bool, '/set_white_balance_auto', 10)
        self.pub_gain_r = self.create_publisher(Float32, '/set_gain_r', 10)
        self.pub_gain_g = self.create_publisher(Float32, '/set_gain_g', 10)
        self.pub_gain_b = self.create_publisher(Float32, '/set_gain_b', 10)

        self.get_logger().info("Painel de Controle Iniciado!")

    # Funções de envio
    def send_float(self, publisher, value):
        msg = Float32()
        msg.data = float(value)
        publisher.publish(msg)

    def send_bool(self, publisher, value):
        msg = Bool()
        msg.data = bool(value)
        publisher.publish(msg)

# --- LÓGICA DA INTERFACE GRÁFICA (TKINTER) ---
def start_gui(node):
    root = tk.Tk()
    root.title("LKAS Camera Tuner")
    root.geometry("400x750")

    # Estilo
    style = ttk.Style()
    style.theme_use('clam')

    # --- BLOCO 1: IMAGEM (ISP) ---
    lf_isp = ttk.LabelFrame(root, text=" 1. Ajuste de Contraste (ISP) ")
    lf_isp.pack(fill="x", padx=10, pady=5)

    def cb_contrast(val): node.send_float(node.pub_contrast, val)
    create_slider(lf_isp, "Contraste", 0, 200, 100, cb_contrast)

    def cb_gamma(val): node.send_float(node.pub_gamma, val)
    create_slider(lf_isp, "Gamma (Escurecer Asfalto)", 0, 255, 120, cb_gamma)

    def cb_black(val): node.send_float(node.pub_black, val)
    create_slider(lf_isp, "Black Level (Cortar Ruído)", 0, 50, 5, cb_black)

    def cb_sharp(val): node.send_float(node.pub_sharp, val)
    create_slider(lf_isp, "Nitidez (Bordas)", 0, 100, 50, cb_sharp)

    # --- BLOCO 2: COR (WHITE BALANCE) ---
    lf_color = ttk.LabelFrame(root, text=" 2. Correção de Cor (Tirar Azul) ")
    lf_color.pack(fill="x", padx=10, pady=5)

    # Checkbox Auto WB
    wb_var = tk.BooleanVar(value=True)
    def toggle_wb():
        val = wb_var.get()
        node.send_bool(node.pub_wb_auto, val)
        state = "disabled" if val else "normal"
        scale_r.configure(state=state)
        scale_g.configure(state=state)
        scale_b.configure(state=state)

    chk_wb = ttk.Checkbutton(lf_color, text="Auto White Balance", variable=wb_var, command=toggle_wb)
    chk_wb.pack(pady=5)

    def cb_r(val): node.send_float(node.pub_gain_r, val)
    scale_r = create_slider(lf_color, "Red (R)", 0, 400, 120, cb_r)
    
    def cb_g(val): node.send_float(node.pub_gain_g, val)
    scale_g = create_slider(lf_color, "Green (G)", 0, 400, 100, cb_g)

    def cb_b(val): node.send_float(node.pub_gain_b, val)
    scale_b = create_slider(lf_color, "Blue (B)", 0, 400, 80, cb_b)
    
    # Inicia desativado se Auto estiver True
    toggle_wb()

    # --- BLOCO 3: ROI (CÉU E CAPÔ) ---
    lf_roi = ttk.LabelFrame(root, text=" 3. Ignorar Áreas (ROI de Exposição) ")
    lf_roi.pack(fill="x", padx=10, pady=5)

    def cb_top(val): node.send_float(node.pub_roi_top, float(val)/100)
    create_slider(lf_roi, "Ignorar Topo (%)", 0, 80, 35, cb_top)

    def cb_bot(val): node.send_float(node.pub_roi_bot, float(val)/100)
    create_slider(lf_roi, "Ignorar Fundo (%)", 0, 50, 17, cb_bot)

    # --- BLOCO 4: EXPOSIÇÃO MANUAL ---
    lf_exp = ttk.LabelFrame(root, text=" 4. Exposição (Se necessário) ")
    lf_exp.pack(fill="x", padx=10, pady=5)
    
    def cb_exp(val): node.send_float(node.pub_exposure, val)
    create_slider(lf_exp, "Tempo (µs)", 100, 30000, 10000, cb_exp)

    root.mainloop()

def create_slider(parent, label_text, min_val, max_val, default_val, callback):
    frame = ttk.Frame(parent)
    frame.pack(fill="x", padx=5, pady=2)
    
    lbl = ttk.Label(frame, text=f"{label_text}: {default_val}")
    lbl.pack(anchor="w")
    
    def on_move(val):
        v = float(val)
        lbl.config(text=f"{label_text}: {int(v) if v > 1 else v:.2f}")
        callback(v)

    scale = ttk.Scale(frame, from_=min_val, to=max_val, command=on_move)
    scale.set(default_val)
    scale.pack(fill="x")
    return scale

def main():
    rclpy.init()
    node = CameraTunerGUI()
    
    # Roda o ROS em uma thread separada para não travar a GUI
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    try:
        start_gui(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()