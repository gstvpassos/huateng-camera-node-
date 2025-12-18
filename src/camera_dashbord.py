import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import tkinter as tk
from tkinter import ttk
from PIL import Image as PILImage, ImageTk
import threading
import time

class CameraDashboard(Node):
    def __init__(self):
        super().__init__('camera_dashboard')

        # --- CONFIGURAÇÕES ---
        self.bridge = CvBridge()
        self.latest_cv_image = None
        self.lock = threading.Lock()
        
        # Estado Visual
        self.roi_top_pct = 0.35
        self.roi_bot_pct = 0.17
        self.fps = 0
        self.last_time = time.time()
        self.frame_count = 0

        # --- PUBLISHERS ---
        self.pub_contrast = self.create_publisher(Float32, '/set_contrast', 10)
        self.pub_gamma = self.create_publisher(Float32, '/set_gamma', 10)
        self.pub_black = self.create_publisher(Float32, '/set_black_level', 10)
        self.pub_sharp = self.create_publisher(Float32, '/set_sharpness', 10)
        self.pub_exposure = self.create_publisher(Float32, '/set_exposure', 10)
        self.pub_auto_exp = self.create_publisher(Bool, '/set_auto_exposure', 10)
        self.pub_roi_top = self.create_publisher(Float32, '/set_ae_roi_top_skip', 10)
        self.pub_roi_bot = self.create_publisher(Float32, '/set_ae_roi_bottom_skip', 10)
        self.pub_wb_auto = self.create_publisher(Bool, '/set_white_balance_auto', 10)
        self.pub_gain_r = self.create_publisher(Float32, '/set_gain_r', 10)
        self.pub_gain_g = self.create_publisher(Float32, '/set_gain_g', 10)
        self.pub_gain_b = self.create_publisher(Float32, '/set_gain_b', 10)

        # QoS "Sensor Data" para evitar lag de buffer
        from rclpy.qos import qos_profile_sensor_data
        self.sub_image = self.create_subscription(
            Image, '/image_raw', self.image_callback, qos_profile_sensor_data)

        self.get_logger().info("Dashboard V2 Iniciado!")

    def image_callback(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Cálculo FPS
            self.frame_count += 1
            now = time.time()
            if now - self.last_time >= 1.0:
                self.fps = self.frame_count
                self.frame_count = 0
                self.last_time = now
            
            with self.lock:
                self.latest_cv_image = cv_img
        except Exception as e:
            self.get_logger().error(f"Erro img: {e}")

    def send_float(self, pub, val):
        msg = Float32(); msg.data = float(val); pub.publish(msg)
    def send_bool(self, pub, val):
        msg = Bool(); msg.data = bool(val); pub.publish(msg)

# --- WIDGETS ---
class SmartSlider:
    def __init__(self, parent, label_text, min_val, max_val, default_val, callback):
        self.callback = callback
        self.min_val = min_val; self.max_val = max_val
        
        self.frame = ttk.Frame(parent)
        self.frame.pack(fill="x", padx=5, pady=2)
        
        top = ttk.Frame(self.frame)
        top.pack(fill="x")
        ttk.Label(top, text=label_text, font=("Arial", 9, "bold")).pack(side="left")
        
        self.entry_var = tk.StringVar(value=str(default_val))
        self.entry = ttk.Entry(top, textvariable=self.entry_var, width=6, justify="right")
        self.entry.pack(side="right")
        self.entry.bind('<Return>', self.on_enter); self.entry.bind('<FocusOut>', self.on_enter)
        
        self.scale = ttk.Scale(self.frame, from_=min_val, to=max_val, command=self.on_move)
        self.scale.set(default_val)
        self.scale.pack(fill="x")

    def on_move(self, val):
        v = float(val)
        self.entry_var.set(f"{int(v) if v > 10 else v:.2f}")
        self.callback(v)

    def on_enter(self, event):
        try:
            val = float(self.entry_var.get())
            if val < self.min_val: val = self.min_val
            if val > self.max_val: val = self.max_val
            self.scale.set(val); self.callback(val)
        except ValueError: pass

    def toggle(self, state):
        self.scale.configure(state=state); self.entry.configure(state=state)

def create_slider(parent, txt, min_v, max_v, def_v, cb):
    return SmartSlider(parent, txt, min_v, max_v, def_v, cb)

# --- APP PRINCIPAL ---
class DashboardApp:
    def __init__(self, node):
        self.node = node
        self.root = tk.Tk()
        self.root.title("LKAS Camera Tuning V2")
        self.root.geometry("1200x800")
        
        # === LAYOUT NOVO ===
        # Lado Esquerdo: VÍDEO (Expande e ocupa tudo)
        self.video_container = tk.Frame(self.root, bg="black")
        self.video_container.pack(side="left", fill="both", expand=True)
        
        self.lbl_video = tk.Label(self.video_container, bg="black", text="Aguardando Câmera...", fg="white")
        self.lbl_video.pack(fill="both", expand=True) # Label preenche o container

        # Lado Direito: CONTROLES (Largura fixa)
        self.sidebar = tk.Frame(self.root, width=320, bg="#f0f0f0")
        self.sidebar.pack(side="right", fill="y")
        self.sidebar.pack_propagate(False) # Força largura fixa
        
        # Scrollbar para Sidebar
        canvas = tk.Canvas(self.sidebar, bg="#f0f0f0")
        scrollbar = ttk.Scrollbar(self.sidebar, orient="vertical", command=canvas.yview)
        self.scroll_frame = ttk.Frame(canvas)

        self.scroll_frame.bind("<Configure>", lambda e: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.create_window((0, 0), window=self.scroll_frame, anchor="nw", width=300) # Width ajustado ao sidebar
        canvas.configure(yscrollcommand=scrollbar.set)

        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

        self.setup_controls(self.scroll_frame)
        self.update_video_loop()

    def setup_controls(self, parent):
        pad_opts = {'padx': 5, 'pady': 5, 'fill': 'x'}
        
        # 1. ISP
        lf1 = ttk.LabelFrame(parent, text=" 1. Imagem (ISP) ")
        lf1.pack(**pad_opts)
        create_slider(lf1, "Contraste", 0, 200, 100, lambda v: self.node.send_float(self.node.pub_contrast, v))
        create_slider(lf1, "Gamma", 0, 255, 120, lambda v: self.node.send_float(self.node.pub_gamma, v))
        create_slider(lf1, "Black Level", 0, 50, 5, lambda v: self.node.send_float(self.node.pub_black, v))
        create_slider(lf1, "Nitidez", 0, 100, 50, lambda v: self.node.send_float(self.node.pub_sharp, v))

        # 2. Cor
        lf2 = ttk.LabelFrame(parent, text=" 2. Cor (WB) ")
        lf2.pack(**pad_opts)
        wb_var = tk.BooleanVar(value=True)
        def toggle_wb():
            val = wb_var.get()
            self.node.send_bool(self.node.pub_wb_auto, val)
            st = "disabled" if val else "normal"
            s_r.toggle(st); s_g.toggle(st); s_b.toggle(st)
            
        ttk.Checkbutton(lf2, text="Auto White Balance", variable=wb_var, command=toggle_wb).pack(pady=5)
        s_r = create_slider(lf2, "Red", 0, 400, 120, lambda v: self.node.send_float(self.node.pub_gain_r, v))
        s_g = create_slider(lf2, "Green", 0, 400, 100, lambda v: self.node.send_float(self.node.pub_gain_g, v))
        s_b = create_slider(lf2, "Blue", 0, 400, 80, lambda v: self.node.send_float(self.node.pub_gain_b, v))
        toggle_wb()

        # 3. ROI
        lf3 = ttk.LabelFrame(parent, text=" 3. ROI (Área Ignorada) ")
        lf3.pack(**pad_opts)
        def upd_top(v): self.node.roi_top_pct = float(v)/100; self.node.send_float(self.node.pub_roi_top, float(v)/100)
        def upd_bot(v): self.node.roi_bot_pct = float(v)/100; self.node.send_float(self.node.pub_roi_bot, float(v)/100)
        create_slider(lf3, "Topo (Céu) %", 0, 80, 35, upd_top)
        create_slider(lf3, "Fundo (Capô) %", 0, 50, 17, upd_bot)

        # 4. Exposição
        lf4 = ttk.LabelFrame(parent, text=" 4. Exposição ")
        lf4.pack(**pad_opts)
        ae_var = tk.BooleanVar(value=True)
        def toggle_ae():
            val = ae_var.get()
            self.node.send_bool(self.node.pub_auto_exp, val)
            s_exp.toggle("disabled" if val else "normal")
            
        ttk.Checkbutton(lf4, text="Auto Exposição", variable=ae_var, command=toggle_ae).pack(pady=5)
        s_exp = create_slider(lf4, "Manual (µs)", 100, 50000, 10000, lambda v: self.node.send_float(self.node.pub_exposure, v))
        toggle_ae()

    def update_video_loop(self):
        img = None
        with self.node.lock:
            if self.node.latest_cv_image is not None:
                img = self.node.latest_cv_image.copy()
        
        if img is not None:
            # 1. Obter tamanho atual do container (para encher a tela)
            win_w = self.video_container.winfo_width()
            win_h = self.video_container.winfo_height()
            
            if win_w > 1 and win_h > 1: # Só desenha se a janela já iniciou
                # 2. Resizing Inteligente (Manter Aspect Ratio)
                h, w = img.shape[:2]
                aspect_ratio = w / h
                
                # Tenta encaixar na largura
                new_w = win_w
                new_h = int(new_w / aspect_ratio)
                
                # Se ficou muito alto, encaixa na altura
                if new_h > win_h:
                    new_h = win_h
                    new_w = int(new_h * aspect_ratio)
                
                img_resized = cv2.resize(img, (new_w, new_h))
                
                # 3. Desenhar ROI e FPS
                # ROI Top
                y_top = int(new_h * self.node.roi_top_pct)
                cv2.rectangle(img_resized, (0, 0), (new_w, y_top), (0,0,255), -1) # Vermelho solido (OpenCV não tem alpha direto no rectangle simples, mas serve)
                
                # ROI Bot
                y_bot = int(new_h * (1.0 - self.node.roi_bot_pct))
                cv2.rectangle(img_resized, (0, y_bot), (new_w, new_h), (0,0,255), -1)
                
                # Se quiser "transparência", teria que usar addWeighted como no anterior, 
                # mas o solid rectangle mostra bem onde está cortando.
                
                cv2.putText(img_resized, f"FPS: {self.node.fps}", (20, 50), 
                           cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 2)
                
                # 4. Converter e Mostrar
                img_rgb = cv2.cvtColor(img_resized, cv2.COLOR_BGR2RGB)
                pil_img = PILImage.fromarray(img_rgb)
                tk_img = ImageTk.PhotoImage(image=pil_img)
                
                self.lbl_video.configure(image=tk_img)
                self.lbl_video.image = tk_img
        
        self.root.after(30, self.update_video_loop)

    def run(self):
        self.root.mainloop()

def main():
    rclpy.init()
    node = CameraDashboard()
    t = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    t.start()
    try: DashboardApp(node).run()
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()