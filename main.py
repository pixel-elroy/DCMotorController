import tkinter as tk
from tkinter import ttk
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

class DC_Motor_Simulator:
    def __init__(self, root):
        self.root = root
        self.root.title("DC Motor Closed-Loop Speed & Position Control Simulator")
        self.root.geometry("1350x900")

        # Handle clean exit when closing the window via 'X'
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

        # --- Motor Physical Parameters ---
        self.Ra = 2.0      # Armature Resistance (Ohms)
        self.Kt = 0.1      # Torque Constant (Nm/A)
        self.Kb = 0.1      # Back-EMF Constant (V/(rad/s))
        self.J = 0.001     # Rotor Inertia (kg*m^2)
        self.B = 0.0001    # Viscous Friction (Nm*s/rad)

        # --- UI Control Variables ---
        self.control_mode = tk.StringVar(value="Position")   
        self.controller_type = tk.StringVar(value="PID")     
        self.vin_var = tk.DoubleVar(value=2.5)               
        self.la_var = tk.DoubleVar(value=0.005)              
        self.tsim_var = tk.DoubleVar(value=2.0)              
        self.tl_var = tk.DoubleVar(value=0.0)                # Load Torque Variable (Nm)
        
        # Default gains
        self.kp_var = tk.DoubleVar(value=6.0)
        self.ki_var = tk.DoubleVar(value=4.0)
        self.kd_var = tk.DoubleVar(value=0.2)

        self.slider_labels = []
        self._update_job = None  # Used for smooth event debouncing

        self.create_widgets()
        self.run_simulation()

    def create_widgets(self):
        style = ttk.Style()
        style.theme_use('clam')
        
        control_frame = ttk.LabelFrame(self.root, text=" System & Controller Configuration ", padding=15)
        control_frame.pack(side=tk.LEFT, fill=tk.Y, padx=15, pady=15)

        plot_frame = ttk.Frame(self.root)
        plot_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=15, pady=15)

        # --- Control Mode Selection ---
        mode_frame = ttk.LabelFrame(control_frame, text="Control Mode", padding=10)
        mode_frame.pack(fill=tk.X, pady=5)
        for mode in ["Speed", "Position"]:
            ttk.Radiobutton(mode_frame, text=f"{mode} Control", variable=self.control_mode, value=mode, command=self.on_mode_change).pack(side=tk.LEFT, padx=15)

        # --- Controller Type Selection ---
        ctrl_frame = ttk.LabelFrame(control_frame, text="Controller Architecture", padding=10)
        ctrl_frame.pack(fill=tk.X, pady=8)
        for c_type in ["P", "PI", "PD", "PID"]:
            ttk.Radiobutton(ctrl_frame, text=c_type, variable=self.controller_type, value=c_type, command=self.run_simulation).pack(side=tk.LEFT, padx=12)

        # --- Parameters & Tuning Sliders Frame ---
        tune_frame = ttk.LabelFrame(control_frame, text="Plant & PID Tuning (Smooth Sliders)", padding=10)
        tune_frame.pack(fill=tk.X, pady=8)

        # Vin Slider with smooth debounced callback
        self.vin_label_text = tk.StringVar()
        self.update_vin_label()

        vin_container = ttk.Frame(tune_frame)
        vin_container.pack(fill=tk.X, pady=4)
        ttk.Label(vin_container, textvariable=self.vin_label_text, width=24, font=("Arial", 9)).pack(side=tk.LEFT)
        self.vin_val_lbl = ttk.Label(vin_container, width=8, anchor=tk.E, font=("Arial", 9, "bold"))
        self.vin_val_lbl.pack(side=tk.RIGHT, padx=5)
        self.vin_val_lbl.config(text=f"{self.vin_var.get():.2f} V")
        
        vin_scale = ttk.Scale(vin_container, from_=0.0, to=5.0, variable=self.vin_var, command=self.on_vin_change)
        vin_scale.pack(side=tk.RIGHT, fill=tk.X, expand=True)

        # Expanded Plant & Disturbance Sliders
        self.add_slider(tune_frame, "Armature Inductance (La)", self.la_var, 0.0001, 0.05, "%.4f H")
        self.add_slider(tune_frame, "Load Torque (TL)", self.tl_var, 0.0, 0.2, "%.3f Nm")
        self.add_slider(tune_frame, "Simulation Time (tsim)", self.tsim_var, 0.5, 10.0, "%.1f s")
        
        ttk.Separator(tune_frame, orient='horizontal').pack(fill='x', pady=8)

        self.add_slider(tune_frame, "Kp (Proportional)", self.kp_var, 0.0, 50.0, "%.2f")
        self.add_slider(tune_frame, "Ki (Integral)", self.ki_var, 0.0, 200.0, "%.2f")
        self.add_slider(tune_frame, "Kd (Derivative)", self.kd_var, 0.0, 2.0, "%.3f")

        # Action Buttons
        btn_frame = ttk.Frame(control_frame)
        btn_frame.pack(fill=tk.X, pady=15)
        ttk.Button(btn_frame, text="Reset / Update Simulation", command=self.run_simulation).pack(fill=tk.X, ipady=5)

        # --- Matplotlib 2x2 Subplots Setup ---
        self.fig, self.axs = plt.subplots(2, 2, figsize=(9, 7))
        self.fig.tight_layout(pad=3.0)

        self.canvas = FigureCanvasTkAgg(self.fig, master=plot_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def update_vin_label(self):
        if self.control_mode.get() == "Speed":
            self.vin_label_text.set("Input Vin (Max 300RPM)")
        else:
            self.vin_label_text.set("Input Vin (Max 2π rad)")

    def on_vin_change(self, val):
        self.vin_val_lbl.config(text=f"{float(val):.2f} V")
        self.schedule_simulation()

    def add_slider(self, parent, label_text, var, min_val, max_val, fmt_str):
        frame = ttk.Frame(parent)
        frame.pack(fill=tk.X, pady=4)
        
        ttk.Label(frame, text=label_text, width=24, font=("Arial", 9)).pack(side=tk.LEFT)
        
        val_label = ttk.Label(frame, width=8, anchor=tk.E, font=("Arial", 9, "bold"))
        val_label.pack(side=tk.RIGHT, padx=5)
        val_label.config(text=fmt_str % var.get())

        self.slider_labels.append((var, val_label, fmt_str))

        def on_scale_move(val):
            val_label.config(text=fmt_str % float(val))
            self.schedule_simulation()

        scale = ttk.Scale(frame, from_=min_val, to=max_val, variable=var, command=on_scale_move)
        scale.pack(side=tk.RIGHT, fill=tk.X, expand=True)

    def schedule_simulation(self):
        """Throttles simulation redraws during rapid dragging to keep sliders buttery smooth."""
        if self._update_job is not None:
            self.root.after_cancel(self._update_job)
        self._update_job = self.root.after(15, self.run_simulation)

    def update_all_slider_labels(self):
        for var, label, fmt_str in self.slider_labels:
            label.config(text=fmt_str % var.get())
        if hasattr(self, 'vin_val_lbl'):
            self.vin_val_lbl.config(text=f"{self.vin_var.get():.2f} V")

    def on_mode_change(self):
        self.update_vin_label()
        if self.control_mode.get() == "Position":
            self.kp_var.set(6.0)
            self.ki_var.set(4.0)
            self.kd_var.set(0.2)
        else:
            self.kp_var.set(1.2)
            self.ki_var.set(15.0)
            self.kd_var.set(0.02)
        
        self.update_all_slider_labels()
        self.run_simulation()

    def run_simulation(self):
        self._update_job = None
        Vin = self.vin_var.get()
        La = self.la_var.get()
        t_sim = self.tsim_var.get()
        T_L = self.tl_var.get()   # Retrieve load torque value
        dt = 0.0001  # 0.1 ms time step
        steps = int(t_sim / dt)

        Kp = self.kp_var.get()
        Ki = self.ki_var.get()
        Kd = self.kd_var.get()
        c_type = self.controller_type.get()
        mode = self.control_mode.get()

        if c_type == "P":
            Ki, Kd = 0.0, 0.0
        elif c_type == "PI":
            Kd = 0.0
        elif c_type == "PD":
            Ki = 0.0

        # --- State Variables Initialization ---
        I_a = 0.0      
        omega = 0.0    
        theta = 0.0    

        integral_error = 0.0
        prev_error = 0.0

        # Storage arrays
        t_arr = np.linspace(0, t_sim, steps)
        primary_actual_arr = np.zeros(steps)
        primary_ref_arr = np.zeros(steps)
        Va_arr = np.zeros(steps)
        Ia_arr = np.zeros(steps)
        omega_rpm_arr = np.zeros(steps)
        theta_arr = np.zeros(steps)

        # --- Simulation Loop ---
        for i in range(steps):
            omega_rpm = omega * (60.0 / (2.0 * np.pi))

            if mode == "Speed":
                ref_val = Vin * (300.0 / 5.0)
                actual_val = omega_rpm
                error = ref_val - actual_val
            else:
                ref_val = (Vin / 5.0) * (2.0 * np.pi)
                actual_val = theta
                error = ref_val - actual_val

            integral_error += error * dt
            derivative_error = (error - prev_error) / dt
            prev_error = error

            Va_raw = (Kp * error) + (Ki * integral_error) + (Kd * derivative_error)
            Va = np.clip(Va_raw, -5.0, 5.0)

            # Physics equations incorporating load torque T_L
            dI_dt = (Va - (self.Ra * I_a) - (self.Kb * omega)) / La
            dOmega_dt = ((self.Kt * I_a) - (self.B * omega) - T_L) / self.J
            dTheta_dt = omega

            I_a += dI_dt * dt
            omega += dOmega_dt * dt
            theta += dTheta_dt * dt

            primary_ref_arr[i] = ref_val
            primary_actual_arr[i] = actual_val
            omega_rpm_arr[i] = omega_rpm
            theta_arr[i] = theta
            Va_arr[i] = Va
            Ia_arr[i] = I_a

        for ax_row in self.axs:
            for ax in ax_row:
                ax.clear()

        if mode == "Speed":
            self.axs[0, 0].plot(t_arr, primary_ref_arr, 'r--', label="Setpoint RPM", linewidth=1.5)
            self.axs[0, 0].plot(t_arr, primary_actual_arr, 'b-', label="Actual RPM", linewidth=2)
            self.axs[0, 0].set_ylabel("Speed (RPM)")
            self.axs[0, 0].set_title(f"Speed Response ({c_type} Controller, TL={T_L:.3f} Nm)")
        else:
            self.axs[0, 0].plot(t_arr, primary_ref_arr, 'r--', label="Target Angle (2π max)", linewidth=1.5)
            self.axs[0, 0].plot(t_arr, primary_actual_arr, 'b-', label="Actual Angle", linewidth=2)
            self.axs[0, 0].set_ylabel("Angle (rad)")
            self.axs[0, 0].set_title(f"Position Response ({c_type} Controller, TL={T_L:.3f} Nm)")
        
        self.axs[0, 0].set_xlabel("Time (s)")
        self.axs[0, 0].legend(loc="lower right")
        self.axs[0, 0].grid(True)

        self.axs[0, 1].plot(t_arr, Va_arr, 'g-', linewidth=1.5)
        self.axs[0, 1].set_title("Armature Voltage (Va) [H-Bridge Output]")
        self.axs[0, 1].set_xlabel("Time (s)")
        self.axs[0, 1].set_ylabel("Voltage (V)")
        self.axs[0, 1].grid(True)

        self.axs[1, 0].plot(t_arr, Ia_arr, 'm-', linewidth=1.5)
        self.axs[1, 0].set_title("Armature Current (Ia) [Inrush & Load]")
        self.axs[1, 0].set_xlabel("Time (s)")
        self.axs[1, 0].set_ylabel("Current (A)")
        self.axs[1, 0].grid(True)

        if mode == "Speed":
            self.axs[1, 1].plot(t_arr, theta_arr, 'c-', linewidth=1.5)
            self.axs[1, 1].set_title("Shaft Position (theta)")
            self.axs[1, 1].set_ylabel("Angle (radians)")
        else:
            self.axs[1, 1].plot(t_arr, omega_rpm_arr, 'c-', linewidth=1.5)
            self.axs[1, 1].set_title("Motor Speed (Omega)")
            self.axs[1, 1].set_ylabel("Speed (RPM)")

        self.axs[1, 1].set_xlabel("Time (s)")
        self.axs[1, 1].grid(True)

        self.fig.tight_layout(pad=2.0)
        self.canvas.draw()

    def on_closing(self):
        plt.close('all')
        self.root.quit()
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = DC_Motor_Simulator(root)
    root.mainloop()
