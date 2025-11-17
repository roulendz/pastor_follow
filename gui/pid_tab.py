import customtkinter as ctk
import tkinter as tk


class PIDTabView:
    """Encapsulates the PID Control tab UI and interactions.

    This class builds the widgets inside a provided parent frame and exposes
    getters/setters so the hosting app can read values or update labels without
    depending on widget internals. Callbacks passed in the constructor are
    wired to widget commands so existing app logic can be reused.
    """

    def __init__(
        self,
        parent: ctk.CTkFrame,
        config,
        *,
        on_pid_change,
        on_apply_motor_settings,
        on_deadband_change,
        on_min_interval_change,
        on_invert_direction_change,
        on_toggle_adaptive_pid,
    ):
        self.parent = parent
        self.config = config

        # PID Gains
        gains_frame = ctk.CTkFrame(self.parent)
        gains_frame.pack(fill="x", padx=10, pady=10)

        ctk.CTkLabel(gains_frame, text="PID Gains", font=("Arial", 14, "bold")).pack()

        # P Gain
        p_frame = ctk.CTkFrame(gains_frame)
        p_frame.pack(fill="x", pady=5)
        ctk.CTkLabel(p_frame, text="P:", width=30).pack(side="left", padx=5)
        self.p_slider = ctk.CTkSlider(
            p_frame, from_=0, to=10, number_of_steps=100,
            command=on_pid_change,
        )
        self.p_slider.set(self.config.get('pid', 'kp'))
        self.p_slider.pack(side="left", fill="x", expand=True, padx=5)
        self.p_value = ctk.CTkLabel(p_frame, text=f"{self.config.get('pid', 'kp'):.2f}", width=50)
        self.p_value.pack(side="left", padx=5)

        # I Gain
        i_frame = ctk.CTkFrame(gains_frame)
        i_frame.pack(fill="x", pady=5)
        ctk.CTkLabel(i_frame, text="I:", width=30).pack(side="left", padx=5)
        self.i_slider = ctk.CTkSlider(
            i_frame, from_=0, to=2, number_of_steps=100,
            command=on_pid_change,
        )
        self.i_slider.set(self.config.get('pid', 'ki'))
        self.i_slider.pack(side="left", fill="x", expand=True, padx=5)
        self.i_value = ctk.CTkLabel(i_frame, text=f"{self.config.get('pid', 'ki'):.3f}", width=50)
        self.i_value.pack(side="left", padx=5)

        # D Gain
        d_frame = ctk.CTkFrame(gains_frame)
        d_frame.pack(fill="x", pady=5)
        ctk.CTkLabel(d_frame, text="D:", width=30).pack(side="left", padx=5)
        self.d_slider = ctk.CTkSlider(
            d_frame, from_=0, to=5, number_of_steps=100,
            command=on_pid_change,
        )
        self.d_slider.set(self.config.get('pid', 'kd'))
        self.d_slider.pack(side="left", fill="x", expand=True, padx=5)
        self.d_value = ctk.CTkLabel(d_frame, text=f"{self.config.get('pid', 'kd'):.2f}", width=50)
        self.d_value.pack(side="left", padx=5)

        # Motor Settings
        motor_frame = ctk.CTkFrame(self.parent)
        motor_frame.pack(fill="x", padx=10, pady=10)

        ctk.CTkLabel(motor_frame, text="Motor Settings", font=("Arial", 14, "bold")).pack()

        # Max Speed
        speed_frame = ctk.CTkFrame(motor_frame)
        speed_frame.pack(fill="x", pady=5)
        ctk.CTkLabel(speed_frame, text="Max Speed:", width=100).pack(side="left", padx=5)
        self.speed_entry = ctk.CTkEntry(speed_frame, width=100)
        self.speed_entry.insert(0, str(self.config.get('arduino', 'max_speed')))
        self.speed_entry.pack(side="left", padx=5)

        # Max Acceleration
        accel_frame = ctk.CTkFrame(motor_frame)
        accel_frame.pack(fill="x", pady=5)
        ctk.CTkLabel(accel_frame, text="Max Accel:", width=100).pack(side="left", padx=5)
        self.accel_entry = ctk.CTkEntry(accel_frame, width=100)
        self.accel_entry.insert(0, str(self.config.get('arduino', 'max_acceleration')))
        self.accel_entry.pack(side="left", padx=5)

        # Apply button
        self.apply_motor_btn = ctk.CTkButton(
            motor_frame, text="Apply Motor Settings",
            command=on_apply_motor_settings,
        )
        self.apply_motor_btn.pack(pady=10)

        # Invert direction checkbox
        try:
            invert_default = bool(self.config.get('arduino', 'invert_direction'))
        except Exception:
            invert_default = False
        self.invert_dir_var = tk.BooleanVar(value=invert_default)
        self.invert_dir_check = ctk.CTkCheckBox(
            motor_frame, text="Invert Direction", variable=self.invert_dir_var,
            command=on_invert_direction_change,
        )
        self.invert_dir_check.pack(pady=5)

        # Command throttling (Deadband & Min Interval)
        throttle_frame = ctk.CTkFrame(self.parent)
        throttle_frame.pack(fill="x", padx=10, pady=10)
        ctk.CTkLabel(throttle_frame, text="Command Throttling", font=("Arial", 14, "bold")).pack()

        # Deadband slider
        db_frame = ctk.CTkFrame(throttle_frame)
        db_frame.pack(fill="x", pady=5)
        ctk.CTkLabel(db_frame, text="Deadband (deg):", width=140).pack(side="left", padx=5)
        self.deadband_slider = ctk.CTkSlider(
            db_frame, from_=0.0, to=0.50, number_of_steps=50,
            command=on_deadband_change,
        )
        self.deadband_slider.set(self.config.get('arduino', 'command_deadband_deg'))
        self.deadband_value = ctk.CTkLabel(db_frame, text=f"{self.config.get('arduino', 'command_deadband_deg'):.3f}")
        self.deadband_slider.pack(side="left", fill="x", expand=True, padx=5)
        self.deadband_value.pack(side="left", padx=5)

        # Min command interval slider
        mi_frame = ctk.CTkFrame(throttle_frame)
        mi_frame.pack(fill="x", pady=5)
        ctk.CTkLabel(mi_frame, text="Min Cmd Interval (ms):", width=180).pack(side="left", padx=5)
        self.min_interval_slider = ctk.CTkSlider(
            mi_frame, from_=10, to=100, number_of_steps=90,
            command=on_min_interval_change,
        )
        self.min_interval_slider.set(self.config.get('arduino', 'min_command_interval_ms'))
        self.min_interval_value = ctk.CTkLabel(mi_frame, text=f"{self.config.get('arduino', 'min_command_interval_ms'):.0f}")
        self.min_interval_slider.pack(side="left", fill="x", expand=True, padx=5)
        self.min_interval_value.pack(side="left", padx=5)

        # Auto-tune options
        autotune_frame = ctk.CTkFrame(self.parent)
        autotune_frame.pack(fill="x", padx=10, pady=10)

        ctk.CTkLabel(autotune_frame, text="Auto-Tuning", font=("Arial", 14, "bold")).pack()

        self.adaptive_var = tk.BooleanVar(value=False)
        self.adaptive_check = ctk.CTkCheckBox(
            autotune_frame, text="Enable Adaptive PID",
            variable=self.adaptive_var,
            command=on_toggle_adaptive_pid,
        )
        self.adaptive_check.pack(pady=5)

    # --- Interface helpers for the host app ---
    def get_pid_values(self):
        return float(self.p_slider.get()), float(self.i_slider.get()), float(self.d_slider.get())

    def set_pid_labels(self, kp: float, ki: float, kd: float):
        self.p_value.configure(text=f"{kp:.2f}")
        self.i_value.configure(text=f"{ki:.3f}")
        self.d_value.configure(text=f"{kd:.2f}")

    def get_motor_settings(self):
        return float(self.speed_entry.get()), float(self.accel_entry.get())

    def set_deadband_label(self, db: float):
        self.deadband_value.configure(text=f"{db:.3f}")

    def set_min_interval_label(self, mi: float):
        self.min_interval_value.configure(text=f"{mi:.0f}")

    def get_invert_dir_value(self) -> bool:
        return bool(self.invert_dir_var.get())

    def get_adaptive_enabled(self) -> bool:
        return bool(self.adaptive_var.get())