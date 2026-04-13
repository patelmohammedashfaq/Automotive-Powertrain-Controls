import numpy as np

class EnergyManager:
    def __init__(self):
        # --- Calibration: K_ Constants [cite: 81] ---
        self.K_DT = 0.01             # 10ms Task Rate [cite: 86]
        self.K_SOC_TARGET_LOW = 0.25 # Lower bound for Charge-Sustaining
        self.K_SOC_TARGET_HIGH = 0.75# Upper bound for EV Mode
        self.K_EV_ONLY_SPD_MPS = 8.33# 30 kph threshold
        self.K_MAX_ENG_TRQ = 400.0   # Actuator Limit [cite: 89]
        self.K_MAX_MOT_TRQ = 200.0   # Actuator Limit
        self.K_HYSTERESIS = 5        # Debounce counter [cite: 88]

        # --- Runtime States ---
        self.mode_switch_counter = 0
        self.current_mode = "EV_PRIORITY" # Initial State

    def run_step(self, in_SoC, in_v_veh, in_TrqReq_Nm):
        """
        Main arbitration loop following the 'Observer-Action' pattern[cite: 39].
        """
        # 1. State Management (Hysteresis & Debouncing) [cite: 58, 88]
        if in_SoC < self.K_SOC_TARGET_LOW:
            self.mode_switch_counter += 1
            if self.mode_switch_counter >= self.K_HYSTERESIS:
                self.current_mode = "CHARGE_SUSTAIN"
        else:
            self.mode_switch_counter = 0
            if in_v_veh < self.K_EV_ONLY_SPD_MPS:
                self.current_mode = "EV_PRIORITY"
            else:
                self.current_mode = "HYBRID_PARALLEL"

        # 2. Logic: Mode-Specific Torque Split
        out_EngTrq_Nm = 0.0
        out_MotTrq_Nm = 0.0

        if self.current_mode == "CHARGE_SUSTAIN":
            # ICE handles demand + extra to charge battery
            out_EngTrq_Nm = in_TrqReq_Nm + 30.0
            out_MotTrq_Nm = -30.0 # Generating Mode
        
        elif self.current_mode == "EV_PRIORITY":
            out_EngTrq_Nm = 0.0
            out_MotTrq_Nm = in_TrqReq_Nm
            
        else: # HYBRID_PARALLEL
            out_EngTrq_Nm = in_TrqReq_Nm * 0.6
            out_MotTrq_Nm = in_TrqReq_Nm * 0.4

        # 3. Defensive Logic: Actuator Saturation [cite: 57, 89]
        # Prevents "wind-up" or physical over-torque
        out_EngTrq_Nm = np.clip(out_EngTrq_Nm, 0, self.K_MAX_ENG_TRQ)
        out_MotTrq_Nm = np.clip(out_MotTrq_Nm, -self.K_MAX_MOT_TRQ, self.K_MAX_MOT_TRQ)

        return out_EngTrq_Nm, out_MotTrq_Nm

    def _division_protection(self, val):
        # Internal helper for division-by-zero protection [cite: 56, 90]
        return max(val, 0.001)