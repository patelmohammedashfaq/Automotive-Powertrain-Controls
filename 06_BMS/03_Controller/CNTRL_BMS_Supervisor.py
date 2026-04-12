import numpy as np

class BMS_Controller:
    def __init__(self, dt=0.01):
        # Constants as per SOP
        self.K_DT = dt  # 10ms loop time 
        self.K_CAP_AH = 100.0
        self.K_DEBOUNCE_LIMIT = 0.5 / self.K_DT  # 500ms counter
        self.K_HYST_LIMIT = 2.0 / self.K_DT      # 2.0s counter
        self.K_EPSILON = 0.1                     # Plausibility threshold

        # Internal States
        self.soc = 0.8  # Initial SoC (80%)
        self.fault_latched = False
        self.debounce_counter = 0
        self.recovery_counter = 0

    def run_step(self, In_CellV, In_PackV, In_Current, In_Temp):
        """
        Main execution step for BMS Supervisor logic.
        """
        # --- BMS_REQ_01: SoC Estimation (Coulomb Counting) ---
        # Integration: SoC = SoC + (I/Cap) * dt
        self.soc += (In_Current / (self.K_CAP_AH * 3600.0)) * self.K_DT
        self.soc = np.clip(self.soc, 0.0, 1.0) # Saturation protection 

        # --- Violation Detection (REQ_02, 03, 04, 05) ---
        has_violation = False
        
        # Over/Under Voltage check
        if max(In_CellV) > 4.25 or min(In_CellV) < 2.50:
            has_violation = True
        
        # Over Temp check
        if In_Temp > 60.0:
            has_violation = True

        # Over Current check (REQ_05)
        if In_Current > 350.0:
            has_violation = True

        # --- BMS_REQ_07: Plausibility Check ---
        if abs(In_PackV - sum(In_CellV)) > self.K_EPSILON:
            has_violation = True

        # --- BMS_REQ_06: Fault Debouncing Logic ---
        if has_violation:
            self.debounce_counter += 1
            self.recovery_counter = 0 # Reset recovery if violation exists
        else:
            self.debounce_counter = 0
            # --- BMS_REQ_09: Fault Hysteresis (Recovery) ---
            if self.fault_latched:
                self.recovery_counter += 1

        # Latch fault if debounce limit reached
        if self.debounce_counter >= self.K_DEBOUNCE_LIMIT:
            self.fault_latched = True
        
        # Clear fault if recovery limit reached
        if self.recovery_counter >= self.K_HYST_LIMIT:
            self.fault_latched = False

        # --- BMS_REQ_08: Safe-State Logic ---
        Out_BMS_Status = 0 if self.fault_latched else 1
        
        return Out_BMS_Status, self.soc