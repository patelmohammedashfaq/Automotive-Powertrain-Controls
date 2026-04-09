import numpy as np

class TerrainResponseLogic:
    def __init__(self):
        # 1. Calibration & Maps
        self.MAP_PEDAL_BP = [0, 10, 25, 50, 75, 100]
        self.MAP_NORMAL_TRQ = [0, 45, 112, 225, 337, 450]
        self.MAP_SAND_TRQ   = [0, 120, 280, 400, 440, 450]
        self.MAP_ROCK_TRQ   = [0, 15, 40, 120, 300, 450]

        # 2. Buffers for Signal Processing
        self.speed_buffer = [] # Stores last 10 wheel speed samples
        self.buffer_size = 10
        
        # 3. State Management
        self.auto_detected_mode = 1 
        self.switch_counter = 0
        self.K_HYSTERESIS = 3 # Reduced from 20 to 3 for faster response 

    def calculate_jitter(self, v_wheel):
        """
        Derives 'Roughness' from Wheel Speed Noise (REQ-401)
        High std dev = Rock/Uneven terrain
        """
        self.speed_buffer.append(v_wheel)
        if len(self.speed_buffer) > self.buffer_size:
            self.speed_buffer.pop(0)
        
        if len(self.speed_buffer) < self.buffer_size:
            return 0.0
            
        # Calculate standard deviation of speed
        jitter = np.std(self.speed_buffer)
        return jitter

    def _observer_logic(self, slip, jitter):
        # Logic thresholds
        if jitter > 0.2: # Jitter Threshold: Lowered it from 1.5 (or 0.4) to 0.2
            return 3     # Rock Crawl
        elif slip > 0.05: #Slip Threshold: Lowered it from 0.18 to 0.05 (since our stabilized wheel is very well-behaved now)
            return 2     # Sand
        else:
            return 1     # Normal

    def run_logic(self, pedal_pct, driver_select, v_wheel, v_veh):
        # 1. Internal Physics Calculations
        v_ref = max(v_veh, 0.5)
        current_slip = np.clip((v_wheel - v_veh) / v_ref, 0, 1.0)
        current_jitter = self.calculate_jitter(v_wheel)

        # 2. Mode Arbitration
        if driver_select == 0:
            target_mode = self._observer_logic(current_slip, current_jitter)
            if target_mode != self.auto_detected_mode:
                self.switch_counter += 1
                if self.switch_counter > self.K_HYSTERESIS:
                    self.auto_detected_mode = target_mode
                    self.switch_counter = 0 # Optional: Reset after switching to start fresh
            else:
                self.switch_counter = 0
            active_mode = self.auto_detected_mode
        else:
            active_mode = driver_select

        # 3. Final Torque Mapping
        maps = {1: self.MAP_NORMAL_TRQ, 2: self.MAP_SAND_TRQ, 3: self.MAP_ROCK_TRQ}
        torque_req = np.interp(pedal_pct, self.MAP_PEDAL_BP, maps[active_mode])
        
        return torque_req, active_mode, current_slip, current_jitter