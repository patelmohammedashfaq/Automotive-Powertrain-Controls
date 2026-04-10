import numpy as np

class CTRL_Traction_Logic:
    def __init__(self):
        # 1. Calibration Parameters (MAAB & DNA Standard)
        self.K_DT = 0.01               # 100Hz Loop Time
        self.K_SLIP_TARGET = 0.12      # Aim for 12% slip for peak grip
        self.K_P_GAIN = 600.0          # Proportional gain for torque reduction
        self.K_MAX_TRQ = 450.0         # Hardware limit (Nm)
        
    def step(self, in_v_veh, in_w_wheel, in_driver_trq_req):
        """Main execution loop called every 10ms."""
        
        # 1. Protection against division by zero (Portfolio DNA)
        v_ref = max(in_v_veh, 0.5)
        
        # 2. Sensing: Calculate current slip ratio
        current_slip = (in_w_wheel * 0.35 - in_v_veh) / v_ref
        
        # 3. Decision: Is slip above target?
        slip_error = current_slip - self.K_SLIP_TARGET
        
        if slip_error > 0:
            # INTERVENTION: Calculate proportional torque reduction
            reduction = slip_error * self.K_P_GAIN
            out_trq_motor = in_driver_trq_req - reduction
        else:
            # NORMAL: Follow driver intent
            out_trq_motor = in_driver_trq_req
            
        # 4. Actuator Protection: Clip to hardware limits (Portfolio DNA)
        out_trq_motor = np.clip(out_trq_motor, 0.0, self.K_MAX_TRQ)
        
        return out_trq_motor, current_slip