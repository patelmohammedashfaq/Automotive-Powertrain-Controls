import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import os
import sys

# Ensure the controller can be imported
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '03_Controller')))
from CTRL_Traction_Logic import CTRL_Traction_Logic

class Mock_PLNT_Traction:
    """Mocking the Modelica FMU physics for instantaneous MIL execution"""
    def __init__(self):
        self.mass, self.r_w, self.J_wheel = 2200.0, 0.35, 15.0
        self.v_veh, self.w_wheel = 10.0, 28.5 # Initial states
        
    def do_step(self, in_trq_motor, in_mu_ground, dt):
        out_slip = (self.w_wheel * self.r_w - self.v_veh) / max(self.v_veh, 0.5)
        F_tractive = in_mu_ground * self.mass * 9.81 * np.tanh(10 * out_slip)
        dv = (F_tractive - (0.5 * 1.225 * 0.38 * 2.8 * self.v_veh**2)) / self.mass
        dw = (in_trq_motor - (F_tractive * self.r_w)) / self.J_wheel
        self.v_veh += dv * dt
        self.w_wheel += dw * dt
        return self.v_veh, self.w_wheel, out_slip

def run_extensive_mil():
    # 1. Configuration
    dt = 0.01
    sim_duration = 3.0  # seconds
    os.makedirs('05_Verification', exist_ok=True)

    # 2. Define the Test Grid (The "All Scenarios" Matrix)
    mu_range = [0.02, 0.05, 0.1, 0.2, 0.4, 0.6, 0.8, 0.9] # From Black Ice to Tarmac
    torque_range = [50, 150, 250, 350, 450]              # From light to max pedal
    
    batch_results = []

    print(f"Starting Batch MIL Verification: {len(mu_range) * len(torque_range)} Scenarios...")

    # 3. Execution Loop
    for mu in mu_range:
        for req_trq in torque_range:
            # Re-initialize for every test case
            plant = Mock_PLNT_Traction()
            ctrl = CTRL_Traction_Logic()
            
            max_slip = 0.0
            min_trq_observed = req_trq
            intervention_active = False

            for _ in range(int(sim_duration/dt)):
                # Note: first step trq is 0, subsequent are from ctrl
                curr_trq = 0.0 if _ == 0 else out_trq
                v, w, slip = plant.do_step(curr_trq, mu, dt)
                out_trq, calc_slip = ctrl.step(v, w, req_trq)
                
                # Metrics for verdict
                max_slip = max(max_slip, calc_slip)
                min_trq_observed = min(min_trq_observed, out_trq)
                if out_trq < (req_trq - 5.0): # 5Nm threshold for "intervention"
                    intervention_active = True

            # 4. Automated Verdict Logic (ASPICE Standard)
            # If slip > target (0.12), the controller MUST have reduced torque.
            # If slip < target, the torque should have remained equal to request.
            if max_slip > 0.13: # 0.13 includes a small tolerance
                verdict = "PASS" if intervention_active else "FAIL"
            else:
                verdict = "PASS" # No intervention needed

            batch_results.append({
                "Friction_Mu": mu,
                "Request_Nm": req_trq,
                "Max_Slip": round(max_slip, 3),
                "Min_Torque_Output": round(min_trq_observed, 1),
                "Intervened": intervention_active,
                "Verdict": verdict
            })

    # 5. Data Processing & Visualization
    df = pd.DataFrame(batch_results)
    df.to_csv('05_Verification/Extensive_MIL_Results.csv', index=False)

    # Heatmap of Peak Slip (Shows how well the controller "caps" slip near 0.12)
    plt.figure(figsize=(10, 6))
    pivot_table = df.pivot(index="Friction_Mu", columns="Request_Nm", values="Max_Slip")
    sns.heatmap(pivot_table, annot=True, cmap="YlOrRd", cbar_kws={'label': 'Peak Slip Ratio'})
    plt.title("Traction Control Operating Envelope: Peak Slip Ratio\n(Target: Close to 0.12 in high-torque/low-mu zones)")
    plt.savefig('05_Verification/PLOT_Batch_Heatmap.png')
    
    # Success Rate calculation
    success_rate = (df['Verdict'] == 'PASS').mean() * 100
    print(f"Batch Testing Complete. Success Rate: {success_rate}%")
    print("Files saved to '05_Verification/'.")

if __name__ == "__main__":
    run_extensive_mil()