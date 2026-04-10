import numpy as np
import matplotlib.pyplot as plt
import os
import sys

# Ensure the controller can be imported from the adjacent folder
# Ensure the controller can be imported from the adjacent folder
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '03_Controller')))
from CTRL_Traction_Logic import CTRL_Traction_Logic

# Note: In a real PyFMI environment, you would use FMU2Slave here. 
# For this script, we will use a purely Python-based numerical solver 
# to mimic the Modelica plant equations so you can run and plot it instantly.

class Mock_PLNT_Traction:
    """Mocking the Modelica FMU physics for instantaneous MIL execution"""
    def __init__(self):
        self.mass = 2200.0
        self.r_w = 0.35
        self.J_wheel = 15.0
        self.Cd = 0.38
        self.Af = 2.8
        self.rho = 1.225
        self.v_veh = 10.0
        self.w_wheel = 28.5  # 10.0 / 0.35 approx
        
    def do_step(self, in_trq_motor, in_mu_ground, dt):
        out_slip = (self.w_wheel * self.r_w - self.v_veh) / max(self.v_veh, 0.5)
        F_tractive = in_mu_ground * self.mass * 9.81 * np.tanh(10 * out_slip)
        
        # Derivatives
        dv = (F_tractive - (0.5 * self.rho * self.Cd * self.Af * self.v_veh**2)) / self.mass
        dw = (in_trq_motor - (F_tractive * self.r_w)) / self.J_wheel
        
        # Euler Integration
        self.v_veh += dv * dt
        self.w_wheel += dw * dt
        return self.v_veh, self.w_wheel, out_slip

def run_mil_simulation():
    # 1. Initialize System
    dt = 0.01
    time = np.arange(0, 5.0, dt)
    
    plant = Mock_PLNT_Traction()
    ctrl = CTRL_Traction_Logic()
    
    # 2. Data Logging Arrays
    log_v_veh = []
    log_v_wheel = [] # Equivalent wheel linear speed (w * r)
    log_slip = []
    log_trq_demand = []
    log_trq_motor = []
    log_mu = []

    # 3. Execution Loop
    out_trq_motor = 0.0 # Initial state
    
    for t in time:
        # SCENARIO: Heavy acceleration demand from driver
        in_driver_trq_req = 400.0 
        
        # SCENARIO: Surface transition (Tarmac -> Ice at t=2.0s)
        in_mu_ground = 0.9 if t < 2.0 else 0.1
        
        # Step Plant
        v_veh, w_wheel, current_slip = plant.do_step(out_trq_motor, in_mu_ground, dt)
        
        # Step Controller
        out_trq_motor, calc_slip = ctrl.step(v_veh, w_wheel, in_driver_trq_req)
        
        # Log Data
        log_v_veh.append(v_veh)
        log_v_wheel.append(w_wheel * plant.r_w)
        log_slip.append(calc_slip)
        log_trq_demand.append(in_driver_trq_req)
        log_trq_motor.append(out_trq_motor)
        log_mu.append(in_mu_ground)

    # 4. Generate JLR Verification Plot
    os.makedirs('05_Verification', exist_ok=True)
    
    fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    fig.suptitle('MIL Verification: Terrain-Adaptive Traction Control (Ice Patch Scenario)', fontsize=14)

    # Subplot 1: Torque Arbitration
    axs[0].plot(time, log_trq_demand, 'k--', label='Driver Demand (Nm)')
    axs[0].plot(time, log_trq_motor, 'b-', label='Actual Motor Torque (Nm)')
    axs[0].set_ylabel('Torque [Nm]')
    axs[0].legend(loc='lower right')
    axs[0].grid(True)

    # Subplot 2: Vehicle vs Wheel Speed
    axs[1].plot(time, log_v_veh, 'g-', label='Vehicle Ground Speed (m/s)')
    axs[1].plot(time, log_v_wheel, 'r--', label='Wheel Speed Equivalent (m/s)')
    axs[1].set_ylabel('Velocity [m/s]')
    axs[1].legend(loc='lower right')
    axs[1].grid(True)

    # Subplot 3: Slip and Environment
    axs[2].plot(time, log_slip, 'm-', label='Calculated Slip Ratio')
    axs[2].axhline(y=0.12, color='orange', linestyle=':', label='Target Slip Limit (0.12)')
    axs[2].plot(time, log_mu, 'c-', alpha=0.5, label='Surface Friction (\u03bc)')
    axs[2].set_ylabel('Ratio / \u03bc')
    axs[2].set_xlabel('Time [s]')
    axs[2].legend(loc='upper right')
    axs[2].grid(True)

    plt.tight_layout()
    plt.savefig('05_Verification/PLOT_Traction_Split_Mu.png')
    print("Simulation complete. Verification plot saved to '05_Verification/PLOT_Traction_Split_Mu.png'")

if __name__ == "__main__":
    run_mil_simulation()