import numpy as np
import matplotlib.pyplot as plt
import shutil
import os
import sys
from fmpy import read_model_description, extract
from fmpy.fmi2 import FMU2Slave

# Ensure we can import the controller from the ASPICE directory structure [cite: 95]
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '03_Controller')))
try:
    from CNTRL_EMS import EnergyManager
except ImportError:
    print("Ensure CNTRL_EMS.py is located in the ../03_Controller/ directory.")

def get_scenario_demand(t):
    """
    Scenario Layer: Injects driver intent to excite the controller logic[cite: 104, 215].
    """
    if t < 15:
        return 50.0    # Low demand, should trigger EV_PRIORITY if SoC > 30% and Spd < 30kph
    elif t < 40:
        return 350.0   # High demand, should trigger HYBRID_PARALLEL
    elif t < 60:
        return -100.0  # Regen braking
    else:
        return 120.0   # Cruising

def run_mil_test(initial_soc=0.8, scenario_name="Nominal"):
    """
    Closed-Loop Simulation using fmpy[cite: 103].
    """
    fmu_filename = '../02_Plant/PLNT_EMS.fmu' # Path based on SOP [cite: 94]
    
    if not os.path.exists(fmu_filename):
        print(f"FMU not found at {fmu_filename}. Please compile PLNT_EMS.mo first.")
        return None

    # Constants & Setup
    t_start = 0.0
    t_stop = 100.0
    dt = 0.01  # K_DT Task Rate [cite: 239]
    
    controller = EnergyManager()
    
    # 1. Setup FMU (Plant)
    model_description = read_model_description(fmu_filename)
    vrs = {var.name: var.valueReference for var in model_description.modelVariables}
    unzipdir = extract(fmu_filename)
    
    fmu = FMU2Slave(
        guid=model_description.guid,
        unzipDirectory=unzipdir,
        modelIdentifier=model_description.coSimulation.modelIdentifier,
        instanceName='PLNT_EMS_Instance'
    )
    fmu.instantiate()
    fmu.setupExperiment(startTime=t_start)
    fmu.enterInitializationMode()
    
    # Override initial SoC for sensitivity sweep [cite: 217, 218]
    fmu.setReal([vrs['SoC']], [initial_soc])
    fmu.exitInitializationMode()

    # Data Logging
    log = {'time': [], 'soc': [], 'spd_kph': [], 'req_trq': [], 'eng_trq': [], 'mot_trq': [], 'mode': []}
    
    print(f"Starting MIL Simulation: {scenario_name} (Initial SoC: {initial_soc*100}%)")
    time = t_start
    
    # 2. Simulation Loop (Orchestrator) [cite: 91]
    while time <= t_stop:
        # Step A: Read Plant Sensors
        plant_outputs = fmu.getReal([vrs['out_SoC'], vrs['out_v_veh']])
        soc, v_veh = plant_outputs[0], plant_outputs[1]
        
        # Step B: Read Scenario Layer
        trq_req = get_scenario_demand(time)
        
        # Step C: Execute Controller
        eng_trq, mot_trq = controller.run_step(soc, v_veh, trq_req)
        
        # Step D: Write Plant Actuators (Ensure clipping per Defensive Logic) [cite: 210]
        fmu.setReal([vrs['in_EngTrq_Nm'], vrs['in_MotTrq_Nm']], [eng_trq, mot_trq])
        
        # Step E: Step Plant Physics
        fmu.doStep(currentCommunicationPoint=time, communicationStepSize=dt)
        
        # Log data
        log['time'].append(time)
        log['soc'].append(soc)
        log['spd_kph'].append(v_veh * 3.6)
        log['req_trq'].append(trq_req)
        log['eng_trq'].append(eng_trq)
        log['mot_trq'].append(mot_trq)
        log['mode'].append(controller.current_mode)
        
        time += dt

    fmu.terminate()
    fmu.freeInstance()
    shutil.rmtree(unzipdir, ignore_errors=True)
    return log

def generate_verification_report(logs):
    """
    Generates plots for the 05_Verification/ directory[cite: 207, 244].
    """
    fig, axs = plt.subplots(3, 1, figsize=(10, 12), sharex=True)
    
    # Plot 1: Torque Arbitration (REQ-EMS-001 & REQ-EMS-004)
    axs[0].plot(logs['time'], logs['req_trq'], 'k--', label='Driver Demand (Nm)')
    axs[0].plot(logs['time'], logs['eng_trq'], label='ICE Torque', alpha=0.8)
    axs[0].plot(logs['time'], logs['mot_trq'], label='Motor Torque', alpha=0.8)
    axs[0].set_ylabel('Torque (Nm)')
    axs[0].set_title('Torque Arbitration & Actuator Delivery')
    axs[0].legend()
    axs[0].grid()

    # Plot 2: Speed and Mode Flags (REQ-EMS-002)
    mode_map = {"EV_PRIORITY": 1, "HYBRID_PARALLEL": 2, "CHARGE_SUSTAIN": 3}
    num_modes = [mode_map[m] for m in logs['mode']]
    
    ax2_1 = axs[1]
    ax2_2 = axs[1].twinx()
    ax2_1.plot(logs['time'], logs['spd_kph'], 'purple', label='Vehicle Speed')
    ax2_1.axhline(30, color='r', linestyle=':', label='30 kph EV Threshold')
    ax2_2.step(logs['time'], num_modes, 'k', where='post', label='Op Mode')
    ax2_2.set_yticks([1, 2, 3])
    ax2_2.set_yticklabels(["EV", "HYBRID", "CS"])
    ax2_1.set_ylabel('Speed (kph)')
    ax2_1.legend(loc='upper left')
    axs[1].grid()

    # Plot 3: Battery SoC Tracking (REQ-EMS-003)
    axs[2].plot(logs['time'], np.array(logs['soc'])*100, 'g', label='Battery SoC (%)')
    axs[2].axhline(25, color='orange', linestyle='--', label='Lower Target (25%)')
    axs[2].axhline(15, color='red', linestyle='-', label='Critical Safety (15%)')
    axs[2].set_ylabel('SoC (%)')
    axs[2].set_xlabel('Time (s)')
    axs[2].legend()
    axs[2].grid()

    plt.tight_layout()
    
    # Save artifact to verification folder [cite: 96, 284]
    os.makedirs('../05_Verification', exist_ok=True)
    plt.savefig('../05_Verification/MIL_EMS_Verification.png')
    print("Verification plot saved to 05_Verification/MIL_EMS_Verification.png")
    plt.show()

if __name__ == "__main__":
    # Batch Sweep Implementation [cite: 218, 278]
    # Test 1: Nominal start at 80% SoC
    log_nominal = run_mil_test(initial_soc=0.80, scenario_name="Nominal")
    
    # Test 2: Edge Case start at 18% SoC to force REQ-EMS-003 (Safety Limit Exitation) 
    log_edge = run_mil_test(initial_soc=0.18, scenario_name="Low_SoC_Safety_Check")
    
    if log_nominal:
        generate_verification_report(log_nominal)