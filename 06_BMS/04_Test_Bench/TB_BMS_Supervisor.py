import numpy as np
import pandas as pd
from fmpy import read_model_description, extract
from fmpy.fmi2 import FMU2Slave
import os
import sys

# --- 1. ASPICE DIRECTORY RESOLUTION ---
# Anchoring to the script's location to ensure G: Drive compatibility
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(SCRIPT_DIR)

CONTROLLER_PATH = os.path.join(PROJECT_ROOT, "03_Controller")
PLANT_PATH = os.path.join(PROJECT_ROOT, "02_Plant")
VERIF_PATH = os.path.join(PROJECT_ROOT, "05_Verification")

sys.path.append(CONTROLLER_PATH)

# Import naming must match your CNTRL_BMS_Supervisor.py file
try:
    from CNTRL_BMS_Supervisor import BMS_Controller
except ImportError:
    print(f"CRITICAL ERROR: CNTRL_BMS_Supervisor.py not found in {CONTROLLER_PATH}")
    sys.exit(1)

FMU_FILE = os.path.join(PLANT_PATH, 'PLNT_BMS_Battery.fmu')

# --- 2. CORE SIMULATION ENGINE ---
def run_bms_test(scenario_name, stimulus_func):
    """
    Standard MIL Orchestrator. 
    Stimulus_func must return (Current_A, Sensor_Offset_V)
    """
    if not os.path.exists(FMU_FILE):
        raise FileNotFoundError(f"FMU missing at: {FMU_FILE}")

    model_desc = read_model_description(FMU_FILE)
    unzip_dir = extract(FMU_FILE)
    
    # --- VERSION-AGNOSTIC FIX FOR modelIdentifier ---
    model_id = None
    if hasattr(model_desc, 'coSimulation'):
        model_id = model_desc.coSimulation.modelIdentifier
    else:
        model_id = model_desc.modelName

    fmu = FMU2Slave(guid=model_desc.guid,
                    unzipDirectory=unzip_dir,
                    modelIdentifier=model_id,
                    instanceName='BMS_Plant_Instance')
    
    vrs = {v.name: v.valueReference for v in model_desc.modelVariables}
    
    fmu.instantiate()
    fmu.setupExperiment(startTime=0.0)
    fmu.enterInitializationMode()
    fmu.exitInitializationMode()

    # Controller init (dt=10ms per SOP)
    bms = BMS_Controller(dt=0.01)
    results = []
    t = 0.0
    dt = 0.01
    t_end = 8.0 # Standard test duration

    while t < t_end:
        # A. Get Inputs from Scenario Layer
        i_req, v_offset = stimulus_func(t)
        
        # B. Step Plant Physics
        fmu.setReal([vrs['In_Current']], [i_req])
        fmu.doStep(currentCommunicationPoint=t, communicationStepSize=dt)
        
        # C. Capture Plant Response
        # Modelica arrays index from 1
        cell_v = [fmu.getReal([vrs[f'Out_CellVoltages[{i+1}]']])[0] for i in range(3)]
        pack_v = fmu.getReal([vrs['Out_PackVoltage']])[0]
        temp = fmu.getReal([vrs['Out_PackTemp']])[0]
        
        # D. Controller Execution with Fault Injection (REQ_07)
        measured_pack_v = pack_v + v_offset
        status, soc = bms.run_step(cell_v, measured_pack_v, i_req, temp)
        
        # E. Logging for V&V Report
        results.append({
            "Time": t,
            "Scenario": scenario_name,
            "In_Current": i_req,
            "Max_Cell_V": max(cell_v),
            "Min_Cell_V": min(cell_v),
            "Temp_C": temp,
            "Status": status,
            "SoC_Estimate": soc,
            "V_Plausibility_Err": abs(measured_pack_v - sum(cell_v))
        })
        t += dt

    fmu.terminate()
    fmu.freeInstance()
    return results

# --- 3. REQUIREMENT-SPECIFIC SCENARIOS (ALL COMBINATIONS) ---

def sc_nominal_soc(t):
    """BMS_REQ_01: Linear discharge test."""
    return 100.0, 0.0 # Constant 100A discharge

def sc_over_voltage(t):
    """BMS_REQ_02 & 06: Charging to trip 4.25V."""
    # Current < 0 charges the battery in the provided plant model
    return -250.0 if t < 5.0 else 0.0, 0.0

def sc_over_current_timing(t):
    """BMS_REQ_05 & 06: Test 500ms debounce vs 350A."""
    # Pulse 400A for exactly 0.6s. BMS should trip after 0.5s.
    return 400.0 if (1.0 < t < 1.6) else 10.0, 0.0

def sc_over_temp(t):
    """BMS_REQ_04: Trigger 60C limit via high current."""
    # Plant Joule heating: Temp = 25 + (I^2 * 0.01 * 0.1)
    # 200A -> 65C
    return 200.0 if t < 5.0 else 0.0, 0.0

def sc_plausibility(t):
    """BMS_REQ_07: Mismatch detection."""
    # Inject 0.2V offset at t=2.0s
    return 20.0, (0.2 if t > 2.0 else 0.0)

def sc_hysteresis_recovery(t):
    """BMS_REQ_09: Verify 2.0s recovery delay."""
    # Force trip, then return to normal and monitor status duration
    return 500.0 if t < 1.0 else 0.0, 0.0

# --- 4. EXECUTION ---
if __name__ == "__main__":
    suite = [
        ("REQ_01_SoC", sc_nominal_soc),
        ("REQ_02_OverVoltage", sc_over_voltage),
        ("REQ_05_OverCurrent", sc_over_current_timing),
        ("REQ_04_OverTemp", sc_over_temp),
        ("REQ_07_Plausibility", sc_plausibility),
        ("REQ_09_Hysteresis", sc_hysteresis_recovery)
    ]
    
    master_data = []
    print(">>> Starting Full Requirement Sweep...")
    
    for label, func in suite:
        print(f"Executing Scenario: {label}")
        master_data.extend(run_bms_test(label, func))
    
    # Save results to 05_Verification
    os.makedirs(VERIF_PATH, exist_ok=True)
    df = pd.DataFrame(master_data)
    log_file = os.path.join(VERIF_PATH, "BMS_Final_Verification_Log.csv")
    df.to_csv(log_file, index=False)
    
    print(f"\n[PASS] All Scenarios Complete. Verification Log: {log_file}")