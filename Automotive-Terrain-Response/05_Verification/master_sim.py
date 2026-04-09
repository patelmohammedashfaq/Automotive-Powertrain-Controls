import numpy as np
import matplotlib.pyplot as plt
from fmpy import read_model_description, extract
from fmpy.fmi2 import FMU2Slave
import sys
import os

# 1. Setup Paths
sys.path.append(os.path.abspath('../04_Controller'))
from controller import TerrainResponseLogic

# Update this path to your actual FMU location
fmu_path = '../03_Plant/VehiclePlant.fmu'

if not os.path.exists(fmu_path):
    print(f"ERROR: FMU not found at {fmu_path}")
    sys.exit()

# 2. Load Model Description FIRST
model_description = read_model_description(fmu_path)
unzipdir = extract(fmu_path)
vcu = TerrainResponseLogic()

# 3. Initialize FMU Slave
fmu = FMU2Slave(guid=model_description.guid,
                modelIdentifier=model_description.modelExchange.modelIdentifier,
                unzipDirectory=unzipdir)

fmu.instantiate()
fmu.setupExperiment(startTime=0.0)
fmu.enterInitializationMode()
fmu.exitInitializationMode()

# 4. Identify Variable References by Name (Safest Method)
# This prevents index errors if the Modelica model changes
vrs = {}
for variable in model_description.modelVariables:
    vrs[variable.name] = variable.valueReference

# 5. Simulation Settings
t_end = 10.0
dt = 0.01 
time = np.arange(0, t_end, dt)
history = {"time": [], "v_veh": [], "v_wheel": [], "mode": []}

print("Starting JLR Terrain Response Simulation...")

for t in time:
    # Get Current Physics from FMU
    v_veh = fmu.getReal([vrs['out_v_veh']])[0]
    v_wheel_raw = fmu.getReal([vrs['out_v_wheel']])[0]
    
    # Simulate Environment Scenarios
    if t < 3.0:
        mu = 0.8  # Asphalt
        v_wheel = v_wheel_raw
    elif 3.0 <= t < 6.0:
        mu = 0.1 # Sand (Reduced friction), Reduced Sand friction from 0.2 to 0.1
        v_wheel = v_wheel_raw
    else:
        mu = 0.6  # Rock Garden
        # Add Jitter/Noise to the wheel speed signal
        v_wheel = v_wheel_raw + np.random.normal(0, 2.5) #Increased the Rock noise from 0.8 to 2.5

    # RUN THE CONTROLLER BRAIN
    # pedal=70%, driver_select=0 (Auto Mode)
    trq_req, active_mode, slip, jitter = vcu.run_logic(40.0, 0, v_wheel, v_veh)
    
    # SEND COMMANDS TO PHYSICS
    fmu.setReal([vrs['motor_torque'], vrs['mu_surface']], [trq_req, mu])
    
    # STEP THE PHYSICS
    fmu.doStep(currentCommunicationPoint=t, communicationStepSize=dt)
    
    # Store Results
    history["time"].append(t)
    history["v_veh"].append(v_veh)
    history["v_wheel"].append(v_wheel)
    history["mode"].append(active_mode)

fmu.terminate()
fmu.freeInstance()

# 6. Plotting Results
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

ax1.plot(history["time"], history["v_veh"], label='Vehicle Speed (m/s)', linewidth=2)
ax1.plot(history["time"], history["v_wheel"], label='Wheel Speed (Sensor)', alpha=0.5)
ax1.set_ylabel('Velocity')
ax1.set_title('Terrain Response 2: Closed-Loop Performance')
ax1.legend()
ax1.grid(True)

ax2.step(history["time"], history["mode"], color='red', where='post', label='Active Mode')
ax2.set_yticks([1, 2, 3])
ax2.set_yticklabels(['Normal', 'Sand', 'Rock'])
ax2.set_ylabel('System State')
ax2.set_xlabel('Time (s)')
ax2.grid(True)

plt.tight_layout()
plt.show()