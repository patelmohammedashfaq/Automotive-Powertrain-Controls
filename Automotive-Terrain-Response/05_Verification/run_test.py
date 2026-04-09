import sys
import os
import matplotlib.pyplot as plt
import numpy as np

sys.path.append(os.path.abspath('../02_Controller'))
from controller import TerrainResponseLogic

vcu = TerrainResponseLogic()
time_steps = np.linspace(0, 4, 400) 
res = {"torque": [], "mode": [], "jitter": []}

for t in time_steps:
    pedal = 30.0
    
    if t < 1.5:
        # Asphalt (Smooth)
        v_veh, v_wheel = 10.0, 10.05
    elif 1.5 <= t < 3.0:
        # Rock Garden (High Jitter/Noise)
        v_veh = 2.0
        # Add random noise to simulate bouncing over rocks
        v_wheel = 2.2 + np.random.normal(0, 0.6) 
    else:
        # Back to Asphalt
        v_veh, v_wheel = 10.0, 10.05
        
    trq, mode, slip, jitter = vcu.run_logic(pedal, 0, v_wheel, v_veh)
    
    res["torque"].append(trq)
    res["mode"].append(mode)
    res["jitter"].append(jitter)

# Plotting the Logic vs. The Noise
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

ax1.plot(time_steps, res["jitter"], color='purple', label='Calculated Jitter (Signal Noise)')
ax1.axhline(y=0.4, color='r', linestyle='--', label='Rock Detection Threshold')
ax1.set_ylabel('Jitter Magnitude')
ax1.set_title('Internal Observer: Real-Time Jitter Calculation')
ax1.legend()

ax2.step(time_steps, res["mode"], color='blue', linewidth=2)
ax2.set_yticks([1, 2, 3])
ax2.set_yticklabels(['Normal', 'Sand', 'Rock Crawl'])
ax2.set_ylabel('Active Mode')
ax2.set_xlabel('Time (s)')

plt.show()