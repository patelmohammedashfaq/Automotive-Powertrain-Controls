Project: Automotive Terrain Response 2 (TR2) Digital Twin
Overview
This project involves the development of a Model-in-the-Loop (MiL) simulation for an automated Terrain Response system. I designed a Vehicle Control Unit (VCU) in Python that dynamically adjusts torque and system modes based on real-time physics feedback from a Modelica-based physical plant.
1. System Architecture
The project follows a co-simulation architecture where the "Brain" and the "Body" are decoupled:
* The Controller (Python): Contains the state machine, slip observers, and signal processing (Standard Deviation for jitter detection).
* The Plant (Modelica/FMU): A high-fidelity physical model of a 2200kg SUV, simulating rotational inertia, damping, and non-linear tire-surface friction.
* Interface: Functional Mock-up Interface (FMI 2.0) used for cross-platform execution.

Shutterstock
Explore

2. Control Logic & Requirements
The controller was designed to meet three primary JLR-style engineering requirements:
* REQ-001 (Slip Detection): System must switch to Sand Mode when longitudinal slip ($\lambda$) exceeds 0.05.
* REQ-002 (Roughness Detection): System must switch to Rock Crawl when wheel speed variance (jitter) exceeds 0.2 rad/s.
* REQ-003 (Hysteresis): To prevent "mode hunting," terrain changes must be sustained for 3 consecutive cycles before arbitration.

3. Physical Modeling (Plant)
The vehicle dynamics are governed by the following differential equation, which I implemented in Modelica:
$$\text{J}_{wheel} \cdot \dot{\omega} = \tau_{motor} - F_{traction} \cdot r_w - b \cdot \omega$$
Where:
* $F_{traction}$ is modeled using a non-linear sigmoid friction curve to simulate traction limits on various surfaces ($\mu$).
* $b$ (damping) and $J$ (inertia) were tuned to stabilize the sensor signals for MIL testing.

4. Verification & Validation (The Results)
The final simulation run demonstrates the system's ability to navigate three distinct environments in a single 10-second window.
Final Test Case Result:
* 0-3s (Normal/Startup): System manages initial start-up oscillations.
* 3-6s (Sand): Friction dropped to $\mu=0.1$. The graph clearly shows Wheel Speed pulling away from Vehicle Speed. The controller successfully identified the slip and triggered Sand Mode.
* 6-10s (Rock Garden): Gaussian noise was injected into the wheel sensors. The controller s signal processing identified the high variance and switched to Rock Mode.


![Closed-Loop Performance](./05_Verification/Figure_1.png)
*Figure 1: MIL Simulation results showing real-time mode switching between Normal, Sand, and Rock modes based on slip and jitter observers.*

---


5. Key Skills Demonstrated
* Systems Engineering: Managing the interaction between software and physical hardware.
* Digital Twin Development: Exporting and integrating FMUs into Python environments.
* Signal Processing: Implementing real-time standard deviation and slip observers.
* Calibration: Tuning PID-like damping and inertia parameters to achieve system stability.

```mermaid
graph LR
    subgraph Controller [ECU: Terrain Response Controller]
        direction TB
        subgraph Input_Processing [Input Conditioning]
            A[In_Pedal_Pct] --> B[Map_Selector]
            C[In_V_Wheel] --> D[Jitter_Calculator_STD]
            E[In_V_Veh] --> F[Slip_Observer]
        end

        subgraph Arbitration_Logic [Mode Arbitration & Hysteresis]
            D --> G{_observer_logic}
            F --> G
            H[In_Driver_Select] --> I[Mode_Arbiter]
            G --> I
            I --> J[K_HYSTERESIS_Counter]
        end

        subgraph Torque_Mapping [Final Mapping]
            J --> K[Active_Mode_Selection]
            B --> L[1D_Interp_Table]
            K --> L
        end
    end

    subgraph Plant [Plant: Vehicle Physical Model]
        M[Motor_Dynamics] --> N[Wheel_Dynamics_J_Wheel]
        N --> O[Traction_Force_F_traction]
        O --> P[Vehicle_Dynamics_Mass]
    end

    %% Connections
    L -- "Out_Torque_Req" --> M
    P -- "V_Veh" --> F
    N -- "V_Wheel" --> D
    N -- "V_Wheel" --> F

