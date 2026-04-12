# ASPICE Layer 1: System Requirements - VCU Torque Demand Arbitrator

This document defines the functional and safety requirements for the Vehicle Control Unit (VCU) Torque Demand Arbitrator. These requirements serve as the "Source of Truth" for the controller logic and verification test benches.

## 1. System Requirements Table

| REQ-ID | Category | Requirement Description | Traceability / Metric |
| :--- | :--- | :--- | :--- |
| **REQ-VCU-01** | Functional | **Driver Interpretation:** The system shall map `In_PedalPos_pct` to `Trq_Driver_Nm` using a 1D Look-Up Table (LUT) defined by $K_{MAP\_DRV}$. | Performance Map |
| **REQ-VCU-02** | Safety | **Brake Priority (ASIL-D):** If `In_BrakeActive_bool` is `TRUE`, the output torque shall be immediately forced to $0\text{Nm}$ regardless of accelerator input. | ISO 26262 |
| **REQ-VCU-03** | Functional | **Signal Conditioning:** The raw pedal input shall be processed through a First-Order Low-Pass Filter with a time constant $K_{\tau} = 0.05\text{s}$ to remove sensor jitter. | Drivability |
| **REQ-VCU-04** | Functional | **Positive Rate Limiting:** To protect the driveline, the torque increase gradient shall not exceed $K_{SLEW\_POS} = 500\text{Nm/s}$. | Mechanical Health |
| **REQ-VCU-05** | Functional | **Negative Rate Limiting:** To prevent "tip-out" jerk, the torque decrease gradient shall be limited to $K_{SLEW\_NEG} = 800\text{Nm/s}$. | Drivability |
| **REQ-VCU-06** | Safety | **Pedal Plausibility:** If `In_PedalPos_pct` is detected outside the range $[-5\%, 105\%]$, the system shall trigger a Safe State ($0\text{Nm}$) within $20\text{ms}$. | ASIL-C |
| **REQ-VCU-07** | Safety | **Thermal Derating:** If `In_MotorTemp_C` exceeds $100^{\circ}\text{C}$, the max allowable torque shall be linearly attenuated to $0\text{Nm}$ at $120^{\circ}\text{C}$. | Component Health |
| **REQ-VCU-08** | Functional | **Output Saturation:** The final `Out_ArbitratedTorque_Nm` shall be strictly clipped to the hardware limit $K_{MAX\_TRQ} = 450\text{Nm}$. | Actuator Limit |

---

## 2. Mathematical Implementation logic

### 2.1 Torque Rate Limiter (Slew Filter)
The implementation of **REQ-VCU-04** and **REQ-VCU-05** follows the discrete-time gradient limit equation:

$$\text{Trq}_{out}(k) = \text{Trq}_{out}(k-1) + \text{clamp}\left(\text{Trq}_{target} - \text{Trq}_{out}(k-1), -R_{neg}, R_{pos}\right)$$

**Where:**
* $R_{pos} = K_{SLEW\_POS} \cdot dt$
* $R_{neg} = K_{SLEW\_NEG} \cdot dt$
* $dt = 0.01\text{s}$ (System Sample Time)

### 2.2 Thermal Derating Factor
The derating multiplier $\lambda$ is calculated as:

$$\lambda = \text{clamp}\left(1.0 - \frac{\text{In\_MotorTemp\_C} - 100}{120 - 100}, 0.0, 1.0\right)$$

---

## 3. Calibration Parameters (Control DNA)

| Parameter Name | Value | Unit | Description |
| :--- | :--- | :--- | :--- |
| `K_DT` | 0.01 | s | Controller sample time (100Hz) |
| `K_MAX_TRQ` | 450.0 | Nm | Absolute maximum hardware torque |
| `K_SLEW_POS` | 500.0 | Nm/s | Max torque increase rate (Tip-in) |
| `K_SLEW_NEG` | 800.0 | Nm/s | Max torque decrease rate (Tip-out) |
| `K_TAU_PEDAL` | 0.05 | s | Low-pass filter time constant for pedal |