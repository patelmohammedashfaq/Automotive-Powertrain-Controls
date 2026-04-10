# Project Documentation: Terrain-Adaptive Traction Control System

This documentation follows the **ASPICE-aligned 4-layer V-Cycle** architecture required for JLR powertrain projects, ensuring full traceability from requirements to MIL verification.

## 1. Requirements Specification (ASPICE Layer 1)
The following requirements define the functional and safety boundaries of the Traction Control System (TCS).

| REQ-ID | Requirement Description | Rationale | ASIL |
| :--- | :--- | :--- | :--- |
| **REQ-TRC-01** | The system shall calculate the real-time slip ratio using vehicle longitudinal velocity ($v_{veh}$) and wheel angular velocity ($\omega_{wheel}$). | Baseline for traction monitoring. | QM |
| **REQ-TRC-02** | To prevent division-by-zero, the slip calculation shall implement a velocity floor of **0.5 m/s**. | Signal Integrity (Portfolio DNA). | ASIL-B |
| **REQ-TRC-03** | The controller shall target a slip ratio of **0.12 (12%)** for optimal longitudinal grip. | Performance Optimization. | QM |
| **REQ-TRC-04** | If the slip error is positive, the system shall intervene by reducing driver torque demand using a proportional gain ($K_P$) of **600.0**. | Closed-loop stability. | ASIL-B |
| **REQ-TRC-05** | Final motor torque output shall be saturated between **0.0 Nm and 450.0 Nm** to protect hardware. | Actuator Protection. | ASIL-B |

---

## 2. Requirements Traceability Matrix (RTM)
This matrix ensures every requirement is mapped to a specific design component and a verification test case.

| REQ-ID | Design Component (Module) | Verification Test Case | Test Result |
| :--- | :--- | :--- | :--- |
| **REQ-TRC-01** | `CTRL_Traction_Logic.step()` | MIL_Scenario_SplitMu | **PASS** |
| **REQ-TRC-02** | `v_ref = max(in_v_veh, 0.5)` | MIL_Scenario_LowSpeed | **PASS** |
| **REQ-TRC-03** | `self.K_SLIP_TARGET` Calibration | MIL_Scenario_SplitMu | **PASS** |
| **REQ-TRC-04** | `P_Reduction` Logic Block | MIL_Scenario_SplitMu | **PASS** |
| **REQ-TRC-05** | `np.clip()` Actuator Protection | Hardware Limit Check | **PASS** |

---

## 3. Subsystem Detailed Design (MAAB Compliant)

### 3.1 Controller Subsystem: `CTRL_Traction_Logic`
Following **MAAB standards**, this diagram illustrates the internal logic with a strictly **Left-to-Right signal flow**.

```mermaid
graph LR
    subgraph Sensing_Layer [1. Sensing & Signal Conditioning]
        direction LR
        V_IN([in_v_veh])
        W_IN([in_w_wheel])
        V_Ref[Math: Max v, 0.5]
        Slip_Calc[Slip Calculation<br/>w*r - v / v_ref]
    end

    subgraph Control_Logic [2. Error & Intervention Logic]
        direction LR
        K_Target[[K_SLIP_TARGET: 0.12]]
        Error_Node((+/-))
        K_Gain[[K_P_GAIN: 600.0]]
        Intervention{Slip Error > 0?}
    end

    subgraph Arbitration_Layer [3. Output Protection]
        direction LR
        Trq_Req([in_driver_trq_req])
        Reducer[Torque Reduction Product]
        Trq_Sum(( - ))
        Sat[Saturation: 0 to 450Nm]
    end

    %% Internal Signal Flow
    V_IN --> V_Ref
    V_Ref --> Slip_Calc
    W_IN --> Slip_Calc
    V_IN --> Slip_Calc

    Slip_Calc --> Error_Node
    K_Target --> Error_Node
    
    Error_Node --> Intervention
    Error_Node --> K_Gain
    K_Gain --> Reducer
    
    Trq_Req --> Trq_Sum
    Reducer --> Trq_Sum
    
    Intervention -- "True" --> Trq_Sum
    Intervention -- "False" --> Sat
    Trq_Sum --> Sat
    Sat --> Out_Trq([out_trq_motor])
3.2 Plant Subsystem: PLNT_Traction_DynamicsThis models the high-fidelity physics (2200kg SUV) including non-linear tire-road friction.Code snippetgraph LR
    subgraph Propulsion_Physics [Wheel Dynamics]
        direction LR
        Trq_In([in_trq_motor])
        J_Wh[[J_wheel: 15.0]]
        Inertia_Eq[dw/dt = T - F*r / J]
    end

    subgraph Friction_Model [Contact Patch]
        direction LR
        Mu_In([in_mu_ground])
        Tanh_Curve[Non-Linear Tanh Model]
        F_Traction[F = mu * m * g * tanh]
    end

    subgraph Chassis_Physics [Longitudinal Dynamics]
        direction LR
        Mass[[mass: 2200.0]]
        Aero_Drag[Drag = 0.5 * rho * Cd * Af * v^2]
        Chassis_Eq[dv/dt = F - Drag / mass]
    end

    %% Physics Feedback Loops
    Trq_In --> Inertia_Eq
    Inertia_Eq --> Tanh_Curve
    Mu_In --> Tanh_Curve
    Tanh_Curve --> F_Traction
    F_Traction --> Chassis_Eq
    F_Traction --> Inertia_Eq
    Chassis_Eq --> Out_V([out_v_veh])
    Inertia_Eq --> Out_W([out_w_wheel])

# MIL Verification Fault Report: Traction Control Failure

## 1. Test Overview
* **Test ID:** MIL_TC_001_TRQ_RED
* **Requirement Target:** REQ-TRC-04 (Proportional Torque Reduction)
* **Surface Condition:** Tarmac to Ice transition at $t=2.0s$ ($\mu$ drop from 0.9 to 0.1).
* **Driver Intent:** Static 400 Nm Torque Request.

## 2. Fault Analysis (Actual vs. Expected)

| Metric | Target/Expected Behavior | Actual Observation (from Plot) | Status |
| :--- | :--- | :--- | :--- |
| **Wheel Slip Detection** | Slip should trigger logic at $t=2.0s$ | Wheel speed spiked to $>40 m/s$ at $t=2.0s$ | **DETECTED** |
| **Torque Response** | `out_trq_motor` should drop to $<100 Nm$ | `out_trq_motor` remained at **400 Nm** | **FAILED** |
| **Logic Activation** | Controller should enter "Intervention" mode | Logic stayed in "Pass-through" mode | **FAILED** |
| **Vehicle Stability** | Longitudinal slip regulated to 12% | Uncontrolled wheel spin (Runaway) | **UNSAFE** |

---

## 3. Evidence of Failure
The provided MIL simulation plot (PLOT_Traction_Split_Mu.png) clearly shows that while the physical plant responded to the ice patch (red dashed line spike), the control logic failed to reduce the blue torque line.

![MIL Verification Plot](05_Verification/PLOT_Traction_Split_Mu.png)
---

## 4. Root Cause Investigation
Based on the trace analysis, three potential software bugs have been identified for code review:

| Fault Type | Potential Cause | Action Item |
| :--- | :--- | :--- |
| **Naming Mismatch** | `slip_error` calculated in sensing block is not used in the arbitration block. | Verify variable scope in `CTRL_Traction_Logic`. |
| **Gain Calibration** | `K_P_GAIN` is set to 0.0 or a very low value in the parameters file. | Check `K_P_GAIN` value in data dictionary. |
| **Logic Path Error** | The `if slip_error > 0:` condition is bypassing the subtraction node. | Debug the arbitration switch statement. |

---

## 5. Requirement Traceability Matrix (Updated)

| REQ-ID | Description | Design Component | MIL Verdict |
| :--- | :--- | :--- | :--- |
| **REQ-TRC-01** | Slip Calculation Logic | `Slip_Calc` Module | **PASS** |
| **REQ-TRC-04** | Torque Reduction Logic | `P_Reduction` Block | **FAIL** |
| **REQ-TRC-05** | Actuator Saturation | `np.clip` Logic | **PASS** |

---

## 6. Corrective Action & Re-test Plan
1. **Fix:** Refactor the `step()` function to ensure the `driver_request` is subtracted by `(slip_error * K_P_GAIN)`.
2. **Verify:** Re-run the simulation at 100Hz task rate.
3. **Exit Criteria:** `out_trq_motor` must drop immediately when `out_slip` exceeds the 0.12 threshold.