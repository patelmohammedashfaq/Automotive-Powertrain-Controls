### Energy Management System (EMS) Requirements

| REQ-ID | Type | Description |
| :--- | :--- | :--- |
| **REQ-EMS-001** | Functional | The controller shall distribute total `In_TrqReq_Nm` between the ICE and Electric Motor to maintain Battery SoC within 20%–80%. |
| **REQ-EMS-002** | Functional | The system shall prioritize "EV-Only" mode if vehicle speed is below 30 kph and SoC > 30%. |
| **REQ-EMS-003** | **Safety** | If Battery SoC falls below 15%, the ICE must be commanded to "Charge-Sustaining" mode regardless of driver demand. |
| **REQ-EMS-004** | **Safety** | The sum of Engine and Motor torque requests shall never exceed the physical limit of the drivetrain (`K_MAX_TRQ`). |