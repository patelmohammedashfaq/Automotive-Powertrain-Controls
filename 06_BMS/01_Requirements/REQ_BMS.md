# Project: BMS State of Charge & Fault Detection
**Module:** Battery Management System (BMS) Supervisor

| Req ID | Requirement Name | Description | Parent ID |
| :--- | :--- | :--- | :--- |
| **BMS_REQ_01** | SoC Estimation | The system shall estimate Battery State of Charge ($SoC$) using Coulomb Counting: $SoC(t) = SoC(0) + \int \frac{I}{Cap} dt$. | Powertrain_Goal_01 |
| **BMS_REQ_02** | Over-Voltage (OV) | A fault state shall be triggered if any cell voltage exceeds 4.25V. | ASIL-C (Safety) |
| **BMS_REQ_03** | Under-Voltage (UV) | A fault state shall be triggered if any cell voltage drops below 2.50V to prevent chemical degradation. | ASIL-C (Safety) |
| **BMS_REQ_04** | Over-Temp (OT) | A fault state shall be triggered if the pack temperature exceeds 60.0°C. | ASIL-C (Safety) |
| **BMS_REQ_05** | Over-Current (OC) | The system shall trigger a fault if discharge current exceeds 350A for $> 100ms$. | Hardware_Prot |
| **BMS_REQ_06** | Fault Debouncing | Any voltage or temperature violation must persist for $\ge 500ms$ before latching a Fault to prevent nuisance trips. | Signal_Integrity |
| **BMS_REQ_07** | Plausibility Check | The system shall flag a sensor error if $|V_{pack} - \sum V_{cells}| > \epsilon$. | Sensor_Fusion |
| **BMS_REQ_08** | Safe-State Logic | Upon a confirmed/latched fault, the output `Out_BMS_Status` must transition to 0 (Open Contactors/Safe State). | ISO26262_Safe |
| **BMS_REQ_09** | Fault Hysteresis | A latched fault shall only be clearable if the violation signal remains in the "Normal" range for a continuous 2.0s. | Recovery_Logic |