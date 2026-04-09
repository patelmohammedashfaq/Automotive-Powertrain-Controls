Req ID	Category	Requirement Title	Detailed Technical Requirement Description	Priority
REQ-101	Functional	Manual Mode Selection	"The system shall read In_DriverSelect (UINT8). If the value is 1 (Normal), 2 (Sand), or 3 (Rock), the system shall lock the active mode to that selection, overriding all automatic logic."	High
REQ-102	Functional	Auto-Mode Activation	"The system shall enter ""Auto-Detection"" mode if and only if In_DriverSelect is equal to 0."	High
REQ-103	Estimation	Friction (?) Calculation	"In Auto-Mode, the system shall calculate the friction coefficient (?) by comparing In_MotorTorque_Nm against the WheelSlipRatio. Slip Ratio ? shall be defined as (?r?v)/v."	High
REQ-104	Estimation	Surface Roughness Detection	"The system shall analyze the In_WSS_rpm signal via a High-Pass Butterworth filter (Cut-off: 15Hz). If the RMS of the filtered signal exceeds K_Rough_Thresh (0.15), the surface shall be flagged as ""Rough/Rock""."	Medium
REQ-105	Logic	Auto-Transition Hysteresis	The system shall not transition between modes in Auto-Mode unless the detection criteria for the new mode are met continuously for a period of 500ms.	High
REQ-106	Drivability	Torque Interpretation: Sand	"In Sand Mode, the system shall utilize Map_Sand_Trq (Lookup Table) where 50% pedal travel results in ? 75% of K_MaxTorque_Nm."	Medium
REQ-107	Drivability	Torque Interpretation: Rock	"In Rock Mode, the system shall utilize Map_Rock_Trq (Lookup Table) where 50% pedal travel results in ? 30% of K_MaxTorque_Nm."	Medium
REQ-108	Drivability	Torque Rate Limiting	"Upon any mode change (Manual or Auto), the output Out_TrqReq_Nm shall be limited to a maximum gradient of 500 Nm/s to prevent drivetrain ""clunk""."	High
REQ-201	Safety	ASIL-B: Range Check	"The system shall monitor In_PedalPos_pct. If the value is <?5% or >105%, a Signal_Invalid fault shall be latched."	High
REQ-202	Safety	Safe State Definition	"Upon a Signal_Invalid fault or a communication loss (Timeout > 100ms), the system shall immediately set Out_TrqReq_Nm to 0Nm."	High
REQ-301	HMI	Driver Feedback	The system shall output Out_ActiveMode_ID to the instrument cluster. Manual modes shall be IDs 1-3; Auto-detected modes shall be IDs 11-13.	Low
