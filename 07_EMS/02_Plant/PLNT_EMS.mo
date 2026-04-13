model PLNT_EMS
  // 1. Inherited Core Vehicle DNA (Baseline) [cite: 75]
  parameter Real mass = 2200.0 "kg";
  parameter Real r_w = 0.35 "m";
  parameter Real Cd = 0.38 "1";
  parameter Real Af = 2.8 "m2";
  parameter Real rho = 1.225 "kg/m3";
  parameter Real g = 9.81 "m/s2";
  
  // 2. Project-Specific Expansion (Electrical/ICE) [cite: 76]
  parameter Real C_bat_Ah = 15.0 "Battery Capacity";
  parameter Real V_nom = 350.0 "Nominal Voltage";

  // 3. Signal Interfaces (MAAB Standard) [cite: 80]
  input Real in_EngTrq_Nm "ICE Torque Command";
  input Real in_MotTrq_Nm "Motor Torque Command";
  output Real out_v_veh "Vehicle Speed (m/s)";
  output Real out_SoC "Battery State of Charge (0-1)";

  // 4. Internal State Variables
  Real v_veh(start=0.0);
  Real SoC(start=0.8); // Start at 80% SoC
  Real F_prop;
  Real F_resist;

equation
  // Physics: Force Balance
  F_prop = (in_EngTrq_Nm + in_MotTrq_Nm) / r_w;
  F_resist = 0.5 * rho * Cd * Af * v_veh^2;
  
  // Acceleration = (Propulsion - Resistance) / Mass
  der(v_veh) = (F_prop - F_resist) / mass;
  out_v_veh = v_veh;

  // Electrical: SoC Integration (Current = Power / Voltage)
  // Negative MotTrq = Charging (Regen/ICE-gen)
  der(SoC) = -(in_MotTrq_Nm * (v_veh/r_w)) / (V_nom * C_bat_Ah * 3600);
  out_SoC = SoC;

end PLNT_EMS;
