model PLNT_Traction_Dynamics
  // Core Vehicle DNA (Immutable Baseline)
  parameter Real mass = 2200.0 "kg";
  parameter Real r_w = 0.35 "m";
  parameter Real J_wheel = 15.0 "kg.m2";
  parameter Real Cd = 0.38 "1";
  parameter Real Af = 2.8 "m2";
  parameter Real rho = 1.225 "kg/m3";
  
  // Signal Interfaces (MAAB Naming)
  input Real in_mu_ground "0..1 - Surface Friction (Ice=0.1, Tarmac=0.9)";
  input Real in_trq_motor "Nm - Torque from VCU";
  
  output Real out_v_veh "m/s";
  output Real out_w_wheel "rad/s";
  output Real out_slip "Ratio";

  // State Variables
  Real v_veh(start=10.0);
  Real w_wheel(start=28.5); // Initial w matching v_veh
  Real F_tractive;

equation
  // Slip Ratio Calculation with zero-speed protection
  out_slip = (w_wheel * r_w - v_veh) / max(v_veh, 0.5);
  
  // Simplified Linear Tractive Force 
  F_tractive = in_mu_ground * mass * 9.81 * tanh(10 * out_slip);

  // Longitudinal Equation of Motion
  mass * der(v_veh) = F_tractive - (0.5 * rho * Cd * Af * v_veh^2);
  
  // Wheel Rotational Equation of Motion
  J_wheel * der(w_wheel) = in_trq_motor - (F_tractive * r_w);
  
  // Assign to outputs
  out_v_veh = v_veh;
  out_w_wheel = w_wheel;
end PLNT_Traction_Dynamics;