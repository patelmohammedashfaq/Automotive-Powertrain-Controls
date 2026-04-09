model VehiclePlant
  // Inputs from Python Controller
  input Real motor_torque "Nm";
  input Real mu_surface = 0.8 "Road Friction (Asphalt=0.8, Sand=0.2)";
  
  // Vehicle Parameters
  parameter Real mass = 2200 "kg";
  parameter Real J_wheel = 15.0 "Increased from 2.5 to stabilize signal";
  parameter Real b_damping = 10.0 "Added physical damping";
  parameter Real r_w = 0.35 "m";
  parameter Real Cd = 0.38;
  parameter Real Af = 2.8;
  
  // Internal States
  Real v_veh(start=0) "Vehicle Velocity m/s";
  Real w_wheel(start=0) "Wheel Angular Velocity rad/s";
  Real slip;
  Real F_traction;
  
  // Outputs for Python Sensors
  output Real out_v_veh = v_veh;
  output Real out_v_wheel = w_wheel * r_w;

equation
  // Calculate Longitudinal Slip
  slip = (w_wheel*r_w - v_veh) / max(v_veh, 0.1);
  
  // Traction Force based on friction and slip
  F_traction = mu_surface * mass * 9.81 * (2 / (1 + exp(-2*slip)) - 1);
  
  // Wheel Dynamics: Torque balance
der(w_wheel) = (motor_torque - F_traction*r_w - b_damping*w_wheel) / J_wheel;
  
  // Vehicle Dynamics: Force balance
  der(v_veh) = (F_traction - 0.5*1.225*Cd*Af*v_veh^2) / mass;
  
end VehiclePlant;
