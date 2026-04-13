model PLNT_TorqueVectoring "Vehicle plant model for torque vectoring"
  
  // --- VEHICLE PARAMETERS ---
  parameter Real mass = 2200.0 "kg - Total vehicle mass";
  parameter Real J_yaw = 4500.0 "kg.m2 - Yaw Moment of Inertia";
  parameter Real L_f = 1.4 "m - Distance CG to front axle";
  parameter Real L_r = 1.6 "m - Distance CG to rear axle";
  parameter Real track_width = 1.8 "m - Lateral distance between wheels";
  parameter Real r_w = 0.35 "m - Wheel radius";

  // --- INPUTS ---
  input Real in_Trq_L "Nm";
  input Real in_Trq_R "Nm";
  input Real in_Steer_rad "rad";

  // --- OUTPUTS ---
  output Real out_YawRate "rad/s";
  output Real out_Vx "m/s";

  // --- STATE VARIABLES ---
  Real vx(start=10.0, fixed=true);
  Real yaw_rate(start=0.0, fixed=true);
  
equation
  // Longitudinal Dynamics
  mass * der(vx) = (in_Trq_L + in_Trq_R) / r_w;

  // Yaw Dynamics
  J_yaw * der(yaw_rate) = ((in_Trq_R - in_Trq_L) * (track_width / 2) / r_w) + 
                          (in_Steer_rad * vx * 5000); 

  out_YawRate = yaw_rate;
  out_Vx = vx;
  
end PLNT_TorqueVectoring;
