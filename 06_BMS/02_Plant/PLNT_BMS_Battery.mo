model PLNT_BMS_Battery
  // Parameters
  parameter Real C_nominal = 100 "Nominal Capacity in Ah";
  parameter Real R_internal = 0.01 "Internal resistance in Ohms";
  parameter Real V_nom_cell = 3.7 "Nominal Cell Voltage";
  
  // Inputs/Outputs
  input Real In_Current "Pack Current [A]";
  output Real Out_CellVoltages[3] "Individual Cell Voltages [V]";
  output Real Out_PackVoltage "Total Pack Voltage [V]";
  output Real Out_PackTemp "Pack Temperature [degC]";
  
  // Internal State
  Real SoC(start=0.8); 
  
equation
  // Simple SoC physics for plant simulation
  der(SoC) = -In_Current / (C_nominal * 3600);
  
  // Simulated voltage drops due to resistance and SoC
  for i in 1:3 loop
    Out_CellVoltages[i] = V_nom_cell + (SoC - 0.5) * 0.5 - (In_Current * R_internal);
  end for;
  
  Out_PackVoltage = sum(Out_CellVoltages);
  Out_PackTemp = 25.0 + (In_Current^2 * R_internal * 0.1); // Simple Joule heating
end PLNT_BMS_Battery;
