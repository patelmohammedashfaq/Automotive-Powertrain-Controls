import matplotlib.pyplot as plt

def run_test_bench():
    # Simulation Parameters
    dt = 0.01
    duration = 5.0
    steps = int(duration / dt)
    
    # Initial Conditions
    m_air = 20.0  # kg/h
    target = 14.7
    ctrl = FuelControllerPID(Kp=1.2, Ki=0.5, Kd=0.1, target_lambda=target)
    
    # Data logging
    history = {"time": [], "lambda": [], "m_air": []}
    current_lambda = 14.7

    for i in range(steps):
        t = i * dt
        
        # Simulate a disturbance (Throttle opening at 2.0s)
        if t > 2.0:
            m_air = 40.0
            
        # Get Controller Action
        fuel_cmd = (m_air / target) + ctrl.update(current_lambda, dt)
        
        # Physics Placeholder (Simplified Plant Response)
        # In a full setup, you'd call the OpenModelica FMU here
        delay_factor = 0.15
        current_lambda = (m_air / fuel_cmd) * (1 - delay_factor) + (current_lambda * delay_factor)
        
        history["time"].append(t)
        history["lambda"].append(current_lambda)
        history["m_air"].append(m_air)

    # Visualization
    plt.figure(figsize=(10, 5))
    plt.subplot(2, 1, 1)
    plt.plot(history["time"], history["m_air"], label="Air Mass Flow")
    plt.legend()
    plt.subplot(2, 1, 2)
    plt.plot(history["time"], history["lambda"], color='red', label="Actual Lambda")
    plt.axhline(y=14.7, color='black', linestyle='--', label="Target")
    plt.legend()
    plt.show()

if __name__ == "__main__":
    run_test_bench()