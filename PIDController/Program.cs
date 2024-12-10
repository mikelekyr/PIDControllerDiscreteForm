using PIDControllerNamespace;

// Controller parameters
float KP = 5.0f;
float KI = 5.5f;
float KD = 0.25f;
float LIM_MIN = -10.0f;
float LIM_MAX = 10.0f;
float LIM_MIN_INT = -5.0f;
float LIM_MAX_INT = 5.0f;
float SAMPLE_TIME_S = 0.01f;
float TAU = SAMPLE_TIME_S / 5.0f;

// Maximum run-time of simulation
float SIMULATION_TIME_MAX = 4.0f;

PIDController pid = new(KP, KI, KD, TAU, LIM_MIN, LIM_MAX, LIM_MIN_INT, LIM_MAX_INT, SAMPLE_TIME_S);

// Simulate response using test system
float setpoint = 2.0f;
float correction = 0.0f;

DCMotorSimulator.SetParameters(0.5f, SAMPLE_TIME_S); 
Console.WriteLine( "Time (s)\tSystem Output\tControllerOutput\r\n");

for (float t = 0.0f; t <= SIMULATION_TIME_MAX; t += SAMPLE_TIME_S)
{
    // Get measurement from system
    DCMotorSimulator.SimulateStep(correction);
    float angularVelocity = DCMotorSimulator.AngularVelocity; 

    // calculate PID correction based on feedback
    correction = pid.Update(setpoint, angularVelocity);

    // Compute new control signal
    Console.WriteLine("{0:f}\t{1:f}\t{2:f}", t, angularVelocity, correction);
}
