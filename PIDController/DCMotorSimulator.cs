namespace PIDControllerNamespace
{
    public static class DCMotorSimulator
    {
        // Motor parameters
        private static float omega = 0.0f;  // Angular velocity (rad/s)
        private static float tau = 1.0f;    // Time constant (s)
        private static float dt = 0.01f;    // Time step for simulation (s)

        /// <summary>
        /// Gets the current angular velocity.
        /// </summary>
        public static float AngularVelocity => omega;

        /// <summary>
        /// Sets the motor parameters.
        /// </summary>
        /// <param name="timeConstant">Time constant of the motor (s).</param>
        /// <param name="timeStep">Simulation time step (s).</param>
        public static void SetParameters(float timeConstant, float timeStep)
        {
            if (timeConstant <= 0)
                throw new ArgumentException("Time constant must be positive.", nameof(timeConstant));
            if (timeStep <= 0)
                throw new ArgumentException("Time step must be positive.", nameof(timeStep));

            tau = timeConstant;
            dt = timeStep;

            omega = 0.0f;
        }

        /// <summary>
        /// Simulates one step of the motor dynamics.
        /// </summary>
        /// <param name="input">Control input (e.g., voltage or PWM signal).</param>
        public static void SimulateStep(float input)
        {
            // Update omega using a first-order model (Euler integration)
            omega += (1.0f / tau) * (input - omega) * dt;
        }
    }
}
