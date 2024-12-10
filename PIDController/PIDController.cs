namespace PIDControllerNamespace
{
    public sealed class PIDController(float parKp, float parKi, float parKd, float parTau, float parLimMin, float parLimMax, float parLimMinInt, float parLimMaxInt, float parSampleTime)
    {
        // Controller memory
        private float integrator = 0.0f;
        private float prevError = 0.0f; // Required for integrator
        private float differentiator = 0.0f;
        private float prevMeasurement = 0.0f; // Required for differentiator 

        // Controller gains 
        private readonly float Kp = parKp;
        private readonly float Ki = parKi;
        private readonly float Kd = parKd;

        // Derivative low-pass filter time constant
        private readonly float tau = parTau;

        // Output limits
        private readonly float limMin = parLimMin;
        private readonly float limMax = parLimMax;

        // Integrator limits
        private readonly float limMinInt = parLimMinInt;
        private readonly float limMaxInt = parLimMaxInt;

        /* Sample time (in seconds) */
        private readonly float T = parSampleTime;

        /// <summary>
        /// Reinitialize controller memory
        /// </summary>
        public void Reinitialize()
        {
            integrator = 0.0f;
            prevError = 0.0f;
            differentiator = 0.0f;
            prevMeasurement = 0.0f;
        }

        /// <summary>
        /// Update function
        /// </summary>
        /// <param name="setpoint">System setpoint</param>
        /// <param name="measurement">Real value measurement</param>
        /// <returns>Control value</returns>
        public float Update(float setpoint, float measurement)
        {
            // Error signal
            float error = setpoint - measurement;

            // Proportional
            float proportional = Kp * error;

            // Integral
            integrator += 0.5f * Ki * T * (error + prevError);

            /* Anti-wind-up via integrator clamping */
            integrator = integrator > limMaxInt ? limMaxInt : integrator < limMinInt ? limMinInt : integrator;

            // Derivative (band-limited differentiator)
            // Note: derivative on measurement, therefore minus sign in front of equation! 
            differentiator = -(2.0f * Kd * (measurement - prevMeasurement)   
                             + (2.0f * tau - T) * differentiator)
                             / (2.0f * tau + T);

            // Compute output and apply limits
            float result = proportional + integrator + differentiator;

            // Limit PID output
            result = result > limMax ? limMax : result < limMin ? limMin : result;

            // Store error and measurement for later use
            prevError = error;
            prevMeasurement = measurement;

            // Return controller output
            return result;
        }
    }
}
