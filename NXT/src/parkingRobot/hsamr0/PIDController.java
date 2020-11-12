package parkingRobot.hsamr0;

public class PIDController {
	
	/**
	 * Calculates the amount of change required to bring the process variable to its
	 * setpoint.
	 * 
	 * @param data 
	 * @return The amount of power change required to bring the process variable to its
	 * 	setpoint. 
	 */
	
	static double pi_ctrl(PIDData data) {
		double error = data.setpoint - data.processVariable;
		
		data.integral = data.integral + error;
		
		double result = error * data.param_p + data.integral * data.param_i;
		
		return result;
	}
}
