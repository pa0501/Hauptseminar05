package parkingRobot.hsamr0;

import lejos.nxt.LCD;

public class PIDController {

	/**
	 * Calculates the amount of change required to bring the process variable to its
	 * setpoint.
	 * 
	 * @param data
	 * @return The amount of power change required to bring the process variable to
	 *         its setpoint.
	 */

	static double pi_ctrl(PIDData data) {
		double error = data.setpoint - data.processVariable;

		data.integral = data.integral + error;

		double result = error * data.param_p + data.integral * data.param_i;
		

		

		return result;
	}

	static double pd_ctrl(PIDData data) {
		double error = data.setpoint - data.processVariable;
		double derivate = (error - data.error_prev) / (System.currentTimeMillis() - data.millis_prev);

		double result = error * data.param_p + derivate * data.param_d;
		
		data.millis_prev = System.currentTimeMillis();

		return result;
	}
	
	static double pid_ctrl(PIDData data) {
		double error = data.setpoint - data.processVariable;
		double derivate = (error - data.error_prev) / (System.currentTimeMillis() - data.millis_prev);

		data.integral = data.integral + error;
		
		if (error > data.error_max) {
			data.error_max = error;
		}
		
		data.error_prev = error;
		
		//LCD.drawString("e_max = " + data.error_max, 0, 5);
		
		double result = error * data.param_p + derivate * data.param_d + data.integral * data.param_i;
		
		data.millis_prev = System.currentTimeMillis();

		return result;
	}
}
