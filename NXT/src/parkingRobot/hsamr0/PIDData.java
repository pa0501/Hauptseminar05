package parkingRobot.hsamr0;

public class PIDData {
	public double setpoint = 0;
	public double processVariable = 0;
	public double param_p = 0;
	public double param_i = 0;
	public double param_d = 0;
	
	public double output_max = 0;
	public double i_max = 0;
	public boolean cap_output, cap_i;
	
	public double integral = 0;
	
	public double error_prev = 0;
	public long millis_prev = 0;
	
	public double error_max = 0;

	public PIDData() {
		millis_prev = System.currentTimeMillis();
	}
	
	public PIDData cap_output(double max) {
		cap_output = true;
		output_max = max;
		
		return this;
	}
	
	public PIDData cap_i() {
		cap_i = true;
		
		return this;
	}
	
	public PIDData cap_i(double max) {
		i_max = max;
		cap_i = true;
		
		return this;
	}
	
	public static PIDData pi(double sp, double pv, double kp, double ki) {
		PIDData result = new PIDData();
		
		result.setpoint = sp;
		result.processVariable = pv;
		result.param_p = kp;
		result.param_i = ki;
		
		return result;
	}
	
	public static PIDData pd(double sp, double pv, double kp, double kd) {
		PIDData result = new PIDData();
		
		result.setpoint = sp;
		result.processVariable = pv;
		result.param_p = kp;
		result.param_d = kd;
		
		return result;
	}
	
	public static PIDData pid(double sp, double pv, double kp, double ki, double kd) {
		PIDData result = new PIDData();
		
		result.setpoint = sp;
		result.processVariable = pv;
		result.param_p = kp;
		result.param_i = ki;
		result.param_d = kd;
		
		return result;
	}
}