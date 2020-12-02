package parkingRobot.hsamr0;

public class PIDData {
	public double setpoint = 0;
	public double processVariable = 0;
	public double param_p = 0;
	public double param_i = 0;
	public double param_d = 0;
	
	public double integral = 0;
	
	public double error_prev = 0;
	public long millis_prev = 0;
	
	public double error_max = 0;

	public PIDData() {
		millis_prev = System.currentTimeMillis();
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