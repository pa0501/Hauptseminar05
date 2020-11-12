package parkingRobot.hsamr0;

public class PIDData {
	public double setpoint = 0;
	public double processVariable = 0;
	public double param_p = 0;
	public double param_i = 0;
	
	public double integral = 0;
	
	public PIDData(double sp, double pv, double kp, double ki) {
		setpoint = sp;
		processVariable = pv;
		param_p = kp;
		param_i = ki;
	}
}