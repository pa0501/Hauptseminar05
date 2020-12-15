package parkingRobot.hsamr0;

import lejos.robotics.navigation.Pose;
import parkingRobot.IControl;
import parkingRobot.IMonitor;
import parkingRobot.IPerception;
import parkingRobot.IPerception.*;
import parkingRobot.hsamr0.util.Average;
import lejos.nxt.Button;
import lejos.nxt.LCD;
import lejos.nxt.Motor;
import lejos.nxt.MotorPort;
import lejos.nxt.NXTMotor;
import lejos.nxt.Sound;
import parkingRobot.INavigation;

/**
 * Main class for control module
 *
 */
public class ControlRST implements IControl {

	enum Direction {
		RIGHT, LEFT, NONE
	}

	enum CTRL_ALGO_STATE {
		LINE, CORNER
	}

	/**
	 * reference to {@link IPerception.EncoderSensor} class for left robot wheel
	 * which measures the wheels angle difference between actual an last request
	 */
	IPerception.EncoderSensor encoderLeft = null;
	/**
	 * reference to {@link IPerception.EncoderSensor} class for right robot wheel
	 * which measures the wheels angle difference between actual an last request
	 */
	IPerception.EncoderSensor encoderRight = null;

	/**
	 * reference to data class for measurement of the left wheels angle difference
	 * between actual an last request and the corresponding time difference
	 */
	IPerception.AngleDifferenceMeasurement angleMeasurementLeft = null;
	/**
	 * reference to data class for measurement of the right wheels angle difference
	 * between actual an last request and the corresponding time difference
	 */
	IPerception.AngleDifferenceMeasurement angleMeasurementRight = null;

	/**
	 * line information measured by right light sensor: 0 - beside line, 1 - on line
	 * border or gray underground, 2 - on line
	 */
	int lineSensorRight = 0;
	/**
	 * line information measured by left light sensor: 0 - beside line, 1 - on line
	 * border or gray underground, 2 - on line
	 */
	int lineSensorLeft = 0;

	NXTMotor leftMotor = null;
	NXTMotor rightMotor = null;

	IPerception perception = null;
	INavigation navigation = null;
	IMonitor monitor = null;
	ControlThread ctrlThread = null;

	int leftMotorPower = 0;
	int rightMotorPower = 0;

	// Current angular velocity of both motors in deg/ms

	double w_motor_left = 0;
	double w_motor_right = 0;

	double velocity = 0.0;
	double angularVelocity = 0.0;

	Pose startPosition = new Pose();
	Pose currentPosition = new Pose();
	Pose destination = new Pose();

	ControlMode currentCTRLMODE = null;

	EncoderSensor controlRightEncoder = null;
	EncoderSensor controlLeftEncoder = null;

	int lastTime = 0;

	double currentDistance = 0.0;
	double Distance = 0.0;

	double radius_tire_mm = 225;
	double distance_tires_mm = 120;

	/**
	 * provides the reference transfer so that the class knows its corresponding
	 * navigation object (to obtain the current position of the car from) and starts
	 * the control thread.
	 * 
	 * @param perception corresponding main module Perception class object
	 * @param navigation corresponding main module Navigation class object
	 * @param monitor    corresponding main module Monitor class object
	 * @param leftMotor  corresponding NXTMotor object
	 * @param rightMotor corresponding NXTMotor object
	 */
	public ControlRST(IPerception perception, INavigation navigation, NXTMotor leftMotor, NXTMotor rightMotor,
			IMonitor monitor) {
		this.perception = perception;
		this.navigation = navigation;
		this.monitor = monitor;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;

		this.currentCTRLMODE = ControlMode.INACTIVE;

		this.encoderLeft = perception.getControlLeftEncoder();
		this.encoderRight = perception.getControlRightEncoder();
		this.lineSensorRight = perception.getRightLineSensor();
		this.lineSensorLeft = perception.getLeftLineSensor();

		// MONITOR (example)
		//monitor.addControlVar("RightSensor");
		//monitor.addControlVar("LeftSensor");

		this.ctrlThread = new ControlThread(this);

		ctrlThread.setPriority(Thread.MAX_PRIORITY - 1);
		ctrlThread.setDaemon(true); // background thread that is not need to terminate in order for the user program
									// to terminate
		ctrlThread.start();
	}

	// Inputs

	/**
	 * set velocity
	 * 
	 * @see parkingRobot.IControl#setVelocity(double velocity)
	 */
	public void setVelocity(double velocity) {
		this.velocity = velocity;

		drive(this.velocity, this.angularVelocity);

	}

	/**
	 * set angular velocity
	 * 
	 * @see parkingRobot.IControl#setAngularVelocity(double angularVelocity)
	 */
	public void setAngularVelocity(double angularVelocity) {
		this.angularVelocity = angularVelocity;

		drive(this.velocity, this.angularVelocity);

	}

	public double getAngularVelocity() {
		return this.angularVelocity;
	}

	/**
	 * set destination
	 * 
	 * @see parkingRobot.IControl#setDestination(double heading, double x, double y)
	 */
	public void setDestination(double heading, double x, double y) {
		this.destination.setHeading((float) heading);
		this.destination.setLocation((float) x, (float) y);
	}

	/**
	 * sets current pose
	 * 
	 * @see parkingRobot.IControl#setPose(Pose currentPosition)
	 */
	public void setPose(Pose currentPosition) {
		// TODO Auto-generated method stub
		this.currentPosition = currentPosition;
	}

	/**
	 * set control mode
	 */
	public void setCtrlMode(ControlMode ctrl_mode) {
		this.currentCTRLMODE = ctrl_mode;
	}

	/**
	 * set start time
	 */
	public void setStartTime(int startTime) {
		this.lastTime = startTime;
	}

	/**
	 * selection of control-mode
	 * 
	 * @see parkingRobot.IControl#exec_CTRL_ALGO()
	 */
	public void exec_CTRL_ALGO() {

		switch (currentCTRLMODE) {
		case LINE_CTRL:
			update_LINECTRL_Parameter();
			exec_LINECTRL_ALGO();
			// exec_LINECTRL_ALGO_line();
			break;
		case VW_CTRL:
			update_VWCTRL_Parameter();
			exec_VWCTRL_ALGO();
			break;
		case SETPOSE:
			update_SETPOSE_Parameter();
			exec_SETPOSE_ALGO();
			break;
		case PARK_CTRL:
			update_PARKCTRL_Parameter();
			exec_PARKCTRL_ALGO();
			break;
		case INACTIVE:
			exec_INACTIVE();
			break;
		}

		update_revs();
		
		

	}

	// Private methods

	/**
	 * update parameters during VW Control Mode
	 */

	double kp = 20;

	// kp = 60, ki = 1.05

	// PIDData structs have to be initialized here so that their integral
	// value stays constant.

	private void update_VWCTRL_Parameter() {
		setPose(navigation.getPose());

	}

	PIDData data_right = PIDData.pi(0, 0, 60, 1.05);
	PIDData data_left = PIDData.pi(0, 0, 60, 1.05);

	private void update_revs() {
		setPose(navigation.getPose());

		// Measure angle difference and time delta between measurements
		AngleDifferenceMeasurement meas_left = perception.getControlLeftEncoder().getEncoderMeasurement();
		AngleDifferenceMeasurement meas_right = perception.getControlRightEncoder().getEncoderMeasurement();

		// Calculate angular velocities
		double w_meas_left = meas_left.getAngleSum() / (meas_left.getDeltaT());
		double w_meas_right = meas_right.getAngleSum() / (meas_right.getDeltaT());

		// Set PID Data
		data_right.setpoint = w_motor_right;
		data_right.processVariable = w_meas_right;

		data_left.setpoint = w_motor_left;
		data_left.processVariable = w_meas_left;

		//LCD.drawString("wl " + w_motor_left, 0, 4);
		
		try {
			Thread.sleep(10);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		// Hallo

		//LCD.drawString("wl = " + w_meas_left, 0, 0);
		//LCD.drawString("wr = " + w_meas_right, 0, 1);

		///LCD.drawString("wl,sp = " + data_left.setpoint, 0, 3);
		//LCD.drawString("wr,sp = " + data_right.setpoint, 0, 4);

		// LCD feedback for debugging
		// LCD.drawString("WR 0." + (int) (w_meas_right * 100) + " 0." + (int)
		// (w_motor_right * 100), 0, 0);
		// LCD.drawString("WL 0." + (int) (w_meas_left * 100) + " 0." + (int)
		// (w_motor_left * 100), 0, 1);
		



		// Adjust motor power based on calculated data
		leftMotorPower += PIDController.pi_ctrl(data_left);
		rightMotorPower += PIDController.pi_ctrl(data_right);

		leftMotor.setPower(leftMotorPower);
		rightMotor.setPower(rightMotorPower);
	}

	int btn_right_prev = 0;
	int btn_left_prev = 0;
	int btn_enter_prev = 0;
	int btn_back_prev = 0;

	int selection = 0; // 0 - none; 1 - velocity; 2 - angular;
	int cursor = 1; // 0 - none; 1 - velocity; 2 - angular;

	// GUI - Nicht in der Aufgabe gefordert!

	public void menu_update() {
		if (Button.ENTER.isDown()) {
			if (btn_enter_prev == 1) {
				btn_enter_prev = 0;
				return;
			} else {
				btn_enter_prev = 1;
			}
		}

		if (Button.LEFT.isDown()) {
			if (btn_left_prev == 1) {
				btn_left_prev = 0;
				return;
			} else {
				btn_left_prev = 1;
			}
		}

		if (Button.RIGHT.isDown()) {
			if (btn_right_prev == 1) {
				btn_right_prev = 0;
				return;
			} else {
				btn_right_prev = 1;
			}
		}

		if (Button.ENTER.isDown()) {
			if (cursor == 0) {
				cursor = selection;
				selection = 0;
			} else {
				selection = cursor;
				cursor = 0;
			}
		} else if (Button.RIGHT.isDown()) {
			if (cursor == 1) {
				cursor = 2;
			} else if (cursor == 2) {
				cursor = 1;
			}

			if (selection == 1) {
				velocity += 5;
			} else if (selection == 2) {
				angularVelocity += 0.1;
			}
		} else if (Button.LEFT.isDown()) {
			if (cursor == 1) {
				cursor = 2;
			} else if (cursor == 2) {
				cursor = 1;
			}

			if (selection == 1) {
				velocity -= 5;
			} else if (selection == 2) {
				angularVelocity -= 0.1;
			}
		}

		if (cursor == 1) {
			LCD.drawString("> ", 0, 5);
		} else if (cursor == 2) {
			LCD.drawString("> ", 0, 6);
		}
	}

	/**
	 * update parameters during SETPOSE Control Mode
	 */
	private void update_SETPOSE_Parameter() {
		setPose(navigation.getPose());
	}

	/**
	 * update parameters during PARKING Control Mode
	 */
	private void update_PARKCTRL_Parameter() {
		// Aufgabe 3.4
	}

	/**
	 * update parameters during LINE Control Mode
	 */
	private void update_LINECTRL_Parameter() {
		this.lineSensorRight = perception.getRightLineSensor();
		this.lineSensorLeft = perception.getLeftLineSensor();
	}

	/**
	 * The car can be driven with velocity in m/s or angular velocity in grade
	 * during VW Control Mode optionally one of them could be set to zero for simple
	 * test.
	 */
	private void exec_VWCTRL_ALGO() {
		// this.drive(this.velocity, this.angularVelocity);

		// w_motor_left = 0.36;
	}

	private void exec_SETPOSE_ALGO() {
		// Aufgabe 3.3
	}

	/**
	 * PARKING along the generated path
	 */
	private void exec_PARKCTRL_ALGO() {
		// Aufgabe 3.4
	}

	private void exec_INACTIVE() {
		this.stop();

	}

	Average avg_sensor_right = new Average(10, true);
	Average avg_sensor_left = new Average(10, true);

	// Average avg_sensor_right = new Average(20, true);
	// Average avg_sensor_left = new Average(20, true);

	int a = 0;

	private Direction LINECTRL_checkForChange(double val_right, double val_left) {

		/*
		 * if (a % 1 == 0) { avg_sensor_right.addValue(val_right);
		 * avg_sensor_left.addValue(val_left);
		 * 
		 * avg_sensor_right.addValue(val_right); avg_sensor_left.addValue(val_left);
		 * 
		 * double avg_r_2 = avg_sensor_right.getAverageOfLastN(1); double avg_r_15 =
		 * avg_sensor_right.getAverageInRange(0, 5);
		 * 
		 * double avg_l_2 = avg_sensor_left.getAverageOfLastN(1); double avg_l_15 =
		 * avg_sensor_left.getAverageInRange(0, 5);
		 * 
		 * LCD.drawString("r(2):  " + avg_r_2, 0, 0); LCD.drawString("r(10): " +
		 * avg_r_15, 0, 1);
		 * 
		 * LCD.drawString("l(2):  " + avg_l_2, 0, 3); LCD.drawString("l(10): " +
		 * avg_l_15, 0, 4);
		 * 
		 * // 50 oder 60 if (avg_l_15 - avg_l_2 > 50) { //Sound.buzz();
		 * 
		 * return Direction.LEFT; }
		 * 
		 * if (Math.abs(avg_r_15 - avg_r_2) > 50) { //Sound.beep();
		 * 
		 * return Direction.RIGHT; }
		 * 
		 * }
		 * 
		 * a++;
		 */

		if (data_sensor_right.error_prev > 90) {
			return Direction.RIGHT;
		}

		if (data_sensor_left.error_prev > 90) {
			return Direction.LEFT;
		}

		return Direction.NONE;
	}

	CTRL_ALGO_STATE LINE_STATE = CTRL_ALGO_STATE.LINE;

	double vel_line = 0.1;

	public void exec_LINECTRL_ALGO() {
		
		
		double val_left = perception.getLeftLineSensorValue();
		double val_right = perception.getRightLineSensorValue();

		setVelocity(vel_line);
		
		if (LINECTRL_checkForChange(val_right, val_left) != Direction.NONE) {
			kp_f = 0.1;
			kp_d = 15;
		} else {
			kp_f = 2;
			kp_d = 0;
		}
		
		

		switch (LINE_STATE) {
		case LINE:

			exec_LINECTRL_ALGO_line();

			Direction cornerat = LINECTRL_checkForChange(val_right, val_left);

			/*if (cornerat != Direction.NONE) {
				if (cornerat == Direction.RIGHT) {
					w_off_l = 0.6;
				} else if (cornerat == Direction.LEFT) {
					w_off_r = 0.6;
				}
				ms_corner_start = System.currentTimeMillis();
				LINE_STATE = CTRL_ALGO_STATE.CORNER;
				
				step = 0;
				vel_line = this.velocity;
				// drive(0, 0);
			}*/
			
			

			break;

		case CORNER:

			exec_LINECTRL_ALGO_corner();

			break;

		default:
			break;
		}
	}

	// PIDData data_sensor_right = PIDData.pid(95, 0, 0.6, 0.000, 10);
	// PIDData data_sensor_left = PIDData.pid(95, 0, 0.6, 0.000, 10);

	//PIDData data_sensor_right = PIDData.pid(95, 0, 1, 0.000, 15);
	//PIDData data_sensor_left = PIDData.pid(95, 0, 1, 0.000, 15);
	
	double kp_f = 2;
	double kp_d = 15;
	
	//PIDData data_sensor_right = PIDData.pid(95, 0, 2, 0.000, 15);
	//PIDData data_sensor_left = PIDData.pid(95, 0, 2, 0.000, 15);
	
	//PIDData data_sensor_right = PIDData.pid(95, 0, kp_f, 0.000, kp_d);
	//PIDData data_sensor_left = PIDData.pid(95, 0, kp_f, 0.000, kp_d);
	
	PIDData data_sensor_right = PIDData.pid(95, 0, 1.5, 0.000, 30);
	PIDData data_sensor_left = PIDData.pid(95, 0, 1.5, 0.000, 30);

	public void exec_LINECTRL_ALGO_line() {
		leftMotor.forward();
		rightMotor.forward();

		double val_left = perception.getLeftLineSensorValue();
		double val_right = perception.getRightLineSensorValue();

		// LINECTRL_checkForChange(val_right, val_left);

		// MONITOR (example)
		//monitor.writeControlVar("LeftSensor", "" + this.lineSensorLeft);
		//monitor.writeControlVar("RightSensor", "" + this.lineSensorRight);

		LCD.drawString(perception.getRightLineSensorValue() + "  ", 0, 0);
		LCD.drawString(perception.getLeftLineSensorValue() + "  ", 0, 1);

		data_sensor_right.processVariable = perception.getRightLineSensorValue();
		data_sensor_left.processVariable = perception.getLeftLineSensorValue();

		double vel_ang = getAngularVelocity();

		double delta_pos = PIDController.pid_ctrl(data_sensor_right);
		double delta_neg = PIDController.pid_ctrl(data_sensor_left);

		vel_ang += delta_pos;
		vel_ang -= delta_neg;
		

		if (delta_pos > 0) {
			LCD.drawString("->", 5, 5);
		} else if (delta_pos < 0) {
			LCD.drawString("<-", 5, 5);
		} else if (delta_pos == 0) {
			LCD.drawString("  ", 5, 5);
		}

		if (delta_neg < 0) {
			LCD.drawString("->", 0, 5);
		} else if (delta_neg > 0) {
			LCD.drawString("<-", 0, 5);
		} else if (delta_neg == 0) {
			LCD.drawString("  ", 0, 5);
		}

		if (vel_ang > 0) {
			LCD.drawString(">", 3, 5);
		} else if (vel_ang < 0) {
			LCD.drawString("<", 3, 5);
		} else if (vel_ang == 0) {
			LCD.drawString("^", 3, 5);
		}

		if (val_left > 50 && val_left > 50) {

		}

		LCD.drawString("l: " + val_left, 0, 6);
		LCD.drawString("r: " + val_right, 0, 7);

		setAngularVelocity(-vel_ang);

		// avg_sensor_left.addValue(val_left);
		// avg_sensor_right.addValue(val_right);
	}

	/**
	 * DRIVING along black line Minimalbeispiel Linienverfolgung fuer gegebene Werte
	 * 0,1,2 white = 0, black = 2, grey = 1
	 */

	long ms_lastChange = 0;
	long ms_corner_start = 0;

	double w_off_l = 0;
	double w_off_r = 0;

	private void exec_LINECTRL_ALGO_corner_new() {

	}
	
	int step = 0;
	int cooldown_both = 0;
	
	double w_motor_left_prev = 0;
	double w_motor_right_prev = 0;

	private void exec_LINECTRL_ALGO_corner() {

		leftMotor.forward();
		rightMotor.forward();
		int lowPower = 1;
		int highPower = 30;

		//double w_high = 0.2;
		//double w_low = -0.1;
		/*
		double w_high = 0.1;
		double w_low = -0.05;*/

		

		double w_high = 0.1;
		double w_low = -0.1;

		if (System.currentTimeMillis() - ms_corner_start < 400) {
			// w_motor_left = w_off_l;
			// w_motor_right = w_off_r;

			//w_high = 0.5;
			w_high = 0.5;
			w_low = -0.1;

		} else {
			if (step > 5 && step < 15) {
				w_high = w_high * Math.pow(0.9, step - 5);
				//w_low = w_low * Math.pow(0.9, step - 5);
			}
			
			
		}
		
		

		w_off_l = 0;
		w_off_r = 0;

		// MONITOR (example)
		//monitor.writeControlVar("LeftSensor", "" + this.lineSensorLeft);
		//monitor.writeControlVar("RightSensor", "" + this.lineSensorRight);

		vel_line = this.velocity;
		


		setVelocity(0.1);
		setAngularVelocity(0);
		this.drive(this.velocity, this.angularVelocity);

		double vel_ang = 0.6;

		if (this.lineSensorLeft >= 1 && this.lineSensorRight == 0) {
			w_motor_left = w_low;
			w_motor_right = w_high;
			
			w_motor_left_prev = w_motor_left;
			w_motor_right_prev = w_motor_right;
			
			step++;

			ms_lastChange = System.currentTimeMillis();
		} else if (this.lineSensorLeft == 0 && this.lineSensorRight >= 1) {
			w_motor_left = w_high;
			w_motor_right = w_low;
			
			w_motor_left_prev = w_motor_left;
			w_motor_right_prev = w_motor_right;
			
			step++;

			ms_lastChange = System.currentTimeMillis();
		} else if (this.lineSensorLeft >= 1 && this.lineSensorRight >= 1) {
			
			
			if (cooldown_both > 0) {
				w_motor_left = -w_motor_left_prev * 2;
				w_motor_right = -w_motor_right_prev * 2;
				
				cooldown_both = 5;
			}
		}
		
		cooldown_both--;
		
		if (cooldown_both == 0) {
			cooldown_both = 0;
		}
		
		

		if (System.currentTimeMillis() - ms_lastChange > 1000) {
			LINE_STATE = CTRL_ALGO_STATE.LINE;
			this.angularVelocity = 0;
			this.velocity = 0;

			return;
		}

	}

	private void stop() {
		this.leftMotor.stop();
		this.rightMotor.stop();
	}

	/**
	 * calculates the left and right angle speed of the both motors with given
	 * velocity and angle velocity of the robot
	 * 
	 * @param v [m/s] velocity of the robot
	 * @param w [deg/s] angle velocity of the robot
	 */

	private void drive(double v, double w) {

		
		double w_radPms = w * Math.PI / 180;
		double v_mmPs = v * 1000;

		double vr_mmPms = v_mmPs + w_radPms * distance_tires_mm / 2;
		double vl_mmPms = v_mmPs - w_radPms * distance_tires_mm / 2;

		double wr = vr_mmPms / (radius_tire_mm * 2);
		double wl = vl_mmPms / (radius_tire_mm * 2);

		w_motor_left = wl;
		w_motor_right = wr;

		leftMotor.forward();
		rightMotor.forward();
		

		

	}

	private void driveRaw(double v, double w) {
		double vr_mms = v + 1 / 2 * w * distance_tires_mm;
		double vl_mms = v - 1 / 2 * w * distance_tires_mm;

		double wr = vr_mms / radius_tire_mm;
		double wl = vl_mms / radius_tire_mm;

		rightMotor.setPower(getPowerForW_M1(wr));
		leftMotor.setPower(getPowerForW_M2(wl));

		// LCD.drawString(wr + " -> " + getPowerForW_M1(wr), 0, 0);
		// LCD.drawString(wl + " -> " + getPowerForW_M1(wl), 0, 1);
	}

	private int getPowerForW_M1(double w) {
		return (int) (1 / 8.24 * (w + 50));
	}

	private int getPowerForW_M2(double w) {
		return (int) (1 / 8.67 * (w + 8));
	}
}