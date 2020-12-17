package parkingRobot.hsamr0;

import lejos.robotics.PressureDetector;
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
	
	ParkingPath path_park = null;

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
		// monitor.addControlVar("RightSensor");
		// monitor.addControlVar("LeftSensor");

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
		
		path_park = ParkingPath.withEnd(0.65/0.1 * x, 3.5 / 0.5 * y);
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
			exec_LINECTRL_ALGO_line();
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

	private void update_VWCTRL_Parameter() {
		setPose(navigation.getPose());
	}

	// PIDData structs have to be initialized here so that their integral value
	// stays constant throughout
	// multiple method calls.

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

		// Had to add this, otherwise the robot wouldn't start driving for some reason.
		// I guess it could be related to threading.
		try {
			Thread.sleep(10);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

		// Adjust motor power based on calculated data
		leftMotorPower += PIDController.pi_ctrl(data_left);
		rightMotorPower += PIDController.pi_ctrl(data_right);

		leftMotor.setPower(leftMotorPower);
		rightMotor.setPower(rightMotorPower);
	}

	/**
	 * update parameters during SETPOSE Control Mode
	 */
	private void update_SETPOSE_Parameter() {
		setPose(navigation.getPose());
	}

	/**
	 * update parameters during LINE Control Mode
	 */
	private void update_LINECTRL_Parameter() {
		this.lineSensorRight = perception.getRightLineSensor();
		this.lineSensorLeft = perception.getLeftLineSensor();
	}

	/**
	 * update parameters during PARKING Control Mode
	 */
	private void update_PARKCTRL_Parameter() {
		// Aufgabe 3.4
	}

	/**
	 * The car can be driven with velocity in m/s or angular velocity in grade
	 * during VW Control Mode optionally one of them could be set to zero for simple
	 * test.
	 */
	private void exec_VWCTRL_ALGO() {

	}

	private void exec_SETPOSE_ALGO() {
		// Aufgabe 3.3
	}

	/**
	 * PARKING along the generated path
	 */
	private void exec_PARKCTRL_ALGO() {
		double t = (double) (System.currentTimeMillis() - lastTime) / 1000d;
		
		if (path_park != null) {
			
			setVelocity(path_park.calc_v(t));
			setAngularVelocity(-path_park.calc_w(t));
			
			LCD.drawString("w = " + getAngularVelocity(), 0, 0);
			
			LCD.drawString("v = " + path_park.calc_v(t), 0, 1);
			LCD.drawString("x = " + path_park.calc_x(t), 0, 2);
			
			if (t > path_park.T) {
				 path_park = null;
				
				setVelocity(0);
				setAngularVelocity(0);
			}
		}
	}

	private void exec_INACTIVE() {
		this.stop();

	}

	double vel_line = 0.1;
	PIDData data_sensor = PIDData.pid(0, 0, 0.8, 0.0, 2);

	/**
	 * Executes the line follow algorithm based on a PID controller.
	 * 
	 * Process Variable is the difference between right and left color sensor value.
	 * The PID controller calculates a resulting angular velocity which then is
	 * handled by VW-Control.
	 */

	public void exec_LINECTRL_ALGO_line() {
		setVelocity(vel_line);

		leftMotor.forward();
		rightMotor.forward();

		double val_left = perception.getLeftLineSensorValue();
		double val_right = perception.getRightLineSensorValue() + 5;

		double vel_ang = getAngularVelocity();

		data_sensor.processVariable = val_left - val_right;

		LCD.drawString(val_left + " - " + val_right, 0, 4);
		LCD.drawString("e = " + data_sensor.processVariable, 0, 5);

		vel_ang = PIDController.pid_ctrl(data_sensor);

		setAngularVelocity(vel_ang);
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

	private int getPowerForW_M1(double w) {
		return (int) (1 / 8.24 * (w + 50));
	}

	private int getPowerForW_M2(double w) {
		return (int) (1 / 8.67 * (w + 8));
	}
}