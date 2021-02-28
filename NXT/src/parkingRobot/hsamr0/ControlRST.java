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

	enum State_SetPose {
		TURN_IN_DIRECTION, DRIVE_IN_DIRECTION, TURN_TO_HEADING, IDLE
	}

	private double DIFF_HEADING_MAX = 0.5;
	private double DIFF_DISTANCE_MAX = 0.02;

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

	Pose pose_start = new Pose();
	Pose currentPosition = new Pose();
	Pose pose_destination = new Pose();

	double ms_requiredForPath = 0;

	ControlMode currentCTRLMODE = null;
	State_SetPose state_setPose = State_SetPose.IDLE;

	EncoderSensor controlRightEncoder = null;
	EncoderSensor controlLeftEncoder = null;

	ParkingPath path_park = null;

	long lastTime = 0;

	double currentDistance = 0.0;
	double Distance = 0.0;

	double radius_tire_mm = 280; // previously 225
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
		monitor.addControlVar("motor_right");
		monitor.addControlVar("motor_left");

		this.ctrlThread = new ControlThread(this);

		ctrlThread.setPriority(Thread.MAX_PRIORITY - 1);
		ctrlThread.setDaemon(true); // background thread that is not need to terminate in order for the user program
									// to terminate
		ctrlThread.start();
	}

	// Inputs

	/**
	 * Sets velocity for parking, setPose and line follow functions.
	 * 
	 * @see parkingRobot.IControl#setVelocity(double velocity)
	 */
	public void setVelocity(double velocity) {
		this.velocity = velocity;

		drive(this.velocity, this.angularVelocity);

	}

	/**
	 * Sets angular velocity for VW-Control mode. This method has no effect on all
	 * other control modes.
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
	 * Sets destination for park control. x and y are in [m].
	 * 
	 * y is oriented straight ahead. x is oriented in left direction, facing away
	 * from y.
	 * 
	 * @see parkingRobot.IControl#setDestination(double heading, double x, double y)
	 */
	public void setDestination(double heading, double x, double y) {
		this.pose_destination.setHeading((float) heading);
		this.pose_destination.setLocation((float) -x, (float) y);

		// path_park = ParkingPath.withEnd(0.65/0.1 * x, 3.5 / 0.5 * y);

		path_park = ParkingPath.withEndAndVelocity(x + 0.03, y + 0.03, velocity);
	}

	/**
	 * Sets destination pose for straight line movement. x and y are in [m].
	 * 
	 * x is oriented straight ahead. y is oriented in left direction, facing away
	 * from x.
	 * 
	 * @see parkingRobot.IControl#setPose(Pose currentPosition)
	 */
	public void setPose(Pose pose) {
		// Pose can't be assigned directly because of Java's way of copying by reference
		pose_destination = new Pose(pose.getX(), pose.getY(), pose.getHeading());
		pose_start = new Pose(navigation.getPose().getX(), navigation.getPose().getY(),
				navigation.getPose().getHeading());

		state_setPose = State_SetPose.TURN_IN_DIRECTION;
	}

	/**
	 * Sets control mode.
	 * 
	 * All parameters (velocity, angular velocity, destination) have to be specified
	 * before this method is executed.
	 */
	public void setCtrlMode(ControlMode ctrl_mode) {
		v0 = velocity;
		w0 = angularVelocity;
		
		setStartTime(System.currentTimeMillis());
		
		data_left.integral = 0;
		data_right.integral = 0;
		data_sensor.integral = 0;

		this.currentCTRLMODE = ctrl_mode;
	}

	/**
	 * set start time
	 */
	public void setStartTime(long startTime) {
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
			break;
		case SETPOSE:
			exec_SETPOSE_ALGO();
			// update_smoothVelocity();
			break;
		case PARK_CTRL:
			exec_PARKCTRL_ALGO();
			break;
		case INACTIVE:
			exec_INACTIVE();
			break;
		}

		if (currentCTRLMODE != ControlMode.INACTIVE) {
			update_revs();
		}

	}

	long ms_start = 0;
	double tau = 500;

	double v0 = 0;
	double w0 = 0;

	private void update_smoothVelocity() {
		if (v0 == 0) {
			v0 = velocity;
			ms_start = System.currentTimeMillis();
		}

		velocity = v0 * (1 - Math.exp(-(double) (System.currentTimeMillis() - ms_start) / tau));

		// LCD.drawString("v_smooth: " + velocity, 0, 6);
	}

	// PIDData structs have to be initialized here so that their integral value
	// stays constant throughout
	// multiple method calls.

	PIDData data_right = PIDData.pi(0, 0, 40, 50);
	PIDData data_left = PIDData.pi(0, 0, 40, 50);

	/**
	 * Controls wheel speed based on a PI algorithm. This function is used for
	 * movement in all control modes.
	 */

	private void update_revs() {
		// Measure angle difference and time delta between measurements
		AngleDifferenceMeasurement meas_left = perception.getControlLeftEncoder().getEncoderMeasurement();
		AngleDifferenceMeasurement meas_right = perception.getControlRightEncoder().getEncoderMeasurement();

		// Calculate angular velocities
		double w_meas_left = meas_left.getAngleSum() / (meas_left.getDeltaT());
		double w_meas_right = meas_right.getAngleSum() / (meas_right.getDeltaT());

		monitor.writeControlVar("motor_right", w_meas_right + "");
		monitor.writeControlVar("motor_left", w_meas_left + "");

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
		leftMotorPower = (int) PIDController.pi_ctrl(data_left);
		rightMotorPower = (int) PIDController.pi_ctrl(data_right);

		leftMotor.setPower(leftMotorPower);
		rightMotor.setPower(rightMotorPower);
	}

	PIDData data_lat = PIDData.pid(0, 0, 5, 0, 5);

	private void update_lateralControl() {
		
		if (System.currentTimeMillis() - lastTime < 1000) {
			//return;
		}

		//double length_startToNow = pose_start.distanceTo(navigation.getPose().getLocation());
		double length_startToNow = pose_destination.distanceTo(navigation.getPose().getLocation());
		double angle_endToNow = pose_destination.angleTo(navigation.getPose().getLocation());

		double e_lat = length_startToNow * Math.sin(angle_endToNow);

		data_lat.processVariable = e_lat;

		setAngularVelocity(PIDController.pd_ctrl(data_lat));
	}

	/**
	 * update parameters during LINE Control Mode
	 */
	private void update_LINECTRL_Parameter() {
		this.lineSensorRight = perception.getRightLineSensor();
		this.lineSensorLeft = perception.getLeftLineSensor();
	}

	double distance_prev = 0;
	
	private void exec_SETPOSE_ALGO() {
		double angl_startToDest = 0;
		double angl_dest = 0;
		double angl_current = 0;

		switch (state_setPose) {
		case TURN_IN_DIRECTION:
			angl_startToDest = pose_start.angleTo(pose_destination.getLocation());
			angl_current = Math.toDegrees(navigation.getPose().getHeading());

			// Turn as long as difference of current angle and destination heading is too
			// large.
			// Angular velocity is set based on which way to turn is shorter

			if (angl_startToDest - angl_current > DIFF_HEADING_MAX) {
				setAngularVelocity(w0);
				setVelocity(0);
			} else if (angl_current - angl_startToDest > DIFF_HEADING_MAX) {
				setAngularVelocity(-w0);
				setVelocity(0);
			} else {
				state_setPose = State_SetPose.DRIVE_IN_DIRECTION;

				ms_start = System.currentTimeMillis();

				setAngularVelocity(0);
				setVelocity(v0);
				
				distance_prev = navigation.getPose().distanceTo(pose_destination.getLocation());
			}
			

			LCD.drawString("Dest: " + angl_startToDest, 0, 4);
			LCD.drawString("Cur:" + angl_current, 0, 5);

			break;
		case DRIVE_IN_DIRECTION:
			// Safeguard in case of incorrectly set heading at the sequence's beginning
			ms_requiredForPath = pose_start.distanceTo(pose_destination.getLocation()) / v0 * 1000;
			
			angl_startToDest = pose_start.angleTo(pose_destination.getLocation());
			angl_current = Math.toDegrees(navigation.getPose().getHeading());

			// Turn as long as difference of current angle and destination heading is too
			// large.
			// Angular velocity is set based on which way to turn is shorter

			if (angl_startToDest - angl_current > DIFF_HEADING_MAX) {
				//setAngularVelocity(w0);
			} else if (angl_current - angl_startToDest > DIFF_HEADING_MAX) {
				//setAngularVelocity(-w0);
			}

			// Drive straight ahead as long as distance is too large or as long as required
			// time for making the distance
			
			double distance = navigation.getPose().distanceTo(pose_destination.getLocation());
			
			//if (navigation.getPose().distanceTo(pose_destination.getLocation()) <= DIFF_DISTANCE_MAX) {
			
			//LCD.drawString("dis:  " + distance, 0, 4);
			//LCD.drawString("prev: " + distance_prev, 0, 5);
			
			if (distance > distance_prev) {
				state_setPose = State_SetPose.TURN_TO_HEADING;

				ms_start = System.currentTimeMillis();

				setVelocity(0);
			}/* else if (System.currentTimeMillis() - ms_start > ms_requiredForPath) {
				state_setPose = State_SetPose.TURN_TO_HEADING;

				ms_start = System.currentTimeMillis();

				setVelocity(0);
			}*/
			
			if (distance < distance_prev && pose_start.distanceTo(navigation.getPose().getLocation()) > 0.2) {
				distance_prev = distance;
			}

			/*
			 * LCD.clear(); Test_SetPose.showData(navigation, perception);
			 * LCD.drawString("dis=" +
			 * pose_start.distanceTo(pose_destination.getLocation()), 0, 4);
			 * LCD.drawString("xd="+pose_destination.getX(), 0, 5);
			 * LCD.drawString("t="+ms_requiredForPath, 0, 6);
			 */
			// TODO: Implement safeguard in case destination is badly missed. For example by
			// setting a max wait time that is proportional to velocity

			update_lateralControl();

			break;
		case TURN_TO_HEADING:

			angl_dest = pose_destination.getHeading();
			angl_current = Math.toDegrees(navigation.getPose().getHeading()) % 360;

			/*
			 * if (Math.abs(angl_dest - angl_current) <= DIFF_HEADING_MAX) { state_setPose =
			 * State_SetPose.IDLE; setAngularVelocity(0);
			 * 
			 * setVelocity(0); }
			 */

			if (angl_dest - angl_current >= DIFF_HEADING_MAX) {
				setAngularVelocity(w0);
				LCD.drawString(">", 0, 6);
			} else if (angl_current - angl_dest >= DIFF_HEADING_MAX) {
				setAngularVelocity(-w0);
				LCD.drawString("<", 0, 6);
			} else {
				state_setPose = State_SetPose.IDLE;
				
				setAngularVelocity(0);
				setVelocity(0);
				
				Test_Vert2_1.notify_setPose_ready();
			}
			

			LCD.drawString("Dest: " + angl_dest, 0, 4);
			LCD.drawString("Cur:" + angl_current, 0, 5);

			/*
			 * LCD.clear(); Test_SetPose.showData(navigation, perception);
			 * LCD.drawString("agl_d=" + angl_dest, 0, 4); LCD.drawString("agl_c=" +
			 * angl_current, 0, 5);
			 */

			break;

		default:
			/*
			 * LCD.clear(); Test_SetPose.showData(navigation, perception);
			 * LCD.drawString("IDLE", 0, 4);
			 */
			setAngularVelocity(0);

			break;
		}

		LCD.drawString("State:" + state_setPose.name(), 0, 3);
		Test_SetPose.showData(navigation, perception);
	}

	/**
	 * PARKING along the generated path
	 */
	private void exec_PARKCTRL_ALGO() {
		double t = (double) (System.currentTimeMillis() - lastTime) / 1000d;

		if (path_park != null) {

			if (path_park.getEndT() >= 0) {
				setVelocity(path_park.calc_v(t));

				if (path_park.getEndX() >= 0) {
					setAngularVelocity(-path_park.calc_w(t));
				} else {
					setAngularVelocity(path_park.calc_w(t));
				}
			} else {
				setVelocity(-path_park.calc_v(t));

				if (path_park.getEndX() >= 0) {
					setAngularVelocity(path_park.calc_w(t));
				} else {
					setAngularVelocity(-path_park.calc_w(t));
				}
			}

			LCD.drawString("w = " + getAngularVelocity(), 0, 0);

			LCD.drawString("v = " + path_park.calc_v(t), 0, 1);
			LCD.drawString("x = " + path_park.calc_x(t), 0, 2);

			if (t > Math.abs(path_park.T) / path_park.getVelocity()) {
				path_park = null;

				setVelocity(0);
				setAngularVelocity(0);

				setCtrlMode(ControlMode.INACTIVE);
				Test_PathFollow.ctrl_ready = true;

			}
		}
	}

	private void exec_INACTIVE() {
		this.stop();

	}

	double vel_line = 0.1;
	PIDData data_sensor = PIDData.pid(0, 0, 1, 0.0, 1);

	// PIDData data_sensor = PIDData.pid(0, 0, 0.8, 0.0, 1);
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
		double val_right = perception.getRightLineSensorValue();

		double vel_ang = getAngularVelocity();

		data_sensor.processVariable = val_left - val_right;

		LCD.drawString(val_left + " - " + val_right, 0, 4);
		LCD.drawString("e = " + data_sensor.processVariable, 0, 5);

		vel_ang = PIDController.pid_ctrl(data_sensor);

		setAngularVelocity(vel_ang);
		
		Test_Vert2_1.showData(navigation, perception);
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