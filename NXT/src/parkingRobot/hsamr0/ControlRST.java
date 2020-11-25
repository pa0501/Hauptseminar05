package parkingRobot.hsamr0;

import lejos.robotics.navigation.Pose;
import parkingRobot.IControl;
import parkingRobot.IMonitor;
import parkingRobot.IPerception;
import parkingRobot.IPerception.*;
import lejos.nxt.Button;
import lejos.nxt.LCD;
import lejos.nxt.NXTMotor;
import parkingRobot.INavigation;

/**
 * Main class for control module
 *
 */
public class ControlRST implements IControl {
	
	/**
	 * reference to {@link IPerception.EncoderSensor} class for left robot wheel which measures the wheels angle difference
	 * between actual an last request
	 */
	IPerception.EncoderSensor encoderLeft							=	null;
	/**
	 * reference to {@link IPerception.EncoderSensor} class for right robot wheel which measures the wheels angle difference
	 * between actual an last request
	 */
	IPerception.EncoderSensor encoderRight							=	null;
	
	/**
	 * reference to data class for measurement of the left wheels angle difference between actual an last request and the
	 * corresponding time difference
	 */
	IPerception.AngleDifferenceMeasurement angleMeasurementLeft 	= 	null;
	/**
	 * reference to data class for measurement of the right wheels angle difference between actual an last request and the
	 * corresponding time difference
	 */
	IPerception.AngleDifferenceMeasurement angleMeasurementRight	= 	null;
	
	/**
	 * line information measured by right light sensor: 0 - beside line, 1 - on line border or gray underground, 2 - on line
	 */
	int lineSensorRight	=	0;
	/**
	 * line information measured by left light sensor: 0 - beside line, 1 - on line border or gray underground, 2 - on line
	 */
	int lineSensorLeft	=	0;
	
	NXTMotor leftMotor = null;
	NXTMotor rightMotor = null;
	
	IPerception perception = null;
	INavigation navigation = null;
	IMonitor monitor = null;
	ControlThread ctrlThread = null;

    int leftMotorPower = 0;
	int rightMotorPower = 0;
	
	double w_motor_left = 0;
	double w_motor_right = 0;
	
	double velocity = 0.0;
	double angularVelocity = 0.0;
	
	Pose startPosition = new Pose();
	Pose currentPosition = new Pose();
	Pose destination = new Pose();
	
	ControlMode currentCTRLMODE = null;
	
	EncoderSensor controlRightEncoder    = null;
	EncoderSensor controlLeftEncoder     = null;

	int lastTime = 0;
	
    double currentDistance = 0.0;
    double Distance = 0.0;
    
    double radius_tire_mm = 225;
    double distance_tires_mm = 120;
  
	
	/**
	 * provides the reference transfer so that the class knows its corresponding navigation object (to obtain the current 
	 * position of the car from) and starts the control thread.
	 * 
	 * @param perception corresponding main module Perception class object
	 * @param navigation corresponding main module Navigation class object
	 * @param monitor corresponding main module Monitor class object
	 * @param leftMotor corresponding NXTMotor object
	 * @param rightMotor corresponding NXTMotor object
	 */
	public ControlRST(IPerception perception, INavigation navigation, NXTMotor leftMotor, NXTMotor rightMotor, IMonitor monitor){
		this.perception = perception;
        this.navigation = navigation;
        this.monitor = monitor;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		
		this.currentCTRLMODE = ControlMode.INACTIVE;
			
		this.encoderLeft  = perception.getControlLeftEncoder();
		this.encoderRight = perception.getControlRightEncoder();
		this.lineSensorRight		= perception.getRightLineSensor();
		this.lineSensorLeft  		= perception.getLeftLineSensor();
		
		// MONITOR (example)
		monitor.addControlVar("RightSensor");
		monitor.addControlVar("LeftSensor");
		
		this.ctrlThread = new ControlThread(this);
		
		ctrlThread.setPriority(Thread.MAX_PRIORITY - 1);
		ctrlThread.setDaemon(true); // background thread that is not need to terminate in order for the user program to terminate
		ctrlThread.start();
	}
	

	// Inputs
	
	/**
	 * set velocity
	 * @see parkingRobot.IControl#setVelocity(double velocity)
	 */
	public void setVelocity(double velocity) {
		this.velocity = velocity;
		
		
	}

	/**
	 * set angular velocity
	 * @see parkingRobot.IControl#setAngularVelocity(double angularVelocity)
	 */
	public void setAngularVelocity(double angularVelocity) {
		this.angularVelocity = angularVelocity;

	}
	
	/**
	 * set destination
	 * @see parkingRobot.IControl#setDestination(double heading, double x, double y)
	 */
	public void setDestination(double heading, double x, double y){
		this.destination.setHeading((float) heading);
		this.destination.setLocation((float) x, (float) y);
	}
	

	
	/**
	 * sets current pose
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
	public void setStartTime(int startTime){
		this.lastTime = startTime;
	}
	
	/**
	 * selection of control-mode
	 * @see parkingRobot.IControl#exec_CTRL_ALGO()
	 */
	public void exec_CTRL_ALGO(){
		
		switch (currentCTRLMODE)
		{
		  case LINE_CTRL	: update_LINECTRL_Parameter();
		                      exec_LINECTRL_ALGO();
		                      break;
		  case VW_CTRL		: update_VWCTRL_Parameter();
		   					  exec_VWCTRL_ALGO();
		   					  break; 
		  case SETPOSE      : update_SETPOSE_Parameter();
			  				  exec_SETPOSE_ALGO();
		                      break;
		  case PARK_CTRL	: update_PARKCTRL_Parameter();
		  					  exec_PARKCTRL_ALGO();
		  					  break;		  					  
		  case INACTIVE 	: exec_INACTIVE();
			                  break;
		}

	}
	
	// Private methods
	
	/**
	 * update parameters during VW Control Mode
	 */
	
	double kp = 20;
	
	// kp = 60, ki = 1.05
	
	// PIDData structs have to be initialized here so that their integral 
	// value stays constant.
	
	PIDData data_right = new PIDData(0, 0, 60, 1.05);
	PIDData data_left = new PIDData(0, 0, 60, 1.05);
	
	private void update_VWCTRL_Parameter(){
		setPose(navigation.getPose());
	
		// Measure angle difference and time delta between measurements
		AngleDifferenceMeasurement meas_left = perception.getControlLeftEncoder().getEncoderMeasurement();
		AngleDifferenceMeasurement meas_right = perception.getControlRightEncoder().getEncoderMeasurement();
		
		// Calculate angular velocities
		double w_meas_left = meas_left.getAngleSum() / meas_left.getDeltaT();
		double w_meas_right = meas_right.getAngleSum() / meas_right.getDeltaT();

		// Set PID Data
		data_right.setpoint = w_motor_right;
		data_right.processVariable = w_meas_right;

		data_left.setpoint = w_motor_left;
		data_left.processVariable = w_meas_left;

		System.out.println("Hello");
		
		// Hallo
		
		// LCD feedback for debugging
		LCD.drawString("WR 0." + (int) (w_meas_right * 100) + " 0." + (int) (w_motor_right * 100), 0, 0);
		LCD.drawString("WL 0." + (int) (w_meas_left * 100) + " 0." + (int) (w_motor_left * 100), 0, 1);
		
		// Adjust motor power based on calculated data
		leftMotorPower += PIDController.pi_ctrl(data_left);
		rightMotorPower += PIDController.pi_ctrl(data_right);

		leftMotor.setPower(leftMotorPower);
		rightMotor.setPower(rightMotorPower);
		
		// LCD feedback for debugging
		LCD.drawString(PIDController.pi_ctrl(data_left) + "", 0, 3);
		LCD.drawString(PIDController.pi_ctrl(data_left) + "", 0, 4);
		
		// GUI zum Testen!
		menu_update();
		
	}
	
	int btn_right_prev = 0;
	int btn_left_prev = 0;
	int btn_enter_prev = 0;
	int btn_back_prev = 0;
	
	int selection = 0;	// 0 - none; 1 - velocity; 2 - angular;
	int cursor = 1;	// 0 - none; 1 - velocity; 2 - angular;
	
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
				angularVelocity+=0.1;
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
				angularVelocity-=0.1;
			}
		}

		LCD.drawString("  vel = " + (int) velocity + "    ", 0, 5);
		LCD.drawString("  ang = " + angularVelocity + "    ", 0, 6);
		
		if (cursor == 1) {
			LCD.drawString("> ", 0, 5);
		} else if (cursor == 2) {
			LCD.drawString("> ", 0, 6);
		}
	}
	
	/**
	 * update parameters during SETPOSE Control Mode
	 */
	private void update_SETPOSE_Parameter(){
		setPose(navigation.getPose());
	}
	
	/**
	 * update parameters during PARKING Control Mode
	 */
	private void update_PARKCTRL_Parameter(){
		//Aufgabe 3.4
	}

	/**
	 * update parameters during LINE Control Mode
	 */
	private void update_LINECTRL_Parameter(){
		this.lineSensorRight		= perception.getRightLineSensor();
		this.lineSensorLeft  		= perception.getLeftLineSensor();		
	}
	
	/**
	 * The car can be driven with velocity in m/s or angular velocity in grade during VW Control Mode
	 * optionally one of them could be set to zero for simple test.
	 */
    private void exec_VWCTRL_ALGO(){  
		this.drive(this.velocity, this.angularVelocity);
	}
	
    private void exec_SETPOSE_ALGO(){
    	//Aufgabe 3.3
	}
	
	/**
	 * PARKING along the generated path
	 */
	private void exec_PARKCTRL_ALGO(){
		//Aufgabe 3.4
	}
	
    private void exec_INACTIVE(){
    	this.stop();
	}
	
	/**
	 * DRIVING along black line
	 * Minimalbeispiel
	 * Linienverfolgung fuer gegebene Werte 0,1,2
	 * white = 0, black = 2, grey = 1
	 */
    
	private void exec_LINECTRL_ALGO(){
		leftMotor.forward();
		rightMotor.forward();
		int lowPower = 1;
		int highPower = 30;
		
		// MONITOR (example)
		monitor.writeControlVar("LeftSensor", "" + this.lineSensorLeft);
		monitor.writeControlVar("RightSensor", "" + this.lineSensorRight);

        if(this.lineSensorLeft == 2 && (this.lineSensorRight == 1)){
			
			// when left sensor is on the line, turn left
    	    leftMotor.setPower(lowPower);
			rightMotor.setPower(highPower);
			
			// MONITOR (example)
			monitor.writeControlComment("turn left");
			
		} 
        else if(this.lineSensorRight == 2 && (this.lineSensorLeft == 1)){
		
			// when right sensor is on the line, turn right
			leftMotor.setPower(highPower);
			rightMotor.setPower(lowPower);
			
			// MONITOR (example)
			monitor.writeControlComment("turn right");
		}
		else if(this.lineSensorLeft == 2 && (this.lineSensorRight == 0)){
			
			// when left sensor is on the line, turn left
			leftMotor.setPower(lowPower);
			rightMotor.setPower(highPower);
			
			// MONITOR (example)
			monitor.writeControlComment("turn left");
			
		} 
		else if(this.lineSensorRight == 2 && (this.lineSensorLeft == 0)){
		
			// when right sensor is on the line, turn right
			leftMotor.setPower(highPower);
			rightMotor.setPower(lowPower);
			
			// MONITOR (example)
			monitor.writeControlComment("turn right");
		}
		else if(this.lineSensorLeft == 1 && this.lineSensorRight == 0) {
				
			// when left sensor is on the line, turn left
			leftMotor.setPower(lowPower);
			rightMotor.setPower(highPower);
			
			// MONITOR (example)
			monitor.writeControlComment("turn left");
				
		} 
		else if(this.lineSensorRight == 1 && this.lineSensorLeft == 0) {
			
			// when right sensor is on the line, turn right
			leftMotor.setPower(highPower);
			rightMotor.setPower(lowPower);
			
			// MONITOR (example)
			monitor.writeControlComment("turn right");
		}
	}
	
	private void stop(){
		this.leftMotor.stop();
		this.rightMotor.stop();
	}
		
    /**
     * calculates the left and right angle speed of the both motors with given velocity 
     * and angle velocity of the robot
     * @param v velocity of the robot
     * @param w angle velocity of the robot
     */
	
	private void drive(double v, double w){
		double vr_mms = v + w*distance_tires_mm/2;
		double vl_mms = v - w*distance_tires_mm/2;
		
		
		double wr = vr_mms / radius_tire_mm;
		double wl = vl_mms / radius_tire_mm;
	
		
		w_motor_left = wl;
		w_motor_right = wr;
		
		leftMotor.forward();
		rightMotor.forward();
		
	}
	
	private void driveRaw(double v, double w) {
		double vr_mms = v + 1/2*w*distance_tires_mm;
		double vl_mms = v - 1/2*w*distance_tires_mm;
		
		double wr = vr_mms / radius_tire_mm;
		double wl = vl_mms / radius_tire_mm;
		
		rightMotor.setPower(getPowerForW_M1(wr));
		leftMotor.setPower(getPowerForW_M2(wl));

		//LCD.drawString(wr + " -> " + getPowerForW_M1(wr), 0, 0);
		//LCD.drawString(wl + " -> " + getPowerForW_M1(wl), 0, 1);
	}
	
	private int getPowerForW_M1(double w) {
		return (int) (1/8.24 * (w + 50));
	}
	
	private int getPowerForW_M2(double w) {
		return (int) (1/8.67 * (w + 8));
	}
}