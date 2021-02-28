package parkingRobot;

import lejos.robotics.navigation.Pose;

/**
 * interface for the main module 'Control', providing methods for executing the algorithms to
 * control the robot according to different modes which is set by guidance.
 * 
 * @author RST
 */
public interface IControl {
	// Inputs
	
	
	/**
	 * The predefined control modes
	 */
	public enum ControlMode {
		
		/**
		 * folgt schwarzer Linie
		 */
		LINE_CTRL,
		
		/**
		 * einparken
		 */
		PARK_CTRL,
		
		/**
		 * v/w-Control
		 */
		VW_CTRL,
		
		/**
		 * eine Zielpsoe anfahren
		 */
		SETPOSE,
		
		/**
		 * NXT in Ruhe versetzen
		 */
		INACTIVE
	}
	
		
	/**
	 * set the required speed
	 * 
	 * @param velocity [m/s] the velocity of the robot to be set
	 */	
	public void setVelocity(double velocity);

	
	/**
	 *  set the required angular velocity
	 *  
	 * @param angularVelocity [deg/s] the angular velocity to be set for vw-control
	 */
	public void setAngularVelocity(double angularVelocity);

	/**
	 * Set the destination to be driven to at given velocity for park control.
	 * 
	 * @param heading the heading angle of the robot at the destination
	 * @param x [m] the destination position in x axis
	 * @param y [m] the destination position in y axis
	 */
	public void setDestination(double heading, double x, double y);
		
	
	/**
	 * the Robot's current position 
	 * 
	 * @param currentPosition the current position of the robot at each sampling  
	 */	
	public void setPose(Pose currentPosition); 	
	

	
	
	/**
	 * set the current control mode
	 * 
	 * @param ctrl_mode parameter for control mode which is defined by Guidance 
	 */
	public void setCtrlMode(ControlMode ctrl_mode);
	
	/**
	 * set start time
	 * @param startTime start time
	 */
	public void setStartTime(long startTime);
	
	
	/**
	 * execute the selected algorithms for control which was set by guidance
	 */
	public void exec_CTRL_ALGO();

}

