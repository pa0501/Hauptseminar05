package parkingRobot.hsamr0;

import java.util.ArrayList;

import lejos.geom.Line;
import lejos.robotics.navigation.Pose;

import parkingRobot.INavigation;
import parkingRobot.IPerception;
import parkingRobot.INavigation.ParkingSlot.ParkingSlotStatus;
import parkingRobot.IMonitor;

import parkingRobot.hsamr0.NavigationThread;


/**
 * A executable basic example implementation of the corresponding interface provided by the Institute of Automation with
 * limited functionality:
 * <p>
 * In this example only the both encoder sensors from the {@link IPerception} implementation are used for periodically calculating
 * the robots position and corresponding heading angle (together called 'pose'). Neither any use of the map information or other
 * perception sensor information is used nor any parking slot detection is performed, although the necessary members are already
 * prepared. Advanced navigation calculation and parking slot detection should be developed and invented by the students.  
 * 
 * @author IfA
 */
public class NavigationAT implements INavigation{
	
	/**
	 * line information measured by right light sensor: 0 - beside line, 1 - on line border or gray underground, 2 - on line
	 */
	int lineSensorRight	=	0;
	/**
	 * line information measured by left light sensor: 0 - beside line, 1 - on line border or gray underground, 2 - on line
	 */
	int lineSensorLeft	=	0;
	
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
	 * reference to {@link IPerception.OdoSensor} class for mouse odometry sensor to measure the ground displacement
	 * between actual an last request
	 */
	IPerception.OdoSensor mouseodo = null;	
		
	/**
	 * reference to data class for measurement of the mouse odometry sensor to measure the ground displacement between
	 * actual an last request and the corresponding time difference
	 */
	IPerception.OdoDifferenceMeasurement mouseOdoMeasurement = null;
	
	/**
	 * distance from optical sensor pointing in driving direction to obstacle in mm
	 */
	double frontSensorDistance		=	0;
	/**
	 * distance from optical sensor pointing to the right side of robot to obstacle in mm (sensor mounted at the front)
	 */
	double frontSideSensorDistance	=	0;
	/**
	 * distance from optical sensor pointing in opposite of driving direction to obstacle in mm
	 */
	double backSensorDistance		=	0;
	/**
	 * distance from optical sensor pointing to the right side of robot to obstacle in mm (sensor mounted at the back)
	 */
	double backSideSensorDistance	=	0;


	/**
	 * robot specific constant: radius of left wheel
	 */
	static final double LEFT_WHEEL_RADIUS	= 	0.23; // only rough guess, to be measured exactly and maybe refined by experiments
	/**
	 * robot specific constant: radius of right wheel
	 */
	static final double RIGHT_WHEEL_RADIUS	= 	0.23; // only rough guess, to be measured exactly and maybe refined by experiments
	/**
	 * robot specific constant: distance between wheels
	 */
	static final double WHEEL_DISTANCE		= 	0.112; // only rough guess, to be measured exactly and maybe refined by experiments

	// CPI for Mouse Sensor
	private static final double MOUSE_CPI = 400;
	private static final double MOUSE_FULL_TURN = 12;

	// control sensor fusion
	private static final double G_POS = 0;   // weight 
	private static final double G_HEADING = 0;
	
	private static final double DISTANCCE_FROM_WALL = 280;   // only rough guess, Because there is no vlaus from Distance Sensor
	private static final int TURN_ON_THE_CORNER = 35;

	
	/**
	 * map array of line references, whose corresponding lines form a closed chain and represent the map of the robot course
	 */
	Line[] map 								= null;
	/**
	 * reference to the corresponding main module Perception class
	 */
	IPerception perception 	        		= null;
	/**
	 * reference to the corresponding main module Monitor class
	 */
	IMonitor monitor = null;
	/**
	 * indicates if parking slot detection should be switched on (true) or off (false)
	 */
	boolean parkingSlotDetectionIsOn		= false;
	/**
	 * pose class containing bundled current X and Y location and corresponding heading angle phi
	 */
	Pose pose								= new Pose();

	/**
	 * thread started by the 'Navigation' class for background calculating
	 */
	NavigationThread navThread = new NavigationThread(this);
	
	
	// Saving Parkingslot in The array 
	ArrayList<ParkingSlot> parkingslots = new ArrayList<ParkingSlot>();

	
	/**
	 * provides the reference transfer so that the class knows its corresponding perception object (to obtain the measurement
	 * information from) and starts the navigation thread.
	 * 
	 * @param perception corresponding main module Perception class object
	 * @param monitor corresponding main module Monitor class object
	 */
	public NavigationAT(IPerception perception, IMonitor monitor){
		this.perception   = perception;
		this.monitor = monitor;
		this.encoderLeft  = perception.getNavigationLeftEncoder();
		this.encoderRight = perception.getNavigationRightEncoder();
		this.mouseodo = perception.getNavigationOdo();	
		
		
		//monitor.addNavigationVar("X");
		//monitor.addNavigationVar("Y");
		//monitor.addNavigationVar("W");
		
		
		navThread.setPriority(Thread.MAX_PRIORITY - 1);
		navThread.setDaemon(true); // background thread that is not need to terminate in order for the user program to terminate
		navThread.start();
	}
	
	
	// Inputs
	
	/* (non-Javadoc)
	 * @see parkingRobot.INavigation#setMap(lejos.geom.Line[])
	 */
	public void setMap(Line[] map){
		this.map = map;
	}
	/* (non-Javadoc)
	 * @see parkingRobot.INavigation#setDetectionState(boolean)
	 */
	public void setDetectionState(boolean isOn){
		this.parkingSlotDetectionIsOn = isOn;
	}
	
	
	// 	Class control
	
	/* (non-Javadoc)
	 * @see parkingRobot.INavigation#updateNavigation()
	 */
	public synchronized void updateNavigation(){	
		this.updateSensors();
		this.calculateLocation();
		if (this.parkingSlotDetectionIsOn)
				this.detectParkingSlot();
		
		// MONITOR (example)
		//monitor.writeNavigationComment("Navigation");
		   
			
	}
	
	
	// Outputs
	
	/* (non-Javadoc)n
	 * @see parkingRobot.INavigation#getPose()
	 */
	public synchronized Pose getPose(){
		return this.pose;
	}
	/* (non-Javadoc)
	 * @see parkingRobot.INavigation#getParkingSlots()
	 */
	public synchronized ParkingSlot[] getParkingSlots() {
		return (ParkingSlot[]) parkingslots.toArray();
	}
	
	
	// Private methods
	
	/**
	 * calls the perception methods for obtaining actual measurement data and writes the data into members
	 */
	private void updateSensors(){		
		this.lineSensorRight		= perception.getRightLineSensor();
		this.lineSensorLeft  		= perception.getLeftLineSensor();
		
		this.angleMeasurementLeft  	= this.encoderLeft.getEncoderMeasurement();
		this.angleMeasurementRight 	= this.encoderRight.getEncoderMeasurement();

		this.mouseOdoMeasurement	= this.mouseodo.getOdoMeasurement();

		this.frontSensorDistance	= perception.getFrontSensorDistance();
		this.frontSideSensorDistance = perception.getFrontSideSensorDistance();
		this.backSensorDistance		= perception.getBackSensorDistance();
		this.backSideSensorDistance	= perception.getBackSideSensorDistance();
	}		

	double lastMouseX = 0;
	double lastMouseY = 0;

	/**
	 * calculates the robot pose from the measurements
	 */
	private void calculateLocation(){
		double leftAngleSpeed 	= this.angleMeasurementLeft.getAngleSum()  / ((double)this.angleMeasurementLeft.getDeltaT()/1000);  //degree/seconds
		double rightAngleSpeed 	= this.angleMeasurementRight.getAngleSum() / ((double)this.angleMeasurementRight.getDeltaT()/1000); //degree/seconds

		double vLeft		= (leftAngleSpeed  * Math.PI * LEFT_WHEEL_RADIUS ) / 180 ; //velocity of left  wheel in m/s
		double vRight		= (rightAngleSpeed * Math.PI * RIGHT_WHEEL_RADIUS) / 180 ; //velocity of right wheel in m/s		
		double w 			= (vRight - vLeft) / WHEEL_DISTANCE; //angular velocity of robot in rad/s
		
		Double R 			= new Double(( WHEEL_DISTANCE / 2 ) * ( (vLeft + vRight) / (vRight - vLeft) ));								
		
		double ICCx 		= 0;
		double ICCy 		= 0;

		double xResult 		= 0;
		double yResult 		= 0;
		double angleResult 	= 0;
		
		double deltaT       = ((double)this.angleMeasurementLeft.getDeltaT())/1000;
		
		if (R.isNaN()) { //robot don't move
			xResult			= this.pose.getX();
			yResult			= this.pose.getY();
			angleResult 	= this.pose.getHeading();
			
			//monitor.writeNavigationComment("robot don't move");
			
		} else if (R.isInfinite()) { //robot moves straight forward/backward, vLeft==vRight
			xResult			= this.pose.getX() + vLeft * Math.cos(this.pose.getHeading()) * deltaT;
			yResult			= this.pose.getY() + vLeft * Math.sin(this.pose.getHeading()) * deltaT;
			angleResult 	= this.pose.getHeading();
			
			//monitor.writeNavigationComment("forward/backward");
		} else {			
			ICCx = this.pose.getX() - R.doubleValue() * Math.sin(this.pose.getHeading());
			ICCy = this.pose.getY() + R.doubleValue() * Math.cos(this.pose.getHeading());
		
			xResult 		= Math.cos(w * deltaT) * (this.pose.getX()-ICCx) - Math.sin(w * deltaT) * (this.pose.getY() - ICCy) + ICCx;
			yResult 		= Math.sin(w * deltaT) * (this.pose.getX()-ICCx) + Math.cos(w * deltaT) * (this.pose.getY() - ICCy) + ICCy;
			angleResult 	= this.pose.getHeading() + w * deltaT;
			
			//monitor.writeNavigationComment("Heading");
		}
		
		// mouse sensor
		double dX = this.mouseOdoMeasurement.getUSum() - lastMouseX;
		double dHead = (float) dX*360.0 / MOUSE_FULL_TURN; 

		double heading_mou = this.pose.getHeading();
    	heading_mou += dHead;
		
		while (heading_mou >= 359)
		heading_mou -= 360.0;
		while (heading_mou < 0)
		heading_mou += 360.0;

		double x_mou = this.pose.getX();
		double y_mou = this.pose.getY();

		double dY = this.mouseOdoMeasurement.getVSum() - lastMouseY;
        double dPos = (float) dY * Math.cos(heading_mou* Math.PI/180) *25.4 / MOUSE_CPI;
		y_mou += dPos;
		dPos = (float) dX * Math.sin(heading_mou* Math.PI/180) *25.4 / MOUSE_CPI;
		x_mou += dPos;

		lastMouseX = this.mouseOdoMeasurement.getUSum();
		lastMouseY = this.mouseOdoMeasurement.getVSum(); 
		//

		// Fusion
		xResult = (1.0 - G_POS) * xResult + G_POS * x_mou;
		yResult = (1.0 - G_POS) * yResult + G_POS * y_mou;
		angleResult = (1.0 - G_HEADING) * angleResult + G_HEADING * heading_mou;
		//
		
		// Robot is parallel to wall
		/*if (this.frontSensorDistance == 0 && this.frontSideSensorDistance > 0) {
			
			this.pose.setHeading(180);
		}
		
		//At the corner
		if (this.frontSensorDistance > 0 && this.frontSideSensorDistance > 0) {
			this.pose.setHeading(90);
			
		}*/
		
		
		this.pose.setLocation((float)xResult, (float)yResult);
		this.pose.setHeading((float)angleResult);		
		
		//monitor.writeNavigationComment("X =" + this.pose.getX());
		//monitor.writeNavigationComment("Y =" + this.pose.getY());
		//monitor.writeNavigationComment("W =" + this.pose.getHeading());
	}
	  
	 public void reset(){
		 
		 this.pose.setLocation(0,0);          
		 this.pose.setHeading(0);    
	     
	    }
	 
	 /**
		 * detects parking slots and manage them by initializing new slots, re-characterizing old slots or merge old and detected slots. 
		 */
		  int ID = 1;
		  //int i = 0;
		  int j=0;
		  ParkingSlot parkingslot = null;

		private void detectParkingSlot(){

			/*if (this.lineSensorRight  < TURN_ON_THE_CORNER|| this.lineSensorLeft < TURN_ON_THE_CORNER) {
				j++;
				if(j == 8) { 
					parkingSlotDetectionIsOn= false;
					return;
				}
			
			}*/
			monitor.writeNavigationComment("Parkingslot =" + perception.getFrontSideSensorDistance());
			    
			if (perception.getFrontSideSensorDistance() > DISTANCCE_FROM_WALL) {
				
				// Check if the Array Empty to avoid IndexOutOfBoundsException 
				/*if(!parkingslots.isEmpty()) {
					parkingslot = parkingslots.get(i);
				}*/
				
				if (parkingslot != null) {
					 return;
				}
				parkingslot = new ParkingSlot(ID);  // Create an object 
				
				monitor.writeNavigationComment("Parkingslot =");
		
				
				
				parkingslot.setBackBoundaryPosition(this.pose.getLocation());    // save the first Point 
						
			}
			else {
				 if (parkingslot == null) {
					 return;
				 }
				parkingslot.setFrontBoundaryPosition(this.pose.getLocation());  // save the second Point 
				
				// Calculate the size of The Parking slot.
				double ParkSlotDistanc = parkingslot.getBackBoundaryPosition().distance(parkingslot.getFrontBoundaryPosition());
				
				// Check if The Parking slot fit the Required condition >= 45 cm
				if(ParkSlotDistanc >= 450) {
				
				parkingslot.setStatus(ParkingSlotStatus.SUITABLE_FOR_PARKING);
				
				monitor.writeNavigationComment("X =" + parkingslot.getStatus());
				monitor.writeNavigationComment("SUITABLE_FOR_PARKING");
				
				}
				
				else {
					
					parkingslot.setStatus(ParkingSlotStatus.NOT_SUITABLE_FOR_PARKING);
					
					monitor.writeNavigationComment("X =" + parkingslot.getStatus());
					monitor.writeNavigationComment("NOT_SUITABLE_FOR_PARKING");
				}

				parkingslots.add(parkingslot);   // add an element
				parkingslot = null;
				 // Increasing ID & i so every Parking slot have a particular ID 
				
				ID++;
				//i++;   
				
			}
			   // has to be implemented by students
			
		}
	    
}