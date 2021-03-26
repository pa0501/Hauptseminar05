package parkingRobot.hsamr0;

import lejos.nxt.Button;
import lejos.nxt.ButtonListener;
import lejos.nxt.MotorPort;
import lejos.nxt.NXTMotor;
import lejos.nxt.Sound;
import lejos.robotics.navigation.Pose;
import parkingRobot.IControl;
import parkingRobot.IControl.*;
import parkingRobot.INxtHmi;
import parkingRobot.INavigation;
import parkingRobot.IPerception;
import parkingRobot.IMonitor;

import lejos.geom.Line;
import lejos.nxt.LCD;

import parkingRobot.hsamr0.ControlRST;
import parkingRobot.hsamr0.HmiPLT;
import parkingRobot.hsamr0.NavigationAT;
import parkingRobot.hsamr0.PerceptionPMP;

//PUSH TEST v2
/**
 * Main class for 'Hauptseminar AMR' project 'autonomous parking' for students
 * of electrical engineering with specialization 'automation, measurement and
 * control'.
 * <p>
 * Task of the robotic project is to develop an mobile robot based on the Lego
 * NXT system witch can perform parking maneuvers on an predefined course. To
 * fulfill the interdisciplinary aspect of this project the software structure
 * is divided in 5 parts: human machine interface, guidance, control, perception
 * and navigation.
 * <p>
 * Guidance is to be realized in this main class. The course of actions is to be
 * controlled by one or more finite state machines (FSM). It may be advantageous
 * to nest more than one FSM.
 * <p>
 * For the other parts there are interfaces defined and every part has to be
 * realized in one main module class. Every class (except guidance) has
 * additionally to start its own thread for background computation.
 * <p>
 * It is important that data witch is accessed by more than one main module
 * class thread is only handled in a synchronized context to avoid inconsistent
 * or corrupt data!
 */
public class Test_Vert2_1 {

	/**
	 * states for the main finite state machine. This main states are requirements
	 * because they invoke different display modes in the human machine interface.
	 */
	public enum CurrentStatus {
		/**
		 * indicates that robot is following the line and maybe detecting parking slots
		 */
		DRIVING,
		/**
		 * indicates that robot is performing an parking maneuver
		 */
		INACTIVE,
		/**
		 * indicates that shutdown of main program has initiated
		 */
		EXIT
	}

	/**
	 * state in which the main finite state machine is running at the moment
	 */
	protected static CurrentStatus currentStatus = CurrentStatus.INACTIVE;
	/**
	 * state in which the main finite state machine was running before entering the
	 * actual state
	 */
	protected static CurrentStatus lastStatus = CurrentStatus.INACTIVE;

	/**
	 * one line of the map of the robot course. The course consists of a closed
	 * chain of straight lines. Thus every next line starts where the last line ends
	 * and the last line ends where the first line starts. This documentation for
	 * line0 hold for all lines.
	 */
	static Line line0 = new Line(0, 0, 180, 0);
	static Line line1 = new Line(180, 0, 180, 60);
	static Line line2 = new Line(180, 60, 150, 60);
	static Line line3 = new Line(150, 60, 150, 30);
	static Line line4 = new Line(150, 30, 30, 30);
	static Line line5 = new Line(30, 30, 30, 60);
	static Line line6 = new Line(30, 60, 0, 60);
	static Line line7 = new Line(0, 60, 0, 0);
	/**
	 * map of the robot course. The course consists of a closed chain of straight
	 * lines. Thus every next line starts where the last line ends and the last line
	 * ends where the first line starts. All above defined lines are bundled in this
	 * array and to form the course map.
	 */
	static Line[] map = { line0, line1, line2, line3, line4, line5, line6, line7 };

	static int index_setPose = 0;
	static boolean setPose_ready = true;
	
	static long ms_start = 0;
	
	static double distance_prev = 0;

	/**
	 * main method of project 'ParkingRobot'
	 * 
	 * @param args standard string arguments for main method
	 * @throws Exception exception for thread management
	 */
	public static void main(String[] args) throws Exception {
		currentStatus = CurrentStatus.INACTIVE;
		lastStatus = CurrentStatus.EXIT;

		// Generate objects

		NXTMotor leftMotor = new NXTMotor(MotorPort.B);
		NXTMotor rightMotor = new NXTMotor(MotorPort.A);

		IMonitor monitor = new Monitor();

		IPerception perception = new PerceptionPMP(leftMotor, rightMotor, monitor);
		perception.calibrateLineSensors();

		INavigation navigation = new NavigationAT(perception, monitor);
		IControl control = new ControlRST(perception, navigation, leftMotor, rightMotor, monitor);

		monitor.startLogging();
		
		Pose pose_destination = new Pose(0.4f, -0.5f, 0f);
		
		index_setPose = 0;
		
		Button.ENTER.waitForPress();

		
		
		while (true) {
			
			if (setPose_ready == true) {

				switch (index_setPose) {
				case 0:
					setPose_ready = false;

					control.setVelocity(0.1);
					control.setAngularVelocity(15);
					
					//control.setStartTime(System.currentTimeMillis());
					control.setPose(new Pose(1.2f, 0f, 0f));
					control.setCtrlMode(IControl.ControlMode.SETPOSE);
					break;

				case 1:
					setPose_ready = false;

					control.setVelocity(0.05);
					control.setAngularVelocity(30);

					control.setPose(new Pose(1.2f, 0.30f, 0));
					control.setCtrlMode(IControl.ControlMode.SETPOSE);
					
					break;

				case 2:
					if (currentStatus != CurrentStatus.DRIVING) {
						currentStatus = CurrentStatus.DRIVING;
						control.setCtrlMode(IControl.ControlMode.LINE_CTRL);
						
						distance_prev = navigation.getPose().distanceTo(pose_destination.getLocation());
					}
					
					double distance = navigation.getPose().distanceTo(pose_destination.getLocation());
					
					if (distance < 1f) {
						if (distance_prev < distance) {
							notify_setPose_ready();
							control.setCtrlMode(ControlMode.INACTIVE);
							currentStatus = CurrentStatus.INACTIVE;
						}
					}
					
					distance_prev = distance;
					break;
					
				case 3:
					if (System.currentTimeMillis() - ms_start > 1000) {
						notify_setPose_ready();
					}
					
					break;
				case 4:
					setPose_ready = false;
					
					control.setVelocity(0.1);
					control.setAngularVelocity(50);
					
					//control.setStartTime(System.currentTimeMillis());
					
					Pose pose = navigation.getPose();
					
					Pose pose_next = new Pose(pose.getX(), pose.getY(), pose.getHeading());
					
					pose_next.translate(0.2f, 0.1f);
					pose_next.rotateUpdate(30);
					control.setPose(pose_next);
					//control.setPose(new Pose(0.2f, 0f, 0f));
					control.setCtrlMode(IControl.ControlMode.SETPOSE);
					
					
					break;
				case 5:
					if (currentStatus != CurrentStatus.DRIVING) {
						currentStatus = CurrentStatus.DRIVING;
						
						control.setVelocity(0.05);
						control.setDestination(0, -0.4f, 0.3f);
						control.setStartTime((int) System.currentTimeMillis());
						control.setCtrlMode(IControl.ControlMode.PARK_CTRL);
						
						distance_prev = navigation.getPose().distanceTo(pose_destination.getLocation());
					}
					
					break;
				default:
					//System.exit(0);

					break;
				}
			}

			if (Button.ESCAPE.isDown()) {
				System.exit(0);
				;
			}
		}
	}

	/**
	 * returns the actual state of the main finite state machine as defined by the
	 * requirements
	 * 
	 * @return actual state of the main finite state machine
	 */
	public static CurrentStatus getCurrentStatus() {
		return Test_Vert2_1.currentStatus;
	}

	/**
	 * plots the actual pose on the robots display
	 * 
	 * @param navigation reference to the navigation class for getting pose
	 *                   information
	 */
	public static void showData(INavigation navigation, IPerception perception) {

		LCD.drawString("X (in cm): " + (navigation.getPose().getX() * 100), 0, 0);
		LCD.drawString("Y (in cm): " + (navigation.getPose().getY() * 100), 0, 1);
		// LCD.drawString("Phi (grd): " + (navigation.getPose().getHeading() / Math.PI *
		// 180), 0, 2);

		LCD.drawString("Phi (grd): " + (navigation.getPose().getHeading() / Math.PI * 180), 0, 2);
		LCD.drawString("distance_prev: " + distance_prev, 0, 3);

//		perception.showSensorData();

//    	if ( hmi.getMode() == parkingRobot.INxtHmi.Mode.SCOUT ){
//			LCD.drawString("HMI Mode SCOUT", 0, 3);
//		}else if ( hmi.getMode() == parkingRobot.INxtHmi.Mode.PAUSE ){
//			LCD.drawString("HMI Mode PAUSE", 0, 3);
//		}else{
//			LCD.drawString("HMI Mode UNKNOWN", 0, 3);
//		}
	}

	public static void notify_setPose_ready() {
		ms_start = System.currentTimeMillis();
		index_setPose++;
		setPose_ready = true;
	}
}