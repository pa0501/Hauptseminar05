package parkingRobot.hsamr0.util;

import java.util.Vector;

import lejos.geom.Point;
import lejos.robotics.navigation.Pose;

public class PosePath {
	private Pose pose_start, pose_end;
	
	private double u_x, u_y;				// normalized direction vector coordinates
	private double v0;
	
	public PosePath(Pose start, Pose end, double velocity) {
		pose_start = start;
		pose_end = end;
		v0 = velocity;
		
		calculate_u();
	}
	
	public PosePath() {
	}
	
	private void calculate_u() {
		double dif_x = pose_end.getX() - pose_start.getX();
		double dif_y = pose_end.getY() - pose_start.getY();
		
		double norm = Math.sqrt(dif_x * dif_x + dif_y * dif_y);
		
		u_x = dif_x / norm;
		u_y = dif_y / norm;
	}
	
	public Point calculate_point_target(double t) {
		float target_x = (float) (pose_start.getX() + u_x * t * v0);
		float target_y = (float) (pose_start.getY() + u_y * t * v0);
		
		return new Point(target_x, target_y);
	}
	
	public double calculate_e_lat(double t, Pose pose_current) {
		double target_x = pose_start.getX() + u_x * t * v0;
		double target_y = pose_start.getY() + u_y * t * v0;
		
		double cross_z = (pose_current.getX() - target_x) * u_y - (pose_current.getY() - target_y) * u_x; 
		
		return (-cross_z);
	}
	
	
}
