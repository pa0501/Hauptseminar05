package parkingRobot.hsamr0;
import java.util.ArrayList;

import lejos.geom.Point;

public class ParkingPath {
	Double coefsX[];
	
	double x1, T;
	
	double v0 = 0.1;
	
	public ParkingPath() {
		coefsX = new Double[6];
	}
	
	public static ParkingPath withCoefficients(Double[] coeffs) {
		ParkingPath path = new ParkingPath();
		
		for (int i=0; i<coeffs.length; i++) {
			path.addCoefficient(i, coeffs[coeffs.length - i - 1]);
		}
		
		return path;
	}
	
	public ParkingPath endPoint(double x, double t) {
		x1 = x;
		T  = t;
		
		return this;
	}
	
	/**
	 * 
	 * @param p_end [x and y in m] The path's end point.
	 * @return A generated path that intersects with the specified end point.
	 */
	
	public static ParkingPath withEnd(double x, double T) {
		double c3 = 10*x / Math.pow(T, 3);
		double c4 = -15*x / Math.pow(T, 4);
		double c5 = 6*x / Math.pow(T, 5);
		
		return ParkingPath.withCoefficients(new Double[] {c5, c4, c3, 0d, 0d, 0d}).endPoint(x, T);
	}
	
	public double getEndX() {
		return x1;
	}
	
	public double getEndT() {
		return T;
	}
	
	public void addCoefficient(int index, double value) {
		coefsX[index] = value;
	}
	
	/**
	 * Calculate phi at time t.
	 * @param t
	 * @return
	 */
	
	public double calc_phi(double t) {
		//return Math.atan(calc_x_dot1(t));
		
		return Math.atan(calc_x_dot1(t) / v0);
	}
	
	/**
	 * Calculate omega at time t.
	 * @param t
	 * @return
	 */
	
	public double calc_w(double t) {
		return Math.toDegrees(calc_x_dot2(t) / (Math.pow(calc_x_dot1(t), 2) + 1));
		
		// Hier muss v0 irgendwie mit eingerechnet werden: Irgendwo ist noch ein Fehler.
		
		//return Math.toDegrees(calc_x_dot2(t) / (Math.pow(calc_x_dot1(t), 2) + Math.pow(v0, 2)));
	}
	
	/**
	 * Calculate v at time t.
	 * @param t
	 * @return
	 */
	
	public double calc_v(double t) {
		//return calc_x_dot1(t) * Math.cos(calc_phi(t)) + Math.sin(calc_phi(t));
		
		return calc_x_dot1(t) * Math.cos(calc_phi(t)) + v0 * Math.sin(calc_phi(t));
	}
	
	public double calc_x(double t) {
		double result = 0;
		
		for (int i=0; i<coefsX.length; i++) {
			result += coefsX[i] * Math.pow(t, i);
		}
		
		return result;
	}
	
	/**
	 * Calculates first derivative of time at point t.
	 * @param t
	 * @return
	 */
	
	public double calc_x_dot1(double t) {
		double result = 0;
		
		for (int i=1; i<coefsX.length; i++) {
			result += i * coefsX[i] * Math.pow(t, i-1);
		}
		
		return result;
	}
	
	/**
	 * Calculates second derivative of time at point t.
	 * @param t
	 * @return
	 */
	
	public double calc_x_dot2(double t) {
		double result = 0;
		
		for (int i=2; i<coefsX.length; i++) {
			result += i * (i-1) * coefsX[i] * Math.pow(t, i-2);
		}
		
		return result;
	}
	
}