package parkingRobot.hsamr0;

public class Point {

	private double x;
	private double y;
	
	public Point(double x, double y){
		this.x = x;
		this.y = y;
		}
	
	public double getx(){
		return this.x;
	}
	
	public double gety(){
		return this.y;
	}
	/**
	 * set new value for x
	 */
	public void setx(double x){
		this.x = x;
	}
	/**
	 * set new value for x
	 */
	public void sety(double y){
		this.y = y;
	}
}
