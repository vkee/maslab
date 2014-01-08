package controller;

public class PID {
	private final double proportionalC;
	private final double derivativeC;
	private final double integralC;
	private final double desired;
	
	private double integralError;
	private double previousError;
	private double previousTime;
	
	public PID (double desired, double proportionalC, double derivativeC, double integralC) {
		this.proportionalC = proportionalC;
		this.derivativeC = derivativeC;
		this.desired = desired;
		this.integralC = integralC;
		this.integralError = 0;
		this.previousError = 0;
		this.previousTime = System.currentTimeMillis();
	}
	
	public double update(double actual, boolean start) {
		
		double proportional;
		double derivative;
		
		if (start) {
			double time = System.currentTimeMillis();
			double error = desired-actual;
			proportional = proportionalC*(error);
			derivative = 0;
			previousTime = time;
			previousError = error;
		} else {

			double time = System.currentTimeMillis();
			double error = desired-actual;
			proportional = proportionalC*(error);
			integralError += integralC*error*(time-previousTime);
			derivative = derivativeC*(error-previousError)/(time-previousTime);
			previousTime = time;
			previousError = error;
		}
		return proportional + integralError + derivative;
	}
	
}
