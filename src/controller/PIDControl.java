package controller;

import java.util.LinkedList;
import java.util.List;

public class PIDControl {
    private final double KP, KI, KD;
    private final double IBOUND = 10;
    private final double OBOUND = 10;
    private final long TIME_DIFF = 150;
    
    private final double desired;
    private double error, int_error, diff;
    private long prev_time;
    private List<TimeError> prev_errors;
	
	public PIDControl (double desired, double KP, double KI, double KD) {
	    this.desired = desired;
	    this.KP = KP;
	    this.KI = KI;
	    this.KD = KD;
	    
	    error = 0;
	    diff = 0;
	    int_error = 0;
	    prev_time = 0;
	    prev_errors = new LinkedList<TimeError>();
	}
	
	public double init(double actual){
	    prev_errors.add(new TimeError(desired - actual, System.currentTimeMillis()));
	    prev_time = System.currentTimeMillis();
	    return desired - actual;
	}
	
	public double update(double actual) {
	    long time = System.currentTimeMillis();
	    prev_errors.add(new TimeError(desired - actual, time));
	    
	    while (prev_errors.get(0).time < time - TIME_DIFF){
	        prev_errors.remove(0);
	    }
	    
	    error = desired - actual;
	    int_error += error*(time - prev_time)/1000;
	    TimeError prev_error = prev_errors.get(0);
	    diff = (error - prev_error.error)/(time - prev_error.time);
	    
	    prev_time = time;
	    
	    return absoluteBound(KP*error + absoluteBound(KI*int_error, IBOUND) + KD*diff, OBOUND);
	}

	public void print(double actual){
	    System.out.println("ERROR: " + error);
	    System.out.println("INTEGRAL: " + int_error);
	    System.out.println("DERIVATIVE: " + diff);
	}
	
	private double absoluteBound(double value, double bound){
	    if (value > bound){
	        return bound;
	    } else if (value < -bound){
	        return -bound;
	    } else {
	        return value;
	    }
	}
	
	private class TimeError {
	    public final double error;
	    public final long time;
	    
	    public TimeError(double error, long time){
	        this.error = error;
	        this.time = time;
	    }
	}	
}
