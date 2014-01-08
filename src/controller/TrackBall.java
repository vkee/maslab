package controller;

public class TrackBall {

	PID pid;
	KitBotModel model;
	
	public TrackBall(KitBotModel model, double proportionalC, double derivativeC, double integralC, double x, double y) {
		PID pid = new PID(1900/2, proportionalC, derivativeC, integralC);
		double pidOut = pid.update(y, true);
		double turn = Math.max(0.2, pidOut/1900);
		model.setMotors(-turn, turn);
	}
	
	public void update(double x, double y) {		
		double pidOut = pid.update(y, false);
		double turn = Math.max(0.2, pidOut/1900);
		model.setMotors(-turn, turn);
	}
	
	public static void main(String[] args) {
		KitBotModel model = new KitBotModel();
		double x = -1;
		double y = -1;
		while (true) {
			if (y == -1) {
				continue;
			} else {
				TrackBall track = new TrackBall(model,1,0,0,x,y);
				break;
			}
		}
		
	}
}
