package controller;

public class WallFollowSonar {
	public static void main(String[] args) {
		KitBotModel model = new KitBotModel();
		new WallFollowSonar(model);
	}
	
	public WallFollowSonar(KitBotModel model) {
		
		double forward = 0.1; 
		
		model.updateData();
		PID pidDist = new PID(150.0,0.0005,0.0,0.0);
		PID pidAngle = new PID(0,100,0.0,0.0);
		double turn1 = Math.max(-0.01, Math.min(0.01, 
				pidDist.update((model.getSonar1Distance()+model.getSonar2Distance())/2, true)));
		double turn2 = Math.max(-0.01, Math.min(0.01, 
				pidAngle.update(this.calcAngle(model.getSonar1Distance(), model.getSonar2Distance(), 100), true)));
		model.setMotors(forward - turn1 - turn2, forward + turn1 + turn2);
		
		while (true) {
			model.updateData();
			//System.out.println(model.getSonarDistance());
			turn1 = Math.max(-0.01, Math.min(0.01, 
					pidDist.update((model.getSonar1Distance()+model.getSonar2Distance())/2, false)));
			turn2 = Math.max(-0.05, Math.min(0.05, 
					pidAngle.update(model.getSonar2Distance() - model.getSonar1Distance(), false)));
			model.setMotors(forward - turn1 + turn2, forward + turn1 - turn2);
			System.out.println(model.getSonar1Distance());
			System.out.println(model.getSonar2Distance());
			//model.setMotors(0.2, 0.2);
			
			try {
				Thread.sleep(100);
			} catch (InterruptedException e) { }
		}
	}
	
	public double calcAngle(double distance1, double distance2, double constant) {
		return Math.asin((distance2-distance1)/constant);
	}
}
