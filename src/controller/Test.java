package controller;

public class Test {

	public static void main(String[] args) {
		KitBotModel model = new KitBotModel();
		//model.setMotors(0.1, 0.1);
		try {
			Thread.sleep(1000);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		model.setMotors(128, 128);
		
		while (true) {
			model.updateData();
			System.out.println("sonar1: " + model.getSonar1Distance());
			System.out.println("sonar2: " + model.getSonar2Distance());
			try {
				Thread.sleep(1000);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
	}
}
