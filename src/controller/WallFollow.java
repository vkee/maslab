package controller;

import comm.MapleComm;
import comm.MapleIO;
import devices.actuators.Cytron;
import devices.sensors.Ultrasonic;

public class WallFollow {

	public static void main(String[] args) {
		new WallFollow();
	}
	
	public WallFollow() {
		MapleComm comm = new MapleComm(MapleIO.SerialPortType.WINDOWS);
		Ultrasonic sonar1 = new Ultrasonic(24,26);
		Ultrasonic sonar2 = new Ultrasonic(29,30);
		Cytron motor1 = new Cytron(2, 1);
		Cytron motor2 = new Cytron(7, 6);
		
		comm.registerDevice(sonar1);
		comm.registerDevice(sonar2);
		comm.registerDevice(motor1);
		comm.registerDevice(motor2);
		
		System.out.println("Initializing");
		comm.initialize();
		
		double forward = 0.1; 
		
		comm.updateSensorData();
		PID pid = new PID(300.0,1.0,0.0,0.0);
		double turn = pid.update(sonar1.getDistance(), true);
		motor1.setSpeed(forward + turn);
		motor1.setSpeed(forward - turn);
		comm.transmit();
		
		while (true) {
			comm.updateSensorData();
			//System.out.println(sonar1.getDistance() + " " + sonar2.getDistance());
			
			turn = pid.update(sonar1.getDistance(), true);
			System.out.println(sonar1.getDistance());
			motor1.setSpeed(forward + turn);
			motor1.setSpeed(forward - turn);
			
			comm.transmit();
			
			try {
				Thread.sleep(100);
			} catch (InterruptedException e) { }
		}
	}
	
	public double calcAngle(double distance1, double distance2, double constant) {
		return Math.asin((distance2-distance1)/constant);
	}
}
