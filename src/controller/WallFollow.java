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
		Ultrasonic sonar1 = new Ultrasonic(1,2);
		//Ultrasonic sonar2 = new Ultrasonic(5,6);
		Cytron motor1 = new Cytron(7, 8);
		Cytron motor2 = new Cytron(9, 10);
		
		comm.registerDevice(sonar1);
		//comm.registerDevice(sonar2);
		comm.registerDevice(motor1);
		comm.registerDevice(motor2);
		
		System.out.println("Initializing");
		comm.initialize();
		
		double forward = 0.1; 
		
		comm.updateSensorData();
		PID pid = new PID(5.0,1.0,0.0,0.0);
		double turn = pid.update(sonar1.getDistance(), true);
		motor1.setSpeed(forward + turn);
		motor1.setSpeed(forward - turn);
		comm.transmit();
		
		while (true) {
			comm.updateSensorData();
			//System.out.println(sonar1.getDistance() + " " + sonar2.getDistance());
			
			turn = pid.update(sonar1.getDistance(), true);
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
