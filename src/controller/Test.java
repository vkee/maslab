package controller;

import comm.MapleComm;
import comm.MapleIO;
import devices.actuators.Cytron;
import devices.actuators.DigitalOutput;
import devices.actuators.PWMOutput;
import devices.sensors.Encoder;
import devices.sensors.Ultrasonic;

public class Test {

	public static void main(String[] args) {
		MapleComm comm = new MapleComm(MapleIO.SerialPortType.WINDOWS);
        Cytron motorL = new Cytron(4, 0);
        Cytron motorR = new Cytron(10, 1);
        Ultrasonic sonarA = new Ultrasonic(30, 29);
        Ultrasonic sonarB = new Ultrasonic(32, 31);
        Ultrasonic sonarC = new Ultrasonic(34, 33);
        Ultrasonic sonarL = new Ultrasonic(35, 36);
        Ultrasonic sonarR = new Ultrasonic(26, 25);
        
        Encoder encoderL = new Encoder(5, 7);
        Encoder encoderR = new Encoder(6, 8);
        
        DigitalOutput relay = new DigitalOutput(37);
        
		comm.registerDevice(motorL);
		comm.registerDevice(motorR);
        comm.registerDevice(sonarA);
        comm.registerDevice(sonarB);
        comm.registerDevice(sonarC);
        comm.registerDevice(sonarL);
        comm.registerDevice(sonarR);
        comm.registerDevice(relay);
        
        comm.registerDevice(encoderL);
        comm.registerDevice(encoderR);
        
		comm.initialize();
		
		relay.setValue(false);
		comm.transmit();
		
		comm.updateSensorData();
		
		double distanceL = sonarL.getDistance();
		double distanceR = sonarR.getDistance();
		double distanceA = sonarA.getDistance();
		double distanceB = sonarB.getDistance();
		double distanceC = sonarC.getDistance();
		
		while (true) {
		    comm.updateSensorData();
            
		    distanceL = sonarL.getDistance();
		    distanceR = sonarR.getDistance();
		    distanceA = sonarA.getDistance();
		    distanceB = sonarB.getDistance();
		    distanceC = sonarC.getDistance();
		    
		    System.out.println("DistanceL: " + distanceL);
		    System.out.println("DistanceR: " + distanceR);
		    System.out.println("DistanceA: " + distanceA);
		    System.out.println("DistanceB: " + distanceB);
		    System.out.println("DistanceC: " + distanceC);
		    
			comm.transmit();
			
			try {
                Thread.sleep(200);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
		}
	}
}
