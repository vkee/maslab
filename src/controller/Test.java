package controller;

import java.util.LinkedList;
import java.util.List;

import comm.MapleComm;
import comm.MapleIO;
import devices.actuators.Cytron;
import devices.actuators.DigitalOutput;
import devices.actuators.PWMOutput;
import devices.sensors.Encoder;
import devices.sensors.Gyroscope;
import devices.sensors.Ultrasonic;

public class Test {

	public static void main(String[] args) {
		MapleComm comm = new MapleComm(MapleIO.SerialPortType.WINDOWS);
        Cytron motorL = new Cytron(4, 0);
        Cytron motorR = new Cytron(3, 1);
        Ultrasonic sonarA = new Ultrasonic(30, 29);
        Ultrasonic sonarB = new Ultrasonic(32, 31);
        Ultrasonic sonarC = new Ultrasonic(34, 33);
        Ultrasonic sonarL = new Ultrasonic(35, 36);
        Ultrasonic sonarR = new Ultrasonic(26, 25);
        
        Encoder encoderL = new Encoder(5, 7);
        Encoder encoderR = new Encoder(6, 8);
        
        //Gyroscope gyro = new Gyroscope(1, 9);
        
        DigitalOutput relay = new DigitalOutput(37);
        
		comm.registerDevice(motorL);
		comm.registerDevice(motorR);
        comm.registerDevice(sonarA);
        comm.registerDevice(sonarB);
        comm.registerDevice(sonarC);
        comm.registerDevice(sonarL);
        comm.registerDevice(sonarR);
        //comm.registerDevice(gyro);
        comm.registerDevice(relay);
        
        comm.registerDevice(encoderL);
        comm.registerDevice(encoderR);
        
		comm.initialize();
		
		relay.setValue(false);
		comm.transmit();

		comm.updateSensorData();

//		double time = System.currentTimeMillis();
//		double prevTime = System.currentTimeMillis();
//		double gyroError = 0;
//		while (System.currentTimeMillis() - time < 5000) {
//		    comm.updateSensorData();
//		    gyroError += gyro.getOmega() * (System.currentTimeMillis() - prevTime)/5000;
//		    prevTime = System.currentTimeMillis();
//		}
		
		double distanceL = sonarL.getDistance();
		double distanceR = sonarR.getDistance();
		double distanceA = sonarA.getDistance();
		double distanceB = sonarB.getDistance();
		double distanceC = sonarC.getDistance();

		//double angle;
		
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
		    
		    //motorL.setSpeed(-0.1);
		    //motorR.setSpeed(-0.1);
		    
//		    angle = gyro.getOmega() - gyroError;
//		    System.out.println(angle);
		    
			comm.transmit();
			
			try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
		}
	}
}
