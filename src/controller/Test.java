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
        Ultrasonic sonarL = new Ultrasonic(36, 35);
        Ultrasonic sonarR = new Ultrasonic(26, 25);
        DigitalOutput relay = new DigitalOutput(37);
        
        //Encoder encoderL = new Encoder(5, 7);
        //Encoder encoderR = new Encoder(6, 8);
        
		comm.registerDevice(motorL);
		comm.registerDevice(motorR);
        comm.registerDevice(sonarA);
        comm.registerDevice(sonarB);
        comm.registerDevice(sonarC);
        comm.registerDevice(sonarL);
        comm.registerDevice(sonarR);
        comm.registerDevice(relay);
        
        //comm.registerDevice(encoderL);
        //comm.registerDevice(encoderR);
        
		comm.initialize();
		
		relay.setValue(false);
		comm.transmit();
		
		comm.updateSensorData();
		
		double distanceL = sonarL.getDistance();
		double distanceR = sonarR.getDistance();
		double distanceA = sonarA.getDistance();
		double distanceB = sonarB.getDistance();
		double distanceC = sonarC.getDistance();
		
		double forward, turn;
		PID pid_align = new PID(0.15, 0.3, 0.08, 0.01);
		pid_align.update(sonarL.getDistance(), true);
		
		long start_time, end_time;
		
		while (true) {
		    comm.updateSensorData();
		    
		    start_time = System.currentTimeMillis();
            
		    distanceL = sonarL.getDistance();
		    distanceR = sonarR.getDistance();
		    distanceA = sonarA.getDistance();
		    distanceB = sonarB.getDistance();
		    distanceC = sonarC.getDistance();
		    
		    //System.out.println("DistanceL: " + distanceL);
		    //System.out.println("DistanceR: " + distanceR);
		    //System.out.println("DistanceA: " + distanceA);
		    //System.out.println("DistanceB: " + distanceB);
		    //System.out.println("DistanceC: " + distanceC);
		    
		    if (distanceC < 0.2){
		        System.out.println("WALL AHEAD");
                turn = 0.12;
                forward = 0;
		    } else if (distanceA < 0.18 && distanceB < 0.18){
		        System.out.println("TURNING");
                turn = 0.12;
                forward = 0;
		    } else if (distanceL < 0.13){
		        System.out.println("TOO CLOSE ON THE LEFT");
		        turn = 0.05;
		        forward = 0.07;
		    } else if (distanceR < 0.08){
		        System.out.println("TOO CLOSE ON THE RIGHT");
		        turn = -0.08;
		        forward = 0;
		    } else {
		        System.out.println("DEFAULT");
		        turn = pid_align.update(distanceL, false);
		        forward = 0.08;
		    }
		    
			motorL.setSpeed(-(forward + turn));
			motorR.setSpeed(forward - turn);
			comm.transmit();
			
			end_time = System.currentTimeMillis();
			
			try {
                Thread.sleep(100 + start_time - end_time);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
		}
	}
}
