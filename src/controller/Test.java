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
        Ultrasonic sonarA = new Ultrasonic(32, 31);
        Ultrasonic sonarB = new Ultrasonic(34, 33);
        Ultrasonic sonarL = new Ultrasonic(36, 35);
        Ultrasonic sonarR = new Ultrasonic(26, 25);
        DigitalOutput relay = new DigitalOutput(37);
        
        //Encoder encoderL = new Encoder(5, 7);
        //Encoder encoderR = new Encoder(6, 8);
        
		comm.registerDevice(motorL);
		comm.registerDevice(motorR);
        comm.registerDevice(sonarA);
        comm.registerDevice(sonarB);
        comm.registerDevice(sonarL);
        comm.registerDevice(sonarR);
        comm.registerDevice(relay);
        
        //comm.registerDevice(encoderL);
        //comm.registerDevice(encoderR);
        
		comm.initialize();
		
		relay.setValue(false);
		comm.transmit();
		
		comm.updateSensorData();
		
		int countL = 0;
		int countR = 0;
		int countA = 0;
		int countB = 0;
		double distanceL = 0;
		double distanceR = 0;
		double distanceA = 0;
		double distanceB = 0;
		double prev = 0;
		
		double forward, turn;
		PID pid_align = new PID(0.15, 0.3, 0.08, 0.01);
		pid_align.update(sonarL.getDistance(), true);
		
		long start_time, end_time;
		
		while (true) {
		    comm.updateSensorData();
		    
		    start_time = System.currentTimeMillis();
		    
//		    prev = distanceL;
//		    distanceL = sonarL.getDistance();
//		    if (prev == distanceL){
//		        countL++;
//		    } else {
//		        countL = 0;
//		    }
//		    try {
//		        Thread.sleep(30);
//		    } catch (InterruptedException e) {
//		        e.printStackTrace();
//		    }
//            prev = distanceR;
//            distanceR = sonarR.getDistance();
//            if (prev == distanceR){
//                countR++;
//            } else {
//                countL = 0;
//            }
//            try {
//                Thread.sleep(30);
//            } catch (InterruptedException e) {
//                e.printStackTrace();
//            }
//            prev = distanceA;
//            distanceA = sonarA.getDistance();
//            if (prev == distanceA){
//                countA++;
//            } else {
//                countA = 0;
//            }
//            try {
//                Thread.sleep(30);
//            } catch (InterruptedException e) {
//                e.printStackTrace();
//            }
//            prev = distanceB;
//            distanceL = sonarB.getDistance();
//            if (prev == distanceB){
//                countB++;
//            } else {
//                countB = 0;
//            }
//            try {
//                Thread.sleep(30);
//            } catch (InterruptedException e) {
//                e.printStackTrace();
//            }
//            
//            if (countL >= 20 || countR >= 20 || countA >= 20 || countB >= 20){
//                System.out.println("Power cycling...");
//                relay.setValue(true);
//                comm.transmit();
//                try {
//                    Thread.sleep(10);
//                } catch (InterruptedException e) {
//                    e.printStackTrace();
//                }
//                relay.setValue(false);
//                comm.transmit();
//                countL = 0;
//                countR = 0;
//                countA = 0;
//                countB = 0;
//            }
            
		    distanceL = sonarL.getDistance();
		    distanceR = sonarR.getDistance();
		    distanceA = sonarA.getDistance();
		    distanceB = sonarB.getDistance();
		    
		    //System.out.println("DistanceL: " + distanceL);
		    //System.out.println("DistanceR: " + distanceR);
		    //System.out.println("DistanceA: " + distanceA);
		    //System.out.println("DistanceB: " + distanceB);
		    
		    if (distanceA < 0.25 || distanceB < 0.25){
		        System.out.println("WALL AHEAD");
                turn = 0.12;
                forward = 0;
		    } else if (distanceL < 0.13){
		        System.out.println("TOO CLOSE ON THE LEFT");
		        turn = 0.05;
		        forward = 0.07;
//		    } else if (distanceL > 0.4){
//		        System.out.println("TOO FAR ON THE LEFT");
//		        turn = -0.05;
//		        forward = 0.05;
//		    } else if (distanceR < 0.25){
//		        System.out.println("ON THE RIGHT");
//		        turn = 0.08;
//		        forward = 0;
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
