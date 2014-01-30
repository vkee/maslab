package controller;

import vision.Vision;
import comm.MapleComm;
import comm.MapleIO;
import devices.actuators.Cytron;
import devices.actuators.DigitalOutput;
import devices.sensors.Encoder;
import devices.sensors.Ultrasonic;

public class ReactorAlign {

	public static void main(String[] args) {
		MapleComm comm = new MapleComm(MapleIO.SerialPortType.WINDOWS);
        Hopper hopper = new Hopper(comm,24,27,28, 14);
        Cytron motorL = new Cytron(4, 0);
        Cytron motorR = new Cytron(3, 1);
        Ultrasonic sonarA = new Ultrasonic(26, 25);
        Ultrasonic sonarB = new Ultrasonic(34, 33);
        Ultrasonic sonarC = new Ultrasonic(35, 36);
        Ultrasonic sonarD = new Ultrasonic(30, 29);
        Ultrasonic sonarE = new Ultrasonic(32, 31);
        
        Encoder encoderL = new Encoder(5, 7);
        Encoder encoderR = new Encoder(6, 8);

        DigitalOutput relay = new DigitalOutput(37);

        comm.registerDevice(motorL);
        comm.registerDevice(motorR);
        comm.registerDevice(sonarA);
        comm.registerDevice(sonarB);
        comm.registerDevice(sonarC);
        comm.registerDevice(sonarD);
        comm.registerDevice(sonarE);
        comm.registerDevice(encoderL);
        comm.registerDevice(encoderR);
        comm.registerDevice(relay);
        comm.initialize();
        relay.setValue(false);
        comm.transmit();

        comm.updateSensorData();

        double distanceD = sonarD.getDistance();
        double distanceE = sonarE.getDistance();
        double distanceA = sonarA.getDistance();
        double distanceB = sonarB.getDistance();
        double distanceC = sonarC.getDistance();
        double distance = 0.0;
        final Vision vision = new Vision(1, 320, 240, true);


        double distanceLeft = vision.getLeftmostWallDistance();
        double distanceRight = vision.getRightmostWallDistance();
    	double distanceReactor = vision.getNextReacterDistance();
        
        double forward, turn;
        hopper.rampClose();
        hopper.pacmanClose();
        hopper.sorterBlocking();
        forward = 0;
        turn = 0;
        int width = 320;
        int height = 240;

        vision.update();

        PID pid_align = new PID(width/2, 0.3, 0, 0);
        PID pid_distance = new PID(0.05, 1.5, -0.2, 0);
        pid_align.update(0, true);
        pid_distance.update(0.05, true);

        while (true) {
        	comm.updateSensorData();
        	vision.update();
        	distanceD = sonarD.getDistance();
            distanceE = sonarE.getDistance();
        	distance = Math.min(distanceD, distanceE);
        	distanceReactor = vision.getNextReacterDistance();
        	distanceLeft = vision.getLeftmostWallDistance();
            distanceRight = vision.getRightmostWallDistance();
            System.out.println("DistanceReactor: " + distanceReactor);
            System.out.println("DistanceLeft: " + distanceLeft);
            System.out.println("DistanceRight: " + distanceRight);
            System.out.println("Distance: " + distance);
            System.out.println("EncoderL: " + (-encoderL.getAngularSpeed()));
            System.out.println("EncoderR: " + encoderR.getAngularSpeed());
            
            
            /*
            if (distanceLeft < 0.1 && distanceRight > 0.3 && distanceReactor > 0.2) {
            	forward = -0.1;
            	turn = 0.2;
            	motorL.setSpeed(-(forward + turn));
                motorR.setSpeed(forward - turn);
                comm.transmit();
            	System.out.println("forward: " + forward);
                System.out.println("turn: " + turn);
//                System.out.println("DistanceReactor: " + distanceReactor);
//                System.out.println("DistanceLeft: " + distanceLeft);
                continue;
            }*/
            
            if (distanceLeft + 0.1 < distanceReactor /*&& (distanceLeft + 0.5 < distanceRight || distanceReactor > 0.2)*/ ) {
            	forward = 0.13;
            	turn = 0;
            	motorL.setSpeed(-(forward + turn));
                motorR.setSpeed(forward - turn);
                comm.transmit();
            	System.out.println("forward: " + forward);
                System.out.println("turn: " + turn);
//                System.out.println("DistanceReactor: " + distanceReactor);
//                System.out.println("DistanceLeft: " + distanceLeft);
                continue;
            }
            
            if (distanceRight + 0.1 < distanceReactor /*&& distanceRight + 0.5 < distanceLeft*/ ) {
            	forward = 0.13;
            	turn = 0;
            	motorL.setSpeed(-(forward + turn));
                motorR.setSpeed(forward - turn);
                comm.transmit();
                System.out.println("forward: " + forward);
                System.out.println("turn: " + turn);
//                System.out.println("DistanceReactor: " + distanceReactor);
//                System.out.println("DistanceRight: " + distanceRight);
                continue;
            }
        	
        	double align = pid_align.update(vision.getNextReactorX(), false)/width;
        	turn = Math.min(0.2, Math.max(-0.2, -align));
        	//double forwardPIDout = -pid_distance.update(distance, false);
        	//forward = Math.min(0.15, Math.max(-0.15, forwardPIDout));
        	if (distance < 0.05 ) {
        		forward = 0;
        		if (turn < 0.05) {
        			motorL.setSpeed(0);
                    motorR.setSpeed(0);
                    comm.transmit();
                    break;
        		}
        	} else if (distance < 0.1) {
        		forward = distance*1.5;
        	} else {
        		forward = 0.13;
        	}
        	
        	
//       	System.out.println(align);
//        	System.out.println("distance: " + distance);
//        	System.out.println("reactor: " + vision.getNextReactorX());
        	System.out.println("forward: " + forward);
            System.out.println("turn: " + turn);

            motorL.setSpeed(-(forward + turn));
            motorR.setSpeed(forward - turn);
            comm.transmit();
            
//            try {
//				Thread.sleep(10);
//			} catch (InterruptedException e) {
//				// TODO Auto-generated catch block
//				e.printStackTrace();
//			}
        }
        
        hopper.sorterGreen();
        try {
			Thread.sleep(2000);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
        hopper.rampHigh();
        hopper.pacmanOpen();
        try {
			Thread.sleep(2000);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
        motorL.setSpeed(0.15);
        motorR.setSpeed(-0.15);
        comm.transmit();
        try {
			Thread.sleep(2000);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
        
        //LOW
        hopper.rampLow();
        motorL.setSpeed(0);
        motorR.setSpeed(0);
        comm.transmit();
        hopper.pacmanClose();
        
        hopper.pacmanOpen();
        try {
			Thread.sleep(2000);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
        hopper.pacmanClose();
        hopper.sorterBlocking();
	}

}
