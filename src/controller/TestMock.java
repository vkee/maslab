package controller;

import vision.Vision;
import comm.MapleComm;
import comm.MapleIO;
import devices.actuators.Cytron;
import devices.actuators.DigitalOutput;
import devices.actuators.PWMOutput;
import devices.sensors.Encoder;
import devices.sensors.Ultrasonic;

public class TestMock {

	public static void main(String[] args) {
	    Vision vision = new Vision(1, 320, 240, true);
	    
		MapleComm comm = new MapleComm(MapleIO.SerialPortType.WINDOWS);
        Cytron motorL = new Cytron(4, 0);
        Cytron motorR = new Cytron(10, 1);
        Ultrasonic sonarA = new Ultrasonic(30, 29);
        Ultrasonic sonarB = new Ultrasonic(32, 31);
        Ultrasonic sonarC = new Ultrasonic(34, 33);
        Ultrasonic sonarL = new Ultrasonic(36, 35);
        Ultrasonic sonarR = new Ultrasonic(26, 25);
        DigitalOutput relay = new DigitalOutput(37);
        
		comm.registerDevice(motorL);
		comm.registerDevice(motorR);
        comm.registerDevice(sonarA);
        comm.registerDevice(sonarB);
        comm.registerDevice(sonarC);
        comm.registerDevice(sonarL);
        comm.registerDevice(sonarR);
        comm.registerDevice(relay);
        
		comm.initialize();
		
		relay.setValue(false);
		comm.transmit();
		
		comm.updateSensorData();
		
		double distanceL = sonarL.getDistance();
		double distanceR = sonarR.getDistance();
		double distanceA = sonarA.getDistance();
		double distanceB = sonarB.getDistance();
		double distanceC = sonarC.getDistance();
		
		double target_x, target_y;
		double forward, turn;
		PID pid_align = new PID(0.1, 0.5, 0.08, 0.01);
		pid_align.update(distanceL, true);
		
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
		    
		    vision.update();
		    try {
	            target_x = vision.getNextBallX();
	            target_y = vision.getNextBallY();
	            PID pidX = new PID(160, 0.4, 0.3, 0);
	            PID pidY = new PID(0.95*240, 0.4, 0.3, 0);
	            double temp_x, temp_y;
	            pidX.update(target_x, true);
	            while (target_y < 0.9*240){
	                vision.update();
	                temp_x = target_x;
	                temp_y = target_y;
	                try {
	                    target_x = vision.getNextBallX();
	                    target_y = vision.getNextBallY();
	                    turn = Math.max(-0.1, Math.min(0.1, pidY.update(target_y, false)/240));
	                    forward = Math.max(-0.1, Math.min(0.1, -pidX.update(target_x, false)/320));
	                    System.out.println("HEADING FOR A BALL");
	                } catch (Exception exc){
	                    target_x = temp_x;
	                    target_y = temp_y;
	                    turn = 0.06;
	                    forward = 0;
	                    System.out.println("LOST TRACK OF THE BALL");
	                }
	                motorL.setSpeed(-(forward + turn));
	                motorR.setSpeed(forward - turn);
	                comm.transmit();
	            }
	            for (int i = 0; i < 8; i++){
	                forward = 0.05;
	                turn = 0;
	                motorL.setSpeed(-(forward + turn));
                    motorR.setSpeed(forward - turn);
                    comm.transmit();
                    try{
                        Thread.sleep(10);
                    } catch (Exception exc){
                        exc.printStackTrace();
                    }
	            }
		    } catch (Exception exc){
		        exc.printStackTrace();
		    }
		    
		    if (distanceC < 0.2){
		        System.out.println("WALL AHEAD");
                turn = 0.12;
                forward = 0;
		    } else if (distanceB < 0.2 || (distanceA < 0.22 && distanceB < 0.22)){
		        // Maybe eliminate second condition
		        System.out.println("TURNING");
                turn = 0.12;
                forward = 0;
		    } else if (distanceL < 0.13){
		        System.out.println("TOO CLOSE ON THE LEFT");
		        turn = 0.05;
		        forward = 0.07;
		    } else if (distanceR < 0.1){
		        System.out.println("TOO CLOSE ON THE RIGHT");
		        turn = -0.08;
		        forward = 0;
		    } else {
		        System.out.println("DEFAULT");
		        turn = Math.max(-0.1, Math.min(0.1, pid_align.update(distanceL, false)));
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
