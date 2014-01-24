package controller;

import vision.Vision;
import comm.MapleComm;
import comm.MapleIO;
import devices.actuators.Cytron;
import devices.actuators.DigitalOutput;
import devices.actuators.PWMOutput;
import devices.sensors.Encoder;
import devices.sensors.Ultrasonic;

public class BallTest {

	public static void main(String[] args) {
	    Vision vision = new Vision(1, 320, 240, true);
		MapleComm comm = new MapleComm(MapleIO.SerialPortType.WINDOWS);
        Cytron motorL = new Cytron(4, 0);
        Cytron motorR = new Cytron(10, 1);
        
		comm.registerDevice(motorL);
		comm.registerDevice(motorR);
        
		comm.initialize();
		
		comm.updateSensorData();
		
		boolean target_found;
		double target_x, target_y;
		double forward, turn;
		PID pid_target = new PID (160, 0.5, 0.3, 0);
		pid_target.update(0, true);
		
		while (true) {
		    vision.update();
		    comm.updateSensorData();
            
            try {
                target_x = vision.getNextBallX();
                target_y = vision.getNextBallY();
                target_found = true;
            } catch (Exception exc){
                target_x = 0;
                target_y = 0;
                target_found = false;
            }
            
            if (target_found){
                turn = - Math.max(-0.1, Math.min(0.1, pid_target.update(target_x, false)/320));
                forward = 0;
            } else {
                turn = 0;
                forward = 0;
            }
            
            motorL.setSpeed(-(forward + turn));
			motorR.setSpeed(forward - turn);
			comm.transmit();
		}
	}
}
