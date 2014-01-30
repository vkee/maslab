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
        Cytron motorL = new Cytron(4, 0);
        Cytron motorR = new Cytron(3, 1);
        Ultrasonic sonarA = new Ultrasonic(26, 25);
        Ultrasonic sonarB = new Ultrasonic(34, 33);
        Ultrasonic sonarC = new Ultrasonic(35, 36);
        Ultrasonic sonarD = new Ultrasonic(30, 29);
        Ultrasonic sonarE = new Ultrasonic(32, 31);

        DigitalOutput relay = new DigitalOutput(37);

        comm.registerDevice(motorL);
        comm.registerDevice(motorR);
        comm.registerDevice(sonarA);
        comm.registerDevice(sonarB);
        comm.registerDevice(sonarC);
        comm.registerDevice(sonarD);
        comm.registerDevice(sonarE);
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

        double forward, turn;

        int width = 320;
        int height = 240;

        vision.update();

        PID pid_align = new PID(160, 0.3, -0.2, 0);
        PID pid_distance = new PID(0.05, 0.3, -0.2, 0);
        pid_align.update(0, true);
        pid_distance.update(0.05, true);

        while (true) {
        	comm.updateSensorData();
        	vision.update();
        	distance = Math.min(distanceD, distanceE);
        	forward = Math.max(0.2, Math.min(-0.2, pid_align.update(vision.getNextReactorX(), false)));
        	turn = Math.max(0.2, Math.min(-0.2, pid_align.update(vision.getNextReactorX(), false)));
        	
        	System.out.println("forward: " + forward);
            System.out.println("turn: " + turn);

            motorL.setSpeed(-(forward + turn));
            motorR.setSpeed(forward - turn);
        }
	}

}
