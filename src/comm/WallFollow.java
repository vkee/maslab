package comm;

import controller.PID;
import devices.actuators.Cytron;
import devices.sensors.Ultrasonic;

public class WallFollow {
	public static void main(String[] args) {
		new WallFollow();
		System.exit(0);
	}

	public WallFollow() {
		
		/*
		 * Create your Maple communication framework by specifying what kind of 
		 * serial port you would like to try to autoconnect to.
		 */
		// MapleComm comm = new MapleComm(MapleIO.SerialPortType.SIMULATION);
		MapleComm comm = new MapleComm(MapleIO.SerialPortType.WINDOWS);
		/*
		 * Create an object for each device. The constructor arguments specify
		 * their pins (or, in the case of the gyroscope, the index of a fixed
		 * combination of pins).
		 * Devices are generally either Sensors or Actuators. For example, a
		 * motor controller is an actuator, and an encoder is a sensor.
		 */
		Cytron motor1 = new Cytron(2, 1);
		Cytron motor2 = new Cytron(7, 6);

		Ultrasonic sonar1 = new Ultrasonic(26, 24);
		Ultrasonic sonar2 = new Ultrasonic(30, 29);
		//Gyroscope gyro = new Gyroscope(1, 9);
		

		/*
		 * Build up a list of devices that will be sent to the Maple for the
		 * initialization step.
		 */
		comm.registerDevice(motor1);
		comm.registerDevice(motor2);
		comm.registerDevice(sonar1);
		comm.registerDevice(sonar2);

		// Send information about connected devices to the Maple
		comm.initialize();

		double forward = 0.1; 
		
		comm.updateSensorData();
		PID pid_center = new PID(0.15,0.1,0.0,0.0);
		PID pid_angle = new PID(0,0.01,0.0,0.0);
		
		double turn_center = pid_center.update(sonar1.getDistance(), true);
		double turn_angle = 0;
		
		motor1.setSpeed(forward + turn_center);
		motor2.setSpeed(forward - turn_center);
		comm.transmit();
		
		while (true) {
			comm.updateSensorData();
			//System.out.println(sonar1.getDistance() + " " + sonar2.getDistance());
			
			turn_center = Math.max(-0.05, Math.min(0.05, 
					pid_center.update((sonar1.getDistance()+sonar2.getDistance())/2, false)));
//			turn_angle = Math.max(-0.05, Math.min(0.05, 
//					pid_angle.update(sonar1.getDistance() - sonar2.getDistance(), false)));
			turn_angle = 0;
			
			System.out.println((sonar1.getDistance()+sonar2.getDistance())/2);
			System.out.println("turn_angle: " + turn_angle);
			System.out.println("turn_center: " + turn_center);
			
			motor1.setSpeed(forward + turn_center + turn_angle);
			motor2.setSpeed(forward - turn_center - turn_angle);
			
			comm.transmit();
			
			try {
				Thread.sleep(100);
			} catch (InterruptedException e) { }
		}
	}
}