package comm;


import devices.actuators.Cytron;
import devices.actuators.DigitalOutput;
import devices.sensors.Encoder;
import devices.sensors.Gyroscope;
import devices.sensors.Ultrasonic;

public class Test {

	public static void main(String[] args) {
		new Test();
		System.exit(0);
	}

	public Test() {
		
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
		Encoder encoder1 = new Encoder(5, 6);
		Encoder encoder2 = new Encoder(7, 8);
		Gyroscope gyro = new Gyroscope(1, 9);
		Cytron motor1 = new Cytron(0, 1);
		Cytron motor2 = new Cytron(2, 3);
		Ultrasonic sonar1 = new Ultrasonic(26,25);
		Ultrasonic sonar2 = new Ultrasonic(30,29);
		Ultrasonic sonar3 = new Ultrasonic(32,31);
		Ultrasonic sonar4 = new Ultrasonic(34,33);
		Ultrasonic sonar5 = new Ultrasonic(36,35);
		
		DigitalOutput d = new DigitalOutput(0);

		/*
		 * Build up a list of devices that will be sent to the Maple for the
		 * initialization step.
		 */
		comm.registerDevice(encoder1);
		comm.registerDevice(encoder2);
		comm.registerDevice(motor1);
		comm.registerDevice(motor2);
		comm.registerDevice(gyro);
		comm.registerDevice(sonar1);
		comm.registerDevice(sonar2);
		comm.registerDevice(sonar3);
		comm.registerDevice(sonar4);
		comm.registerDevice(sonar5);
		comm.registerDevice(d);
		//comm.registerDevice(sonar2);

		// Send information about connected devices to the Maple
		comm.initialize();

		double time = System.currentTimeMillis();
		double prevTime = System.currentTimeMillis();
		double gyroError = 0;
		while (System.currentTimeMillis() - time < 5000) {
			comm.updateSensorData();
			System.out.println("gyro: " + gyro.getOmega());
			gyroError += gyro.getOmega() * (System.currentTimeMillis() - prevTime)/5000;
			prevTime = System.currentTimeMillis();
			try {
				Thread.sleep(100);
			} catch (InterruptedException e1) {
				// TODO Auto-generated catch block
				e1.printStackTrace();
			}
		}
		
		System.out.println("gyroError: " + gyroError);
		
		try {
			Thread.sleep(10000);
		} catch (InterruptedException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		}
		
		while (true) {
			
			// Request sensor data from the Maple and update sensor objects accordingly
			comm.updateSensorData();
//			d.setValue(true);
//			comm.transmit();
//			try {
//				Thread.sleep(1000);
//			} catch (InterruptedException e1) {
//				// TODO Auto-generated catch block
//				e1.printStackTrace();
//			}
//			d.setValue(false);
//			comm.transmit();
//			try {
//				Thread.sleep(1000);
//			} catch (InterruptedException e1) {
//				// TODO Auto-generated catch block
//				e1.printStackTrace();
//			}
//			System.out.println("sonar1: " + sonar1.getDistance());
//			System.out.println("sonar2: " + sonar2.getDistance());
//			System.out.println("sonar3: " + sonar3.getDistance());
//			System.out.println("sonar4: " + sonar4.getDistance());
//			System.out.println("sonar5: " + sonar5.getDistance());
			System.out.println("gyro: " + (gyro.getOmega()-gyroError));
			//System.out.println("gyro: " + gyro.getOmega());
//			motor1.setSpeed(0.1);
//			motor2.setSpeed(0.1);
			// All sensor classes have getters.
			//System.out.println(gyro.getOmega() + " " + ultra1.getDistance());
//			System.out.println(ultra1.getDistance() + " " + ultra2.getDistance());
			//System.out.println(enc.getTotalAngularDistance() + " " + enc.getAngularSpeed());
			
			// All actuator classes have setters.
			//motor1.setSpeed(0.2);
			//motor2.setSpeed(-0.3);

			// Request that the Maple write updated values to the actuators
			comm.transmit();
			
			// Just for console-reading purposes; don't worry about timing
			try {
				Thread.sleep(10);
			} catch (InterruptedException e) { }
		}
	}
}
