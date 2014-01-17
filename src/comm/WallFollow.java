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

		Ultrasonic sonar1 = new Ultrasonic(34, 33);
		Ultrasonic sonar2 = new Ultrasonic(30, 29);
		Ultrasonic sonar3 = new Ultrasonic(32, 31);
		double sonar1dist = 0;
		double sonar2dist = 0;
		double sonar3dist = 0;
//		Ultrasonic sonar3 = new Ultrasonic(30, 29);
//		Ultrasonic sonar2 = new Ultrasonic(32, 31);
		//Gyroscope gyro = new Gyroscope(1, 9);
		

		/*
		 * Build up a list of devices that will be sent to the Maple for the
		 * initialization step.
		 */
		comm.registerDevice(motor1);
		comm.registerDevice(motor2);
		comm.registerDevice(sonar1);
		comm.registerDevice(sonar2);
		comm.registerDevice(sonar3);
		//comm.registerDevice(sonar2);

		// Send information about connected devices to the Maple
		comm.initialize();

		double forward = 0.1; 
		
		//comm.updateSensorData();
		PID pid = new PID(0.2,0.2,0.0,0.0);
//		PID pid_angle = new PID(0,0.01,0.0,0.0);
		
		double turn = pid.update(sonar1dist + sonar2dist, true);
//		double turn_angle = 0;
		
//		motor1.setSpeed(forward + turn_center);
//		motor2.setSpeed(forward - turn_center);
//		comm.transmit();
		
		while (true) {
			comm.updateSensorData();
			sonar1dist = 0.3*sonar1.getDistance() + 0.7*sonar1dist;
			sonar2dist = 0.3*sonar2.getDistance() + 0.7*sonar2dist;
			sonar3dist = 0.3*sonar3.getDistance() + 0.7*sonar3dist;
//			while (true) {
//				comm.updateSensorData();
//				System.out.println("sonar1: " + sonar1.getDistance());
//				System.out.println("sonar2: " + sonar2.getDistance());
//				System.out.println("sonar3: " + sonar3.getDistance());
//			}
			
			while (sonar3dist < 0.3) {
				System.out.println("obstacle in front");
				comm.updateSensorData();
				sonar1dist = 0.3*sonar1.getDistance() + 0.7*sonar1dist;
				sonar2dist = 0.3*sonar2.getDistance() + 0.7*sonar2dist;
				sonar3dist = 0.3*sonar3.getDistance() + 0.7*sonar3dist;
				System.out.println("sonar1: " + sonar1dist);
				System.out.println("sonar2: " + sonar2dist);
				System.out.println("sonar3: " + sonar3dist);
				motor1.setSpeed(0.1);
				motor2.setSpeed(-0.1);
				//comm.updateSensorData();
				comm.transmit();
				try {
					Thread.sleep(100);
				} catch (InterruptedException e) { }
			}

			while (sonar1dist > 0.5 && sonar2dist > 0.5 && sonar3dist > 0.5) {
				System.out.println("cant find anything close");
				comm.updateSensorData();
				sonar1dist = 0.3*sonar1.getDistance() + 0.7*sonar1dist;
				sonar2dist = 0.3*sonar2.getDistance() + 0.7*sonar2dist;
				sonar3dist = 0.3*sonar3.getDistance() + 0.7*sonar3dist;
				System.out.println("sonar1: " + sonar1dist);
				System.out.println("sonar2: " + sonar2dist);
				System.out.println("sonar3: " + sonar3dist);
				motor1.setSpeed(0.1);
				motor2.setSpeed(-0.1);
				//comm.updateSensorData();
				comm.transmit();
				try {
					Thread.sleep(10);
				} catch (InterruptedException e) { }
			}

			while (sonar1dist < 0.07 && sonar2dist > 0.15) {
			System.out.println("back too close");
				comm.updateSensorData();
				sonar1dist = 0.3*sonar1.getDistance() + 0.7*sonar1dist;
				sonar2dist = 0.3*sonar2.getDistance() + 0.7*sonar2dist;
				sonar3dist = 0.3*sonar3.getDistance() + 0.7*sonar3dist;
				System.out.println("sonar1: " + sonar1dist);
				System.out.println("sonar2: " + sonar2dist);
				System.out.println("sonar3: " + sonar3dist);
				motor1.setSpeed(-0.1);
				motor2.setSpeed(0.1);
			//comm.updateSensorData();
				comm.transmit();
				try {
					Thread.sleep(10);
				} catch (InterruptedException e) { }
			}

			while (sonar2dist < 0.07 && sonar1dist > 0.15) {
				System.out.println("front too close");
				comm.updateSensorData();
				sonar1dist = 0.3*sonar1.getDistance() + 0.7*sonar1dist;
				sonar2dist = 0.3*sonar2.getDistance() + 0.7*sonar2dist;
				sonar3dist = 0.3*sonar3.getDistance() + 0.7*sonar3dist;
				System.out.println("sonar1: " + sonar1dist);
				System.out.println("sonar2: " + sonar2dist);
				System.out.println("sonar3: " + sonar3dist);
				motor1.setSpeed(0.1);
				motor2.setSpeed(-0.1);
				//comm.updateSensorData();
				comm.transmit();
				try {
					Thread.sleep(10);
				} catch (InterruptedException e) { }
			}


			System.out.println("normal");
			comm.updateSensorData();
			sonar1dist = 0.3*sonar1.getDistance() + 0.7*sonar1dist;
			sonar2dist = 0.3*sonar2.getDistance() + 0.7*sonar2dist;
			sonar3dist = 0.3*sonar3.getDistance() + 0.7*sonar3dist;
			turn = Math.max(-0.03, Math.min(0.03, pid.update(sonar1dist + sonar2dist, false)));
			System.out.println("sonar1: " + sonar1dist);
			System.out.println("sonar2: " + sonar2dist);
			System.out.println("sonar3: " + sonar3dist);
			motor1.setSpeed(forward + turn);
			motor2.setSpeed(forward - turn);
			System.out.println("motor1: " + Double.toString(forward + turn));
			System.out.println("motor2: " + Double.toString(forward - turn));
			comm.transmit();
			try {
				Thread.sleep(10);
			} catch (InterruptedException e) { }
		}

//		while (true) {
//			comm.updateSensorData();
//			//System.out.println(sonar1.getDistance() + " " + sonar2.getDistance());
//			
//			turn_center = Math.max(-0.05, Math.min(0.05, 
//					pid_center.update((sonar1.getDistance()+sonar2.getDistance())/2, false)));
//			turn_angle = Math.max(-0.05, Math.min(0.05, 
//					pid_angle.update(sonar1.getDistance() - sonar2.getDistance(), false)));
//			turn_angle = 0;
//			
//			System.out.println((sonar1.getDistance()+sonar2.getDistance())/2);
//			System.out.println("turn_angle: " + turn_angle);
//			System.out.println("turn_center: " + turn_center);
//			
//			motor1.setSpeed(forward + turn_center + turn_angle);
//			motor2.setSpeed(forward - turn_center - turn_angle);
//			
//			comm.transmit();
//			
//			try {
//				Thread.sleep(100);
//			} catch (InterruptedException e) { }
//		}
	}
}