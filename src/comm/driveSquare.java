package comm;

import controller.PID;
import devices.actuators.Cytron;
import devices.sensors.Encoder;
import devices.sensors.Gyroscope;
import devices.sensors.Ultrasonic;

public class driveSquare {
	public static void main(String[] args) {
		new driveSquare();
	}
	
	public driveSquare() {
		MapleComm comm = new MapleComm(MapleIO.SerialPortType.WINDOWS);
//		Encoder encoder1 = new Encoder(5, 6);
		Encoder encoder2 = new Encoder(6, 8);
		Gyroscope gyro = new Gyroscope(1, 9);
		Cytron motor1 = new Cytron(4, 0);
		Cytron motor2 = new Cytron(10, 1);
//		Ultrasonic sonar1 = new Ultrasonic(26,25);
//		Ultrasonic sonar2 = new Ultrasonic(30,29);
//		Ultrasonic sonar3 = new Ultrasonic(32,31);
//		Ultrasonic sonar4 = new Ultrasonic(34,33);
//		Ultrasonic sonar5 = new Ultrasonic(36,35);
		
		comm.registerDevice(encoder2);
		comm.registerDevice(gyro);
		comm.registerDevice(motor1);
		comm.registerDevice(motor2);
		
		System.out.println("Initializing");
		comm.initialize();
		
		double time = System.currentTimeMillis();
		double prevTime = System.currentTimeMillis();
		double gyroError = 0;
		while (System.currentTimeMillis() - time < 5000) {
			comm.updateSensorData();
			gyroError += gyro.getAngularSpeed() * (System.currentTimeMillis() - prevTime)/5000;
			prevTime = System.currentTimeMillis();
		}
		
		double forward = 0.1; 
		
		comm.updateSensorData();
		
		PID pid = new PID(0.0,0.05,0.0,0.0);
		
		double turn = pid.update(gyro.getAngularSpeed(), true);
		double c = 0;
		double angle = 0;
		//motor1.setSpeed(forward + turn);
		//motor2.setSpeed(forward - turn);
		//comm.transmit();
		
		for (int i = 0; i < 4; i++) {
			time = System.currentTimeMillis();
			while (encoder2.getTotalAngularDistance() - c < 8) {
				comm.updateSensorData();
				angle += (System.currentTimeMillis() - time) * (gyro.getAngularSpeed()-gyroError) / 1000;
				time = System.currentTimeMillis();
				System.out.println(angle);
				turn = Math.max(-0.05, Math.min(0.05,pid.update(angle-i*Math.PI/2, false)));

				motor1.setSpeed(forward + turn);
				motor2.setSpeed(forward - turn);
				comm.transmit();

				System.out.println("turn: " + turn);
				System.out.println(encoder2.getTotalAngularDistance());
//				try {
//					Thread.sleep(10);
//				} catch (InterruptedException e) { }
			}

			while (angle-i*Math.PI/2 < Math.PI/2) {
				comm.updateSensorData();
				angle += (System.currentTimeMillis() - time) * (gyro.getAngularSpeed()-gyroError) / 1000;
				time = System.currentTimeMillis();
				System.out.println(angle);
				motor1.setSpeed(0.1);
				motor2.setSpeed(-0.1);
				comm.transmit();
//				try {
//					Thread.sleep(10);
//				} catch (InterruptedException e) { }
			}

			c = encoder2.getTotalAngularDistance();
		}
		

		motor1.setSpeed(0);
		motor2.setSpeed(0);
		comm.transmit();
		
	}
	
}
