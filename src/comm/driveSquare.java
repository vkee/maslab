package comm;

import controller.PID;
import devices.actuators.Cytron;
import devices.sensors.Encoder;
import devices.sensors.Gyroscope;

public class driveSquare {
	public static void main(String[] args) {
		new driveSquare();
	}
	
	public driveSquare() {
		MapleComm comm = new MapleComm(MapleIO.SerialPortType.WINDOWS);
		Encoder encoder1 = new Encoder(18, 17);
		Encoder encoder2 = new Encoder(19, 20);
		Gyroscope gyro = new Gyroscope(1, 8);
		Cytron motor1 = new Cytron(2, 1);
		Cytron motor2 = new Cytron(7, 6);
		
		comm.registerDevice(encoder2);
		comm.registerDevice(gyro);
		comm.registerDevice(motor1);
		comm.registerDevice(motor2);
		
		System.out.println("Initializing");
		comm.initialize();
		
		double forward = 0.1; 
		
		comm.updateSensorData();
		
		PID pid = new PID(0,0.05,0.03,0.0);
		
		double turn = pid.update(gyro.getOmega(), true);
		double c = 0;
		double time = 0;
		double angle = 0;
		motor1.setSpeed(forward + turn);
		motor2.setSpeed(forward - turn);
		comm.transmit();
		
		for (int i = 0; i < 4; i++) {
			while (encoder2.getTotalAngularDistance() - c < 8) {
				comm.updateSensorData();
				//System.out.println(gyro.getOmega());
				turn = Math.max(-0.05, Math.min(0.05,pid.update(gyro.getOmega(), false)));

				motor1.setSpeed(forward + turn);
				motor2.setSpeed(forward - turn);
				comm.transmit();
				System.out.println(encoder2.getTotalAngularDistance());
				try {
					Thread.sleep(10);
				} catch (InterruptedException e) { }
			}

			angle = 0;
			time = System.currentTimeMillis();
			while (angle < Math.PI/2) {
				comm.updateSensorData();
				angle += (System.currentTimeMillis() - time) * gyro.getOmega() / 1000;
				time = System.currentTimeMillis();
				System.out.println(angle);
				motor1.setSpeed(0.1);
				motor2.setSpeed(-0.1);
				comm.transmit();
				try {
					Thread.sleep(10);
				} catch (InterruptedException e) { }
			}

			c = encoder2.getTotalAngularDistance();
		}
		

		motor1.setSpeed(0);
		motor2.setSpeed(0);
		comm.transmit();
		
	}
	
}
