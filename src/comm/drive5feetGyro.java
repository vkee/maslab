package comm;

import controller.PID;
import devices.actuators.Cytron;
import devices.sensors.Encoder;
import devices.sensors.Gyroscope;

public class drive5feetGyro {
	public static void main(String[] args) {
		MapleComm comm = new MapleComm(MapleIO.SerialPortType.WINDOWS);
		
		Encoder encoder = new Encoder(19, 20);
		Encoder encoder1 = new Encoder(18, 17);
		Gyroscope gyro = new Gyroscope(1, 8);
		Cytron motor1 = new Cytron(2, 1);
		Cytron motor2 = new Cytron(7, 6);
		
		comm.registerDevice(encoder);
		comm.registerDevice(gyro);
		comm.registerDevice(motor1);
		comm.registerDevice(motor2);
		
		System.out.println("Initializing");
		comm.initialize();
		new drive5feetGyro(comm,encoder,encoder1,gyro,motor1,motor2,true);
		System.out.println("forward done");
		new drive5feetGyro(comm,encoder,encoder1,gyro,motor1,motor2,false);
	}
	
	public drive5feetGyro(MapleComm comm,Encoder encoder,Encoder encoder1, Gyroscope gyro, Cytron motor1, Cytron motor2, boolean forw) {
		

		double forward; 
		if (forw) {
			forward = 0.1;
		} else {
			System.out.println("1");
			forward = -0.1;
		}
		double angle = 0;
		double time = System.currentTimeMillis();
		comm.updateSensorData();
		PID pid = new PID(0,0.1,0.0,0.0);

		System.out.println("2");
		double turn = Math.max(-0.06, Math.min(0.06, pid.update(angle, true)));
		motor1.setSpeed(forward + turn);
		motor2.setSpeed(forward - turn);
		comm.transmit();
		
		if (forw) {		
			while (encoder.getTotalAngularDistance() < 60/3.875*2) {
				comm.updateSensorData();
				angle += (System.currentTimeMillis() - time) * (gyro.getOmega()) / 1000;
				time = System.currentTimeMillis();
				System.out.println("omega: " + gyro.getOmega());
				System.out.println("angle: " + angle);
				turn = Math.max(-0.06, Math.min(0.06, pid.update(angle, false)));

				motor1.setSpeed(forward + turn);
				motor2.setSpeed(forward - turn);
				comm.transmit();
				System.out.println(encoder.getTotalAngularDistance());

			}


			motor1.setSpeed(0);
			motor2.setSpeed(0);
			comm.transmit();

			try {
				Thread.sleep(1000);
			} catch (InterruptedException e1) {
				// TODO Auto-generated catch block
				e1.printStackTrace();
			}
		}
		
		else {
			while (encoder.getTotalAngularDistance() > -5) {
				comm.updateSensorData();
				angle += (System.currentTimeMillis() - time) * (gyro.getOmega()) / 1000;
				time = System.currentTimeMillis();
				System.out.println("omega: " + gyro.getOmega());
				System.out.println("angle: " + angle);
				turn = Math.max(-0.06, Math.min(0.06, pid.update(angle, false)));

				motor1.setSpeed(forward + turn);
				motor2.setSpeed(forward - turn);
				comm.transmit();
				System.out.println(encoder.getTotalAngularDistance());

			}


			motor1.setSpeed(0);
			motor2.setSpeed(0);
			comm.transmit();

			try {
				Thread.sleep(1000);
			} catch (InterruptedException e1) {
				// TODO Auto-generated catch block
				e1.printStackTrace();
			}
		}
		
//		comm.updateSensorData();
//		
//		while (encoder.getTotalAngularDistance() > 0) {
//			comm.updateSensorData();
//			angle += (System.currentTimeMillis() - time) * gyro.getOmega() / 1000;
//			time = System.currentTimeMillis();
//			//System.out.println(angle);
//			turn = Math.max(-0.06, Math.min(0.06,pid.update(angle, false)));
//			
//			motor1.setSpeed(- forward + turn);
//			motor2.setSpeed(- forward - turn);
//			comm.transmit();
//			System.out.println(encoder.getTotalAngularDistance());
//			try {
//				Thread.sleep(10);
//			} catch (InterruptedException e) { }
//		}
//		
//		motor1.setSpeed(0);
//		motor2.setSpeed(0);
//		comm.transmit();
	}
}
