package comm;

import controller.PID;
import devices.actuators.Cytron;
import devices.sensors.Encoder;
import devices.sensors.Gyroscope;

public class drive5feetGyro {
	public static void main(String[] args) {
		new drive5feetGyro();
	}
	
	public drive5feetGyro() {
		MapleComm comm = new MapleComm(MapleIO.SerialPortType.WINDOWS);
		Encoder encoder = new Encoder(19, 20);
		Gyroscope gyro = new Gyroscope(1, 8);
		Cytron motor1 = new Cytron(2, 1);
		Cytron motor2 = new Cytron(7, 6);
		
		comm.registerDevice(encoder);
		comm.registerDevice(gyro);
		comm.registerDevice(motor1);
		comm.registerDevice(motor2);
		
		System.out.println("Initializing");
		comm.initialize();
		
		double forward = 0.1; 
		
		comm.updateSensorData();
		PID pid = new PID(0,0.03,0.0,0.0);
		
		double turn = pid.update(gyro.getOmega(), true);
		
		motor1.setSpeed(forward + turn);
		motor2.setSpeed(forward - turn);
		comm.transmit();
		
		while (encoder.getTotalAngularDistance() < 60/3.875*2) {
			comm.updateSensorData();
			//System.out.println(angle);
			turn = Math.max(-0.04, Math.min(0.04,pid.update(gyro.getOmega(), false)));
			
			motor1.setSpeed(forward + turn);
			motor2.setSpeed(forward - turn);
			comm.transmit();
			System.out.println(encoder.getTotalAngularDistance());
			try {
				Thread.sleep(10);
			} catch (InterruptedException e) { }
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
		
		while (encoder.getTotalAngularDistance() > 0) {
			comm.updateSensorData();
			//System.out.println(angle);
			turn = Math.max(-0.05, Math.min(0.05,pid.update(gyro.getOmega(), false)));
			
			motor1.setSpeed(- forward + turn);
			motor2.setSpeed(- forward - turn);
			comm.transmit();
			System.out.println(encoder.getTotalAngularDistance());
			try {
				Thread.sleep(10);
			} catch (InterruptedException e) { }
		}
		
		motor1.setSpeed(0);
		motor2.setSpeed(0);
		comm.transmit();
	}
}
