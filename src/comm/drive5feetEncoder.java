package comm;

import controller.PID;
import devices.actuators.Cytron;
import devices.sensors.Encoder;

public class drive5feetEncoder {
	public static void main(String[] args) {
		new drive5feetEncoder();
	}
	
	public drive5feetEncoder() {
		MapleComm comm = new MapleComm(MapleIO.SerialPortType.WINDOWS);
		Encoder encoder1 = new Encoder(18, 17);
		Encoder encoder2 = new Encoder(19, 20);
		//Gyroscope gyro = new Gyroscope(1, 8);
		Cytron motor1 = new Cytron(2, 1);
		Cytron motor2 = new Cytron(7, 6);
		
		comm.registerDevice(encoder1);
		comm.registerDevice(encoder2);
		comm.registerDevice(motor1);
		comm.registerDevice(motor2);
		
		System.out.println("Initializing");
		comm.initialize();
		
		double forward = 0.1; 
		
		comm.updateSensorData();
		PID pid = new PID(0,0.001,0.0,0.0);
		
		
		motor1.setSpeed(0.1);
		pid.update(encoder2.getTotalAngularDistance()-encoder1.getTotalAngularDistance(), true);
		motor2.setSpeed(forward);
		comm.transmit();
		
		while (encoder2.getTotalAngularDistance() < 60/3.875*2) {
			comm.updateSensorData();
			//System.out.println(angle);
			
			motor1.setSpeed(pid.update(encoder2.getTotalAngularDistance()-encoder1.getTotalAngularDistance(), false));
			motor2.setSpeed(forward);
			comm.transmit();
			System.out.println(encoder2.getTotalAngularDistance());
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
		
		while (encoder2.getTotalAngularDistance() > 0) {
			comm.updateSensorData();
			//System.out.println(angle);
			
			motor1.setSpeed(-pid.update(encoder2.getTotalAngularDistance()-encoder1.getTotalAngularDistance(), false));
			motor2.setSpeed(-forward);
			comm.transmit();
			System.out.println(encoder2.getTotalAngularDistance());
			try {
				Thread.sleep(10);
			} catch (InterruptedException e) { }
		}
		
	}
}