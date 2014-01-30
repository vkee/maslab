package comm;

import devices.sensors.DigitalInput;

public class testIR {
	public static void main(String[] args) {
		MapleComm comm = new MapleComm(MapleIO.SerialPortType.WINDOWS);
		
		DigitalInput irSensor = new DigitalInput(10);
		
		comm.registerDevice(irSensor);

		comm.initialize();

		comm.transmit();

		comm.updateSensorData();

		boolean value = irSensor.getValue();

		while (true) {
			comm.updateSensorData();

			value = irSensor.getValue();

			System.out.println("Status: " + value);
			
			try {
				Thread.sleep(100);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
	}
}
