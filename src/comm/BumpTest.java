package comm;

import devices.sensors.AnalogInput;

public class BumpTest {

	public static void main(String[] args) {
		MapleComm comm = new MapleComm(MapleIO.SerialPortType.WINDOWS);
		AnalogInput bump = new AnalogInput(15);
		comm.registerDevice(bump);
		comm.initialize();
		while (true) {
			comm.updateSensorData();
			System.out.println(bump.getValue());
			try {
				Thread.sleep(1000);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
	}
}
