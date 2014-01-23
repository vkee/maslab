package controller;

import comm.MapleComm;
import comm.MapleIO;
import devices.actuators.PWMOutput;

public class Test {

	public static void main(String[] args) {
		MapleComm comm = new MapleComm(MapleIO.SerialPortType.WINDOWS);
		PWMOutput left = new PWMOutput(0);
		PWMOutput right = new PWMOutput(1);
		comm.registerDevice(left);
		comm.registerDevice(right);
		comm.initialize();
		
		while (true) {
			left.setValue(0.5);
			right.setValue(0.5);
			comm.transmit();
		}
	}
}
