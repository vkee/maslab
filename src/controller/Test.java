package controller;

import comm.MapleComm;
import comm.MapleIO;
import devices.actuators.Cytron;
import devices.actuators.PWMOutput;

public class Test {

	public static void main(String[] args) {
		MapleComm comm = new MapleComm(MapleIO.SerialPortType.WINDOWS);
        Cytron motorL = new Cytron(4, 0);
        Cytron motorR = new Cytron(10, 1);
		comm.registerDevice(motorL);
		comm.registerDevice(motorR);
		comm.initialize();
		
		while (true) {
			motorL.setSpeed(0.1);
			motorR.setSpeed(0.1);
			comm.transmit();
			try {
                Thread.sleep(10000);
            } catch (InterruptedException e) {
                // TODO Auto-generated catch block
                e.printStackTrace();
            }
		}
	}
}
