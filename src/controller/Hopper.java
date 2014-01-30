package controller;

import comm.MapleComm;
import comm.MapleIO;
import devices.actuators.Servo3001HB;
import devices.actuators.Servo6001HB;

public class Hopper {
    Servo3001HB sorter;
    Servo6001HB pacman;
    Servo3001HB gate;
    Servo6001HB ramp;
    MapleComm comm;
    double pacmanAngle, gateAngle;

    public Hopper(MapleComm comm, int sorterPin, int pacmanPin, int gatePin, int rampPin) {
        sorter = new Servo3001HB(sorterPin);
        pacman = new Servo6001HB(pacmanPin);
        gate = new Servo3001HB(gatePin);
        ramp = new Servo6001HB(rampPin);
        pacmanAngle = 120;
        gateAngle = 70;
        
        this.comm = comm;
        comm.registerDevice(sorter);
        comm.registerDevice(pacman);
        comm.registerDevice(gate);
        comm.registerDevice(ramp);
    }

    public void sorterRed() {
        sorter.setAngle(50);
        comm.transmit();
    }
    
    public void fastRedSort(){
    	sorter.setAngle(50);
        try {
			Thread.sleep(100);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
        sorter.setAngle(90);
        comm.transmit();
    }

    public void sorterGreen() {
        sorter.setAngle(150);
        comm.transmit();
    }
    
    public void fastGreenSort(){
    	
    	System.out.println("Start");
    	sorter.setAngle(150);
        comm.transmit();
        
		try {
			Thread.sleep(450);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
        
        sorter.setAngle(90);
        System.out.println("Finish");
        comm.transmit();
        
		try {
			Thread.sleep(500);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
    }

    public void sorterBlocking() {
        sorter.setAngle(90);
        comm.transmit();
    }

    public void pacmanOpen() {
    	while (pacmanAngle > 40) {
    		pacman.setAngle(pacmanAngle);
    		comm.transmit();
    		pacmanAngle -= 5;
    		try {
				Thread.sleep(30);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
    	}
        pacman.setAngle(40);
        pacmanAngle = 40;
        comm.transmit();
    }

    public void pacmanClose() {
    	while (pacmanAngle < 120) {
    		pacman.setAngle(pacmanAngle);
    		comm.transmit();
    		pacmanAngle += 5;
    		try {
				Thread.sleep(30);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
    	}
        pacman.setAngle(120);
        pacmanAngle = 120;
        comm.transmit();
    }

    public void gateOpen() {
    	while (gateAngle < 120) {
    		gate.setAngle(gateAngle);
    		comm.transmit();
    		gateAngle += 5;
    		try {
				Thread.sleep(30);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
    	}
        gate.setAngle(120);
        gateAngle = 120;
        comm.transmit();
    }

    public void gateClose() {
    	while (gateAngle > 70) {
    		gate.setAngle(gateAngle);
    		comm.transmit();
    		gateAngle -= 5;
    		try {
				Thread.sleep(30);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
    	}
        gate.setAngle(70);
        gateAngle = 70;
        comm.transmit();
    }

    public void rampHigh() {
        ramp.setAngle(110);
        comm.transmit();
    }
    
    public void rampLow() {
        ramp.setAngle(180);
        comm.transmit();
    }

    public void rampClose() {
        ramp.setAngle(0);
        comm.transmit();
    }

    public static void main(String[] args) {
    	MapleComm comm = new MapleComm(MapleIO.SerialPortType.WINDOWS);
        Hopper hopper = new Hopper(comm,24,27,28, 14);
        comm.initialize();
//        hopper.rampHigh();
////      hopper.rampLow();
////      hopper.rampClose();
//      hopper.gateOpen();
////      hopper.gateClose();
//        
//        hopper.sorterRed();
////        hopper.sorterGreen();
////        hopper.sorterBlocking();
////        hopper.pacmanOpen();
//        hopper.pacmanClose();
        
        while (true) {
        	
        	hopper.fastGreenSort();
        	
//            hopper.pacmanOpen();
//            hopper.gateOpen();
            try {
				Thread.sleep(1000);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
//            hopper.pacmanClose();
//            hopper.gateClose();
//            try {
//				Thread.sleep(2000);
//			} catch (InterruptedException e) {
//				// TODO Auto-generated catch block
//				e.printStackTrace();
//			}
        	
        	
        }
    }

}
