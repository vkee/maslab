package controller;

import java.util.LinkedList;
import java.util.List;

import comm.MapleComm;
import comm.MapleIO;
import devices.actuators.Servo3001HB;
import devices.actuators.Servo6001HB;
import devices.sensors.DigitalInput;

public class Hopper {
	Servo3001HB sorter;
	Servo6001HB pacman;
	Servo3001HB gate;
	Servo6001HB ramp;
	MapleComm comm;
	double pacmanAngle, gateAngle;
	boolean value;
	DigitalInput irSensor;
	
	List<Integer> ball_colors;

	public Hopper(final MapleComm comm, int sorterPin, int pacmanPin, int gatePin, int rampPin, final List<Integer> ball_colors) {
		sorter = new Servo3001HB(sorterPin);
		pacman = new Servo6001HB(pacmanPin);
		gate = new Servo3001HB(gatePin);
		ramp = new Servo6001HB(rampPin);
		irSensor = new DigitalInput(10);

		pacmanAngle = 120;
		gateAngle = 70;
		
		this.ball_colors = ball_colors;

		this.comm = comm;
		comm.registerDevice(sorter);
		comm.registerDevice(pacman);
		comm.registerDevice(gate);
		comm.registerDevice(ramp);		
		comm.registerDevice(irSensor);
	}

	public void runHopper(){
		long ball_start_time, ball_end_time;
        while (true){
        	comm.updateSensorData();
        	ball_start_time = System.currentTimeMillis();
        	
        	System.out.println("BALL LIST LENGTH: " + ball_colors.size());
        	if (ball_colors.size() != 0){
        		if (ball_colors.get(0) == 0){
            		System.out.println("GREEN BALL READY");
        		} else {
        			System.out.println("RED BALL READY");
        		}
        	}
        	
            if (ballQueued()){
            	System.out.println("BALL QUEUED");
            	if (ball_colors.size() != 0){
                    if (ball_colors.get(0) == 0){
                        fastGreenSort();
                        //comm.transmit();
                        ball_colors.remove(0);
                    } else {
                    	System.out.println("SORTING RED");
                        fastRedSort();
                        //comm.transmit();
                        ball_colors.remove(0);
                    }
            	}
            }
            
            ball_end_time = System.currentTimeMillis();
//            comm.transmit();
            try {
                if (40 - ball_end_time + ball_start_time >= 0){
                    Thread.sleep(40 - ball_end_time + ball_start_time);
                } else {
                    System.out.println("TIME OVERFLOW: " + (ball_end_time - ball_start_time));
                }
            } catch (Exception exc){
                exc.printStackTrace();
            }
        }
	}
	
	public boolean ballQueued(){
		//System.out.println(irSensor.getValue());
	    return irSensor.getValue();
	}
	
	public void sorterRed() {
		sorter.setAngle(50);
	}

	public void fastRedSort(){
		System.out.println("Start");
		sorter.setAngle(50);
		comm.transmit();
		try {
			Thread.sleep(450);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}

		sorter.setAngle(90);
		comm.transmit();
		System.out.println("Finish");

		try {
			Thread.sleep(500);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

	public void sorterGreen() {
		sorter.setAngle(150);
	}

	public void fastGreenSort(){

		System.out.println("Start");
		//this.sorterGreen();
		sorter.setAngle(150);
		comm.transmit();
		try {
			Thread.sleep(450);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}

		//this.sorterBlocking();
		sorter.setAngle(90);
		comm.transmit();
		System.out.println("Finish");

		try {
			Thread.sleep(500);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
	}

	public void sorterBlocking() {
		sorter.setAngle(90);
	}

	public void pacmanOpen() {
		while (pacmanAngle > 40) {
			pacman.setAngle(pacmanAngle);
			pacmanAngle -= 5;
			try {
				Thread.sleep(30);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
		pacman.setAngle(40);
		pacmanAngle = 40;
	}

	public void pacmanClose() {
		while (pacmanAngle < 120) {
			pacman.setAngle(pacmanAngle);
			pacmanAngle += 5;
			try {
				Thread.sleep(30);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
		pacman.setAngle(120);
		pacmanAngle = 120;
	}

	public void gateOpen() {
		while (gateAngle < 120) {
			gate.setAngle(gateAngle);
			gateAngle += 5;
			try {
				Thread.sleep(30);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
		gate.setAngle(120);
		gateAngle = 120;
	}

	public void gateClose() {
		while (gateAngle > 70) {
			gate.setAngle(gateAngle);
			gateAngle -= 5;
			try {
				Thread.sleep(30);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
		gate.setAngle(70);
		gateAngle = 70;
	}

	public void rampHigh() {
		ramp.setAngle(110);
	}

	public void rampLow() {
		ramp.setAngle(180);
	}

	public void rampClose() {
		ramp.setAngle(0);
	}

	public static void main(String[] args){
		final MapleComm comm = new MapleComm(MapleIO.SerialPortType.WINDOWS);
		final List<Integer> ball_colors = new LinkedList<Integer>();
		final Hopper hopper = new Hopper(comm, 24, 27, 28, 14, ball_colors);
		comm.initialize();
		try {
			Thread.sleep(2000);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
//		hopper.fastGreenSort();
		Thread ball_thread = new Thread(new Runnable(){
			public void run(){
				hopper.runHopper();
			}
		});
		ball_thread.start();
		
		ball_colors.add(1);
		try {
			Thread.sleep(2000);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		ball_colors.add(0);
		
	}
	
//	public static void main(String[] args) {
//		MapleComm comm = new MapleComm(MapleIO.SerialPortType.WINDOWS);
//		Hopper hopper = new Hopper(comm,24,27,28, 14);
//
//		DigitalInput irSensor = new DigitalInput(10);
//
//		comm.registerDevice(irSensor);
//
//		comm.initialize();
//
//		comm.transmit();
//
//		comm.updateSensorData();
//
//		boolean ballInSorter = irSensor.getValue();
//		hopper.sorterBlocking();
//		//        hopper.rampHigh();
//		////      hopper.rampLow();
//		////      hopper.rampClose();
//		//      hopper.gateOpen();
//		////      hopper.gateClose();
//		//        
//		//        hopper.sorterRed();
//		////        hopper.sorterGreen();
//		////        hopper.sorterBlocking();
//		////        hopper.pacmanOpen();
//		//        hopper.pacmanClose();
//		
//		// Sorting the Balls up top using the IR sensor
//		while (true) {
//			comm.updateSensorData();
//
//			ballInSorter = irSensor.getValue();
//			System.out.println(ballInSorter);
//
//			if (ballInSorter == true){ // if there is a ball in the sorter
//				// if red ball
//				hopper.fastRedSort();
//				// else (green ball)
//				hopper.fastGreenSort();
//			}
//		}
//	}
}
