package controller;

import devices.actuators.PWMOutput;

public class Hopper {

	PWMOutput sorter;
	PWMOutput pacman;
	PWMOutput gate;
	
	public Hopper(int sorterPin, int pacmanPin, int gatePin) {
		sorter = new PWMOutput(sorterPin);
		pacman = new PWMOutput(pacmanPin);
		gate = new PWMOutput(gatePin);
	}
	
	public void sorterRed() {
		sorter.setValue(0);
	}
	
	public void sorterGreen() {
		sorter.setValue(0);
	}
	
	public void sorterBlocking() {
		sorter.setValue(0);
	}
	
	public void pacmanOpen() {
		pacman.setValue(0);
	}
	
	public void pacmanClose() {
		pacman.setValue(0);
	}
	
	public void gateOpen() {
		gate.setValue(0);
	}
	
	public void gateClose() {
		gate.setValue(0);
	}
	
	public static void main(String[] args) {
		Hopper hopper = new Hopper(24,27,28);
		hopper.sorterRed();
		hopper.sorterRed();
		hopper.pacmanOpen();
		hopper.pacmanClose();
		hopper.gateOpen();
		hopper.gateClose();
	}

}
