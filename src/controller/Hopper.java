package controller;

import devices.actuators.Servo3001HB;
import devices.actuators.Servo6001HB;

public class Hopper {
    Servo3001HB sorter;
    Servo6001HB pacman;
    Servo3001HB gate;
    Servo6001HB ramp;

    public Hopper(int sorterPin, int pacmanPin, int gatePin, int rampPin) {
        sorter = new Servo3001HB(sorterPin);
        pacman = new Servo6001HB(pacmanPin);
        gate = new Servo3001HB(gatePin);
        ramp = new Servo6001HB(rampPin);
    }

    public void sorterRed() {
        sorter.setAngle(0);
    }

    public void sorterGreen() {
        sorter.setAngle(45);
    }

    public void sorterBlocking() {
        sorter.setAngle(0);
    }

    public void pacmanOpen() {
        pacman.setAngle(45);
    }

    public void pacmanClose() {
        pacman.setAngle(0);
    }

    public void gateOpen() {
        gate.setAngle(0);
    }

    public void gateClose() {
        gate.setAngle(45);
    }

    public void rampHigh() {
        ramp.setAngle(0);
    }
    
    public void rampLow() {
        ramp.setAngle(45);
    }

    public void rampClose() {
        ramp.setAngle(30);
    }

    public static void main(String[] args) {
        Hopper hopper = new Hopper(24,27,28, 14);
        hopper.sorterRed();
        hopper.sorterRed();
        hopper.pacmanOpen();
        hopper.pacmanClose();
        hopper.gateOpen();
        hopper.gateClose();
        hopper.rampHigh();
        hopper.rampLow();
        hopper.rampClose();
    }

}
