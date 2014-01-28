package controller;

import java.awt.Point;
import java.awt.image.BufferedImage;
import java.util.LinkedList;
import java.util.List;

import javax.swing.ImageIcon;
import javax.swing.JLabel;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.highgui.Highgui;
import org.opencv.highgui.VideoCapture;

import Core.Engine;
import Core.FilterOp;
import vision.DetectionGL;
import vision.Mat2Image;
import vision.Vision;
import comm.MapleComm;
import comm.MapleIO;
import devices.actuators.Cytron;
import devices.actuators.DigitalOutput;
import devices.actuators.PWMOutput;
import devices.sensors.Encoder;
import devices.sensors.Gyroscope;
import devices.sensors.Ultrasonic;

public class TestBallCollectForward {

	public static void main(String[] args) {
		MapleComm comm = new MapleComm(MapleIO.SerialPortType.WINDOWS);
        Cytron motorL = new Cytron(4, 0);
        Cytron motorR = new Cytron(3, 1);
        Ultrasonic sonarA = new Ultrasonic(30, 29);
        Ultrasonic sonarB = new Ultrasonic(35, 36);
        Ultrasonic sonarC = new Ultrasonic(32, 31);
        Ultrasonic sonarD = new Ultrasonic(34, 33);
        Ultrasonic sonarE = new Ultrasonic(26, 25);
        
        Encoder encoderL = new Encoder(5, 7);
        Encoder encoderR = new Encoder(6, 8);
        
        //Gyroscope gyro = new Gyroscope(1, 9);
        
        DigitalOutput relay = new DigitalOutput(37);
        
		comm.registerDevice(motorL);
		comm.registerDevice(motorR);
        comm.registerDevice(sonarA);
        comm.registerDevice(sonarB);
        comm.registerDevice(sonarC);
        comm.registerDevice(sonarD);
        comm.registerDevice(sonarE);
        //comm.registerDevice(gyro);
        comm.registerDevice(relay);
        
        comm.registerDevice(encoderL);
        comm.registerDevice(encoderR);
        
		comm.initialize();
		
		relay.setValue(false);
		comm.transmit();
		
		comm.updateSensorData();
		
		final Vision vision = new Vision(1, 320, 240, true);
		
        PID pid_far = new PID(9, 0.3, 0.08, 0);
        pid_far.update(9, true);
        
        PID pid_mid = new PID(7, 0.2, 0.08, 0);
        pid_mid.update(7, true);
        
        PID pid_close = new PID(5, 0.2, 0.08, 0);
        pid_close.update(5, true);
        
        long start_time, end_time;
        double target_x, target_y, target_radius;
        double forward, turn, omega, abs_speed, K_encoder;
        turn = 0;

        int width = 320;
        int height = 240;
        
        vision.update();
        
        PID pidX = new PID(width/2, 0.3, -0.4, 0);
        PID pidY = new PID(0.95*height, 0.4, -0.2, 0);
        double pidOutX = pidX.update(vision.getNextBallX(), true);
        double pidOutY = pidY.update(vision.getNextBallY(), true);
        
        double update_value_x;
        
		while (true) {
		    start_time = System.currentTimeMillis();

		    comm.updateSensorData();
	        
		    vision.update();
            
		    target_x = vision.getNextBallX();
		    target_y = vision.getNextBallY();
		    target_radius = vision.getNextBallRadius();
		    
		    update_value_x = width/2 + ((target_x - width/2)*12/target_radius);
		    
            pidOutX = pidX.update(target_x, false);
            pidOutY = pidY.update(target_y, false);

            turn = Math.max(-0.1, Math.min(0.1, -pidOutX/width));
            forward = Math.max(-0.1, Math.min(0.1, pidOutY/height));
            
            if (target_y == 0.0 || target_y > 180) {
                System.out.println("COLLECTING BALL");
                forward = 0.13;
                abs_speed = Math.abs(encoderL.getAngularSpeed()) + Math.abs(encoderR.getAngularSpeed());
                K_encoder = Math.max(pid_close.update(abs_speed, false), 0.5);
                forward = forward*K_encoder;
                turn = 0;
            } else if (target_y > 150 && target_y <= 180){
                abs_speed = Math.abs(encoderL.getAngularSpeed()) + Math.abs(encoderR.getAngularSpeed());
                K_encoder = Math.max(pid_far.update(abs_speed, false), 0.5);
                forward = forward*K_encoder;
            } else if (target_y > 120 && target_y <= 150){
                abs_speed = Math.abs(encoderL.getAngularSpeed()) + Math.abs(encoderR.getAngularSpeed()); 
                K_encoder = Math.max(pid_mid.update(abs_speed, false), 0.5);
                forward = forward*K_encoder;
            } else {
                abs_speed = Math.abs(encoderL.getAngularSpeed()) + Math.abs(encoderR.getAngularSpeed());
                K_encoder = Math.max(pid_close.update(abs_speed, false), 0.5);
                forward = forward*K_encoder;
            }
            
	        System.out.println("forward: " + forward);
	        System.out.println("turn: " + turn);
	        
	        motorL.setSpeed(-(forward + turn));
	        motorR.setSpeed(forward - turn);

			comm.transmit();
			
			end_time = System.currentTimeMillis();
			
			try {
			    Thread.sleep(40);
			    if (100 + start_time - end_time > 0){
			        Thread.sleep(100 + start_time - end_time);
			    } else {
			        System.out.println("TIME OVERFLOW: " + (end_time - start_time));
			    }
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
		}
	}
}
