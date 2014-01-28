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

public class TestBallCollect {

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
		
		//final Vision vision = new Vision(1, 320, 240, true);
        
//        Thread vision_thread = new Thread(new Runnable(){
//            public void run(){
//                Engine.initGL(320, 240);
//                
//                long start_time, end_time;
//                
//                while (true){
//                    start_time = System.currentTimeMillis();
//                    vision.update();
//                    end_time = System.currentTimeMillis();
//                    try {
//                        if (75 + start_time - end_time > 0){
//                            Thread.sleep(75 + start_time - end_time);
//                        }
//                    } catch (Exception exc){
//                        exc.printStackTrace();
//                    }
//                }
//            }
//        });
//		
//        vision_thread.start();
		
        PID pid_speedbc = new PID(4, 0.2, 0.08, 0.01);
        pid_speedbc.update(4, true);
        
        PID pid_ball = new PID(0, 0.5, 0.5, 0);
        pid_ball.update(0, true);
        
        PID pid_forward = new PID(240, 0.4, 0.3, 0);
        pid_forward.update(0, true);
        
        long start_time, end_time;
        double target_x, target_y, target_radius;
        double forward, turn, omega, abs_speed, K_encoder;
        turn = 0;
        
        // Load the OpenCV library
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
        Engine.initGL(320,240);
        
        // Setup the camera
        VideoCapture camera = new VideoCapture();
        camera.open(1);
        int width = 320;
        int height = 240;
        
        camera.set(Highgui.CV_CAP_PROP_FRAME_WIDTH, width);
        camera.set(Highgui.CV_CAP_PROP_FRAME_HEIGHT, height);
        
        JLabel cameraPane = TrackBall.createWindow("Camera output", width, height);
        JLabel opencvPane = TrackBall.createWindow("OpenCV output", width, height);
        
        FilterOp blur = new FilterOp("blur");
        FilterOp colorize = new FilterOp("colorize");
        FilterOp objRec = new FilterOp("objectRecognition");
       
        Mat rawImage = new Mat();
        
        // Wait until the camera has a new frame
        while (!camera.read(rawImage)) {
            try {
                Thread.sleep(1);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        
        BufferedImage bufferedImage = Mat2Image.getImage(rawImage);
        blur.apply(bufferedImage);
        colorize.apply();
        objRec.apply();
        BufferedImage filtered = FilterOp.getImage();
        Point newCenter = DetectionGL.nextCenter(filtered, width, height);
        org.opencv.core.Point point = new org.opencv.core.Point(newCenter.x, newCenter.y);
        
        
        PID pidX = new PID(width/2, 0.4, 0.3, 0);
        PID pidY = new PID(0.95*height, 0.4, 0.3, 0);
        double pidOutX = pidX.update(point.x, true);
        double pidOutY = pidY.update(point.y, true);
        
		while (true) {
		    start_time = System.currentTimeMillis();
		    
		    //vision.update();

		    comm.updateSensorData();

//		    try {
//		        target_x = vision.getNextBallX();
//		        target_y = vision.getNextBallY();
//		        target_radius = vision.getNextBallRadius();
//		        System.out.println(Math.abs(target_x - 160)/target_radius);
//                System.out.println("TARGETING BALL");
//                turn = -pid_ball.update(target_x - 160, false)/320;
//                forward = 0;
//		        if (Math.abs(target_x - 160)/target_radius > 1 && target_y < 180){
//		            System.out.println("TARGETING BALL");
//		            turn = -pid_ball.update(target_x - 160, false)/320;
//		            forward = 0;
//		        } else {
//		            System.out.println("APPROACHING BALL");
//		            turn = pid_ball.update(target_x - 160, false)/320;
//		            forward = pid_forward.update(target_y, false)/240;
//		        }
//		    } catch (Exception exc){
//		        //omega = gyro.getOmega();
//		        System.out.println("LOST TRACK OF BALL");
//		        if (turn >= 0){
//		            turn = -0.12;
//		        } else {
//		            turn = 0.12;
//		        }
//		        forward = 0;
//		    }

		    //abs_speed = Math.abs(encoderL.getAngularSpeed()) + Math.abs(encoderR.getAngularSpeed()); 
		    //K_encoder = Math.max(pid_speedbc.update(abs_speed, false), 0.5);
		    
	        //turn = K_encoder*turn;
	        //forward = K_encoder*forward;
	        
		    while (!camera.read(rawImage)) {
                try {
                    Thread.sleep(1);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
            double time = System.currentTimeMillis();
            // Process the image however you like
            //Mat processedImage = ImageProcessor.process(rawImage);
            
            bufferedImage = Mat2Image.getImage(rawImage);
            blur.apply(bufferedImage);
            colorize.apply();
            objRec.apply();
            filtered = FilterOp.getImage();
            
            newCenter = DetectionGL.nextCenter(filtered, width, height);
            System.out.println(newCenter.x);
            point = new org.opencv.core.Point(newCenter.x, newCenter.y);
            
            pidOutX = pidX.update(point.x, false);
            pidOutY = pidY.update(point.y, false);
            
            double turn1 = Math.min(0.1, -pidOutX/width);
            turn = Math.max(-0.1, turn1);
            
            double forward1 = Math.min(0.1, pidOutY/height);
            forward = Math.max(-0.1, forward1);
            
            if (point.y == 0.0 || point.y < 30) {
                forward = 0.15;
                turn = 0;
            }
            
            // Update the GUI windows
            TrackBall.updateWindow(cameraPane, rawImage);
            opencvPane.setIcon(new ImageIcon(filtered));
		    
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
