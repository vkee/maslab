package controller;


import java.awt.Point;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.highgui.VideoCapture;

import vision.Detection;


public class TrackBall {

	PID pid;
	KitBotModel model;
	
	public TrackBall(KitBotModel model, double proportionalC, double derivativeC, double integralC, org.opencv.core.Point point) {
		pid = new PID(640/2, proportionalC, derivativeC, integralC);
		double pidOut = pid.update(point.x, true);
		double turn = Math.min(0.2, pidOut/640);
		model.setMotors(-turn, turn);
		this.model = model;
	}
	
	public void update(org.opencv.core.Point point) {		
		double pidOut = pid.update(point.x, false);
		double turn1 = Math.min(0.2, pidOut/640);
		double turn = Math.min(-0.20, turn1);
		model.setMotors(-turn, turn);
		System.out.println(pidOut);
	}
	
	public static void main(String[] args) {
		KitBotModel model = new KitBotModel();
		
		// Load the OpenCV library
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);

        // Setup the camera
        VideoCapture camera = new VideoCapture();
        camera.open(1);
		
		Mat rawImage = new Mat();
		
		// Wait until the camera has a new frame
        while (!camera.read(rawImage)) {
            try {
                Thread.sleep(1);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
		
        Mat binary = Detection.detectHueRange(rawImage);
        org.opencv.core.Point center = Detection.nextCenter(binary, 320, 240, 25);
        TrackBall track = new TrackBall(model,2,0,0,center);
        
        while (true) {
            
            // Process the image however you like
            //Mat processedImage = ImageProcessor.process(rawImage);
            Mat binary1 = Detection.detectHueRange(rawImage);
            org.opencv.core.Point center1 = Detection.nextCenter(binary1, 320, 240, 25);
            track.update(center1);
            
        }
	}
}
