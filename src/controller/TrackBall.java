package controller;


import java.awt.BorderLayout;
import java.awt.Point;
import java.awt.image.BufferedImage;

import javax.swing.ImageIcon;
import javax.swing.JLabel;
import javax.swing.JFrame;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.highgui.Highgui;
import org.opencv.highgui.VideoCapture;

import vision.Detection;
import vision.Mat2Image;


public class TrackBall {

	PID pidX;
	PID pidY;
	int height;
	int width;
	KitBotModel model;
	
	public TrackBall(KitBotModel model, double proportionalC, double derivativeC,
	        double integralC, org.opencv.core.Point point, int width, int height) {
	    this.height = height;
	    this.width = width;
		pidX = new PID(width/2, proportionalC, derivativeC, integralC);
		pidY = new PID(0.95*height, proportionalC, derivativeC, integralC);
		double pidOutX = pidX.update(point.x, true);
		double pidOutY = pidY.update(point.y, true);
		double turn1 = Math.min(0.1, pidOutX/width);
		double turn = Math.max(-0.1, turn1);
		
		double forward1 = Math.min(0.1, pidOutY/height);
		double forward = Math.max(-0.1, forward1);
		
		if (point.x < 30) {
			forward = 0;
		}
		
		model.setMotors(forward+turn, forward-turn);
		this.model = model;
	}
	
	public void update(org.opencv.core.Point point) {		
		double pidOutX = pidX.update(point.x, false);
		double pidOutY = pidY.update(point.x, false);
		
		double turn1 = Math.min(0.2, 2*pidOutX/width);
		double turn = Math.max(-0.2, turn1);
		
		double forward1 = Math.min(0.1, pidOutY/height);
		double forward = Math.max(-0.1, forward1);
		
		if (point.x < 30) {
			forward = 0;
		}
		
		if (point.y == 0.0) {
			turn = 0;
		}
		//System.out.println(forward);
		//System.out.println(turn);
		
		model.setMotors(forward+turn, forward-turn);
		
		//System.out.println(forward);
	}
	
	public static void main(String[] args) {
		KitBotModel model = new KitBotModel();
		
		// Load the OpenCV library
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);

        // Setup the camera
        VideoCapture camera = new VideoCapture();
        camera.open(1);
        int width = 320;
        int height = 240;
        
        camera.set(Highgui.CV_CAP_PROP_FRAME_WIDTH, width);
        camera.set(Highgui.CV_CAP_PROP_FRAME_HEIGHT, height);
        
        JLabel cameraPane = createWindow("Camera output", width, height);
        JLabel opencvPane = createWindow("OpenCV output", width, height);
        
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
        org.opencv.core.Point center = Detection.nextCenter(binary, width/2, height/2, 5);
        TrackBall track = new TrackBall(model, 0.3, 0.5, 0, center, width, height);
        
        while (true) {
        	
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
            Mat binary1 = Detection.detectHueRange(rawImage);
            org.opencv.core.Point center1 = Detection.nextCenter(binary1, width/2, height/2, 5);
            track.update(center1);
            
            // Update the GUI windows
            Mat processedImage = Detection.convertC(binary1);
            updateWindow(cameraPane, rawImage);
            updateWindow(opencvPane, processedImage);
            System.out.println(Double.toString(System.currentTimeMillis()-time));
        }
	}
	
	private static JLabel createWindow(String name, int width, int height) {    
        JFrame imageFrame = new JFrame(name);
        imageFrame.setSize(width, height);
        imageFrame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        
        JLabel imagePane = new JLabel();
        imagePane.setLayout(new BorderLayout());
        imageFrame.setContentPane(imagePane);
        
        imageFrame.setVisible(true);
        return imagePane;
    }
    
    private static void updateWindow(JLabel imagePane, Mat mat) {
        int w = (int) (mat.size().width);
        int h = (int) (mat.size().height);
        if (imagePane.getWidth() != w || imagePane.getHeight() != h) {
            imagePane.setSize(w, h);
        }
        BufferedImage bufferedImage = Mat2Image.getImage(mat);
        imagePane.setIcon(new ImageIcon(bufferedImage));
    }
}
