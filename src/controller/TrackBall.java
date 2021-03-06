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

import comm.MapleComm;
import comm.MapleIO;
import devices.actuators.Cytron;
import Core.Engine;
import Core.FilterOp;
import vision.Detection;
import vision.DetectionGL;
import vision.Mat2Image;


public class TrackBall {

	PID pidX;
	PID pidY;
	int height;
	int width;
	MapleComm comm;
	Cytron motor1;
	Cytron motor2;
	
	public TrackBall(MapleComm comm, double proportionalC, double derivativeC,
	        double integralC, org.opencv.core.Point point, int width, int height) {
		this.comm = comm;
		motor1 = new Cytron(4, 0);
		motor2 = new Cytron(3, 1);
		comm.registerDevice(motor1);
		comm.registerDevice(motor2);
		comm.initialize();
		comm.updateSensorData();
		
	    this.height = height;
	    this.width = width;
		pidX = new PID(width/2, proportionalC, derivativeC, integralC);
		pidY = new PID(0.95*height, proportionalC, derivativeC, integralC);
		double pidOutX = pidX.update(point.x, true);
		double pidOutY = pidY.update(point.y, true);
		double turn1 = Math.min(0.1, -pidOutX/width);
		double turn = Math.max(-0.1, turn1);
		
		double forward1 = Math.min(0.1, pidOutY/height);
		double forward = Math.max(-0.1, forward1);
		//double forward = 0;
		
		if (point.x < 30) {
			forward = 0;
		}
		motor1.setSpeed(forward+turn);
		motor2.setSpeed(forward-turn);
		comm.transmit();
	}
	
	public void update(org.opencv.core.Point point) {		
		double pidOutX = pidX.update(point.x, false);
		double pidOutY = pidY.update(point.y, false);
		
		double turn1 = Math.min(0.1, -pidOutX/width);
		double turn = Math.max(-0.1, turn1);
		
		double forward1 = Math.min(0.1, pidOutY/height);
		double forward = Math.max(-0.1, forward1);
		//double forward = 0.1;
		if (point.y == 0.0) {
			forward = 0;
			turn = 0.1;
			motor1.setSpeed(-(forward+turn));
			motor2.setSpeed(forward-turn);
			comm.transmit();
		} else if (point.y < 30) {
			forward = 0.1;
			turn = 0;
			motor1.setSpeed(-(forward+turn));
			motor2.setSpeed(forward-turn);
			comm.transmit();
			try {
				Thread.sleep(1000);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			forward = 0;
			turn = 0.1;
			motor1.setSpeed(-(forward+turn));
			motor2.setSpeed(forward-turn);
			comm.transmit();
		}
		
		//if (point.x == 0.0) {
		//	turn = 0;
		//}
		
		System.out.println("for: " + forward);
		System.out.println("turn: " + turn);
		
		motor1.setSpeed(-(forward+turn));
		motor2.setSpeed(forward-turn);
		comm.transmit();
		//System.out.println(forward);
	}
	
	public static void main(String[] args) {
        MapleComm comm = new MapleComm(MapleIO.SerialPortType.WINDOWS);
		
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
        
        JLabel cameraPane = createWindow("Camera output", width, height);
        JLabel opencvPane = createWindow("OpenCV output", width, height);
        
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
        org.opencv.core.Point center1 = new org.opencv.core.Point(newCenter.x, newCenter.y);
        TrackBall track = new TrackBall(comm, 0.4, 0.3, 0, center1, width, height);
        
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
            
            bufferedImage = Mat2Image.getImage(rawImage);
            blur.apply(bufferedImage);
            colorize.apply();
            objRec.apply();
            filtered = FilterOp.getImage();
            
            newCenter = DetectionGL.nextCenter(filtered, width, height);
            System.out.println(newCenter.x);
            center1 = new org.opencv.core.Point(newCenter.x, newCenter.y);
            track.update(center1);
            
            // Update the GUI windows
            updateWindow(cameraPane, rawImage);
            opencvPane.setIcon(new ImageIcon(filtered));
            //System.out.println(Double.toString(System.currentTimeMillis()-time));
        }
	}
	
	public static JLabel createWindow(String name, int width, int height) {    
        JFrame imageFrame = new JFrame(name);
        imageFrame.setSize(width, height);
        imageFrame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        
        JLabel imagePane = new JLabel();
        imagePane.setLayout(new BorderLayout());
        imageFrame.setContentPane(imagePane);
        
        imageFrame.setVisible(true);
        return imagePane;
    }
    
    public static void updateWindow(JLabel imagePane, Mat mat) {
        int w = (int) (mat.size().width);
        int h = (int) (mat.size().height);
        if (imagePane.getWidth() != w || imagePane.getHeight() != h) {
            imagePane.setSize(w, h);
        }
        BufferedImage bufferedImage = Mat2Image.getImage(mat);
        imagePane.setIcon(new ImageIcon(bufferedImage));
    }
}
