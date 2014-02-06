package vision;

import java.awt.Point;
import java.awt.image.BufferedImage;
import java.io.File;
import java.util.concurrent.atomic.AtomicInteger;

import javax.swing.ImageIcon;
import javax.swing.JLabel;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.highgui.Highgui;
import org.opencv.highgui.VideoCapture;

import competition.Hopper;

import Core.Engine;
import Core.FilterOp;
import maslabGLvision.TestBed;

public class Test {

	public static void main(String[] args) {

		AtomicInteger red_ball_count = new AtomicInteger(0);
		AtomicInteger green_ball_count = new AtomicInteger(0);

		JLabel camera_pane = ColorSensor.createWindow("Camera output", 32, 24);
		Hopper hopper = new Hopper(null,24,27,28, 14, null);
		ColorSensor color_sensor = new ColorSensor(red_ball_count, green_ball_count, hopper);
		color_sensor.start();

		Mat rawImage = new Mat();
		BufferedImage curr_image;
		
//		// Load the OpenCV library
//		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
//
//		// Setup the camera
//		VideoCapture camera = new VideoCapture();
//		camera.open(1);
//
//		// Create GUI windows to display camera output and OpenCV output
//		Engine.initGL(320,240);
//		int width = 320;
//		int height = 240;
//		camera.set(Highgui.CV_CAP_PROP_FRAME_WIDTH, width);
//		camera.set(Highgui.CV_CAP_PROP_FRAME_HEIGHT, height);
//		JLabel cameraPane = VisionTestGL.createWindow("Camera output", width, height);
//		JLabel opencvPane = VisionTestGL.createWindow("OpenCV output", width, height);
//
//		FilterOp blur = new FilterOp("blur");
//		FilterOp colorize = new FilterOp("colorize");
//		FilterOp objRec = new FilterOp("objectRecognition");
//
//		// Main loop
//		Mat rawImage1 = new Mat();

		
		while (true){
			System.out.println("Red Balls: " + red_ball_count.get());
			System.out.println("Green balls: " + green_ball_count.get());

			while (!color_sensor.camera.read(rawImage)) {
				try {
					Thread.sleep(1);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
			}

			curr_image = Mat2Image.getImage(rawImage);
			ColorSensor.updateWindow(camera_pane, curr_image);
			try{
				Thread.sleep(10);
			} catch (Exception exc){
				exc.printStackTrace();
			}
			
//			// Wait until the camera has a new frame
//			while (!camera.read(rawImage1)) {
//				try {
//					Thread.sleep(1);
//				} catch (InterruptedException e) {
//					e.printStackTrace();
//				}
//			}
//
//			BufferedImage bufferedImage = Mat2Image.getImage(rawImage1);
//			blur.apply(bufferedImage);
//			colorize.apply();
//			//objRec.apply();
//			BufferedImage filtered = FilterOp.getImage();
//			Point center = DetectionGL.nextCenter(filtered, width, height);
//			System.out.println(center.x);
//			System.out.println(center.y);
//
//			// Update the GUI windows
//			VisionTestGL.updateWindow(cameraPane, rawImage1);
//			opencvPane.setIcon(new ImageIcon(filtered));
			
		}
	}
}
