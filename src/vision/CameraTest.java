package vision;

import java.awt.BorderLayout;
import java.awt.Point;
import java.awt.image.BufferedImage;
import java.util.concurrent.atomic.AtomicInteger;

import javax.swing.ImageIcon;
import javax.swing.JFrame;
import javax.swing.JLabel;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.highgui.Highgui;
import org.opencv.highgui.VideoCapture;

import Core.FilterOp;
import controller.Hopper;

public class CameraTest {

	public static void main(String[] args) {
        
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
		VideoCapture camera1 = new VideoCapture();
        camera1.open(3);
        VideoCapture camera2 = new VideoCapture();
        camera2.open(3);
        int width = 320;
        int height = 240;
        camera1.set(Highgui.CV_CAP_PROP_FRAME_WIDTH, width);
        camera1.set(Highgui.CV_CAP_PROP_FRAME_HEIGHT, height);
        camera2.set(Highgui.CV_CAP_PROP_FRAME_WIDTH, width);
        camera2.set(Highgui.CV_CAP_PROP_FRAME_HEIGHT, height);
        JLabel cameraPane1 = createWindow("Camera output1", width, height);
        JLabel cameraPane2 = createWindow("Camera output2", width, height);
        
     // Main loop
        Mat rawImage1 = new Mat();
        Mat rawImage2 = new Mat();
        while (true) {
            // Wait until the camera has a new frame
            while (!camera1.read(rawImage1)) {
                try {
                    Thread.sleep(1);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
            
            updateWindow(cameraPane1, rawImage1);
            
            try {
				Thread.sleep(10);
			} catch (InterruptedException e1) {
				// TODO Auto-generated catch block
				e1.printStackTrace();
			}
            
            while (!camera2.read(rawImage2)) {
                try {
                    Thread.sleep(1);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
            
            updateWindow(cameraPane2, rawImage2);
            
            try {
				Thread.sleep(10);
			} catch (InterruptedException e1) {
				// TODO Auto-generated catch block
				e1.printStackTrace();
			}
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
