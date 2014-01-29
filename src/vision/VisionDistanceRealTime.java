package vision;

import java.awt.BorderLayout;
import java.awt.Point;
import java.awt.image.BufferedImage;

import javax.swing.ImageIcon;
import javax.swing.JFrame;
import javax.swing.JLabel;

import maslabGLvision.TestBed;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.highgui.Highgui;
import org.opencv.highgui.VideoCapture;

import Core.Engine;
import Core.FilterOp;

public class VisionDistanceRealTime {
	public static void main (String args[]) {
        // Load the OpenCV library
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);

        // Setup the camera
        VideoCapture camera = new VideoCapture();
        camera.open(1);
        
        // Create GUI windows to display camera output and OpenCV output
        Engine.initGL(500,500);
        int width = 320;
        int height = 240;
        camera.set(Highgui.CV_CAP_PROP_FRAME_WIDTH, width);
        camera.set(Highgui.CV_CAP_PROP_FRAME_HEIGHT, height);
        JLabel cameraPane = createWindow("Camera output", width, height);
        JLabel opencvPane = createWindow("OpenCV output", width, height);
        
        FilterOp blur = new FilterOp("blur");
		FilterOp modColorize = new FilterOp("modColorize");
		FilterOp eliminateTop = new FilterOp("eliminateTop");
		FilterOp eliminateBottom = new FilterOp("eliminateBottom");
		FilterOp modObjRec = new FilterOp("modObjRec");
		FilterOp centreRec = new FilterOp("centreRecognition");
		FilterOp edge = new FilterOp("edge");
		FilterOp wallDetection = new FilterOp("wallDetection");
		FilterOp wallFilter = new FilterOp("wallFilter");
		FilterOp wallFilterRefine = new FilterOp("wallFilterRefine");
        
        // Main loop
        Mat rawImage = new Mat();
        while (true) {
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
			modColorize.apply();
			eliminateTop.apply();
			eliminateBottom.apply();
			modObjRec.apply();
    		BufferedImage filtered = FilterOp.getImage();
            
    		int center, red, green, blue;
			int widthStrip = 0;
			for (int y = 0; y < 240; y++) {
				center = filtered.getRGB(160, y);
				red = (center >> 16) & 0xFF;
                green = (center >> 8) & 0xFF;
                blue = (center >> 0) & 0xFF;
                
                if (blue > 0) {
                	widthStrip = blue;
                	break;
                }
			}
			System.out.println(340.0/widthStrip);
    		
            // Update the GUI windows
            updateWindow(cameraPane, rawImage);
            opencvPane.setIcon(new ImageIcon(filtered));
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
