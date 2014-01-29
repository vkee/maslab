package maslabGLvision;

import java.awt.BorderLayout;
import java.awt.Point;
import java.awt.image.BufferedImage;
import java.io.File;

import javax.swing.ImageIcon;
import javax.swing.JFrame;
import javax.swing.JLabel;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.highgui.Highgui;
import org.opencv.highgui.VideoCapture;

import vision.DetectionGL;
import vision.Mat2Image;

import comm.MapleComm;
import comm.MapleIO;

import controller.TrackBall;
import Core.Engine;
import Core.FilterOp;

public class ColourTesting {
    //	public static void main( String[] args ) {
    //		TestBed tester = new TestBed(600,450);
    //		
    //		Engine.initGL(600, 450);
    //		FilterOp blur = new FilterOp("blur");
    //		FilterOp op = new FilterOp("colorTesting");
    //		
    //		BufferedImage original = TestBed.loadImage( new File("images\\spectrum.png") );
    //		tester.setImage(original);
    //		
    //		try{ Thread.sleep(1000); } catch ( Exception e ) {}
    //		
    //		blur.apply(original);
    //		op.apply();
    //		BufferedImage filtered = FilterOp.getImage(); 
    //		tester.setImage(filtered);
    //	}

    public static void main(String[] args) {
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
        FilterOp colorize = new FilterOp("modColorize");
        FilterOp objRec = new FilterOp("modObjRec");
        FilterOp op = new FilterOp("tealTesting");
        FilterOp second = new FilterOp("newtesting");

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
        BufferedImage filtered = FilterOp.getImage();

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
            //op.apply();
            //objRec.apply();
            filtered = FilterOp.getImage();

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
