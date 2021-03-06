package vision;

import java.awt.BorderLayout;
import java.awt.image.BufferedImage;

import javax.swing.ImageIcon;
import javax.swing.JFrame;
import javax.swing.JLabel;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.highgui.Highgui;
import org.opencv.highgui.VideoCapture;

import com.google.zxing.ChecksumException;
import com.google.zxing.FormatException;
import com.google.zxing.NotFoundException;
import com.google.zxing.client.j2se.BufferedImageLuminanceSource;
import com.google.zxing.common.BitMatrix;
import com.google.zxing.common.DecoderResult;
import com.google.zxing.common.DetectorResult;
import com.google.zxing.common.HybridBinarizer;
import com.google.zxing.qrcode.decoder.Decoder;
import com.google.zxing.qrcode.detector.Detector;

import Core.Engine;
import Core.FilterOp;

public class VisionPrev {
    public static void main(String[] args){
        final VisionPrev vision = new VisionPrev(1, 320, 240, false);
        Thread display_thread = new Thread(new Runnable(){
            public void run(){
                //JLabel display_pane = createWindow("Display output", vision.WIDTH, vision.HEIGHT);
                JLabel camera_pane = createWindow("Camera output", vision.WIDTH, vision.HEIGHT);
                JLabel colorize_pane = createWindow("Colorize output", vision.WIDTH, vision.HEIGHT);
                int target_x, target_y;
                while (true) {
                    vision.update();
                    //updateWindow(display_pane, vision.curr_image);
                    updateWindow(camera_pane, vision.curr_image);
                    updateWindow(colorize_pane, vision.colorized);
                    try{
                        target_x = vision.getNextBallX();
                        target_y = vision.getNextBallY();
                        //System.out.println("//////////////////////");
                        //System.out.println("target_x: " + target_x);
                        //System.out.println("target_y: " + target_y);
                    } catch (Exception exc){
                        //System.out.println("//////////////////////");
                        //System.out.println("No Ball Found");
                    }
                }
            }
        });
        display_thread.run();
    }
    
    // CONSTANTS
    public final int WIDTH, HEIGHT;
    private final boolean DISPLAY_ON;
    
    // FIELDS
    private final int camera_number;
    public BufferedImage filtered;
    public final VideoCapture camera;
    private Mat rawImage;
    public BufferedImage curr_image;
    public BufferedImage colorized;
    private Ball red_target, green_target;
    private Reactor reactor_target;
    private JLabel camera_pane, colorize_pane;
    
    // FILTERS
    private final FilterOp blur, colorize, eliminateTop, objRec;
    
    public VisionPrev(int camera_number, int width, int height, boolean display_on){
        this.WIDTH = width;
        this.HEIGHT = height;
        this.DISPLAY_ON = display_on;
        
        // LOAD LIBARIES
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
        Engine.initGL(WIDTH, HEIGHT);
        
        // SETUP CAMERA
        camera = new VideoCapture();
        this.camera_number = camera_number;
        camera.open(camera_number);
        camera.set(Highgui.CV_CAP_PROP_FRAME_WIDTH, WIDTH);
        camera.set(Highgui.CV_CAP_PROP_FRAME_HEIGHT, HEIGHT);
        rawImage = new Mat();
        while (!camera.read(rawImage)) {
            try {
                Thread.sleep(1);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        curr_image = Mat2Image.getImage(rawImage);
        
        // SETUP TARGETS
        red_target = new Ball(false);
        green_target = new Ball(false);
        reactor_target = new Reactor(false);
        
        // FILTERS
        blur = new FilterOp("blur");
        colorize = new FilterOp("colorize");
        eliminateTop = new FilterOp("eliminateTop");
        objRec = new FilterOp("objectRecognition");
        
        // FILTERED IMAGE
        while (!camera.read(rawImage)) {
            try {
                Thread.sleep(1);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        curr_image = Mat2Image.getImage(rawImage);
        
        blur.apply(curr_image);           
        colorize.apply();
        eliminateTop.apply();
        colorized = FilterOp.getImage();
        objRec.apply();
        filtered = FilterOp.getImage();
        
        // OPTIONAL DISPLAY
        if (DISPLAY_ON){
            camera_pane = createWindow("Camera output", WIDTH, HEIGHT);
            colorize_pane = createWindow("Filtered output", WIDTH, HEIGHT);
        }
    }
    
    public void update(){        
        while (!camera.read(rawImage)) {
            try {
                Thread.sleep(1);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        curr_image = Mat2Image.getImage(rawImage);
        
        blur.apply(curr_image);           
        colorize.apply();
        eliminateTop.apply();
        colorized = FilterOp.getImage();
        objRec.apply();
        filtered = FilterOp.getImage();
        
        processFilteredImage();
        
        if (DISPLAY_ON){
            updateWindow(camera_pane, curr_image);
            updateWindow(colorize_pane, colorized);
        }
    }
    
    private void processFilteredImage(){
        red_target = new Ball(false);
        green_target = new Ball(false);
        int pixel, red, green;
        double radius, height;
        for (int x = 0; x < WIDTH; x++){
            for (int y = 0; y < HEIGHT; y++){
                if (y >= HEIGHT/2){
                    pixel = filtered.getRGB(x, y);
                    red = (pixel >> 16) & 0xFF;
                    green = (pixel >> 8) & 0xFF;
                    if (red > 0 && green == 0){
                        radius = 50*red/256.0;
                        if (radius > red_target.radius){
                            red_target = new Ball(x, y, radius);
                        }
                    }
                    if (green > 0 && red == 0){
                        radius = 50*green/256.0;
                        if (radius > green_target.radius){
                            green_target = new Ball(x, y, radius);
                        }
                    }
                } else {
                    pixel = colorized.getRGB(x, y);
                    green = (pixel >> 8) & 0xFF;
                    if (green > 0){
                        height = HEIGHT*green/256.0;
                        if (height > reactor_target.height){
                            reactor_target = new Reactor(x, y, height);
                        }
                    }
                }
            }
        }
    }
    
    /**
     * Returns the decoded QR code or throws an exception if no QR code found
     * @param original
     * @return
     */
    public String detectQR(BufferedImage original) throws RuntimeException {
    	BufferedImageLuminanceSource source = new BufferedImageLuminanceSource(original);
		HybridBinarizer hb = new HybridBinarizer(source);
		
		try {
			BitMatrix image = hb.getBlackMatrix();
			Detector detector = new Detector(image);
			DetectorResult detected = detector.detect();
			Decoder decoder = new Decoder();
			DecoderResult decoded = decoder.decode(detected.getBits());
			return decoded.getText();
		} catch (NotFoundException | ChecksumException | FormatException e1) {
		    throw new RuntimeException("QR code not found");
		}
    }
    
    public int getNextBallX() throws RuntimeException {
        if (!green_target.target && !red_target.target){
            throw new RuntimeException("No ball to target");
        } else if (green_target.radius > red_target.radius){
            return green_target.x;
        } else {
            return red_target.x;
        }
    }
    
    public int getNextBallY() throws RuntimeException{
        if (!green_target.target && !red_target.target){
            throw new RuntimeException("No ball to target");
        } else if (green_target.radius > red_target.radius){
            return green_target.y;
        } else {
            return red_target.y;
        }
    }
    
    public double getNextBallRadius() throws RuntimeException{
        if (!green_target.target && !red_target.target){
            throw new RuntimeException("No ball to target");
        } else if (green_target.radius > red_target.radius){
            return green_target.radius;
        } else {
            return red_target.radius;
        }
    }
    
    public int getNextReactorX() throws RuntimeException {
        if (!reactor_target.target){
            throw new RuntimeException("No reactor to target");
        } else {
            return reactor_target.x;
        }
    }
    
    public int getNextReactorY() throws RuntimeException {
        if (!reactor_target.target){
            throw new RuntimeException("No reactor to target");
        } else {
            return reactor_target.y;
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
    
    private static void updateWindow(JLabel imagePane, BufferedImage img) {
        int w = (int) (img.getWidth());
        int h = (int) (img.getHeight());
        if (imagePane.getWidth() != w || imagePane.getHeight() != h) {
            imagePane.setSize(w, h);
        }
        imagePane.setIcon(new ImageIcon(img));
    }
    
    private class Ball {
        public boolean target;
        public int x;
        public int y;
        public double radius;
        
        public Ball(int x, int y, double radius){
            this.x = x;
            this.y = y;
            this.radius = radius;
            this.target = true;
        }
        
        public Ball(boolean target){
            this.target = target;
            this.x = 0;
            this.y = 0;
            this.radius = 0;
        }
    }
    
    private class Reactor {
        public boolean target;
        public int x;
        public int y;
        public double height;
        
        public Reactor(int x, int y, double height){
            this.target = true;
            this.x = x;
            this.y = y;
            this.height = height;
        }
        
        public Reactor(boolean target){
            this.target = false;
            this.x = 0;
            this.y = 0;
            this.height = 0;
        }
    }
}