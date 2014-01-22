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

public class Vision {
    public static void main(String[] args){
        final Vision vision = new Vision(1, 320, 240);
        Thread display_thread = new Thread(new Runnable(){
            public void run(){
                JLabel display_pane = createWindow("Display output", vision.WIDTH, vision.HEIGHT);
                JLabel camera_pane = createWindow("Camera output", vision.WIDTH, vision.HEIGHT);
                JLabel colorize_pane = createWindow("Colorize output", vision.WIDTH, vision.HEIGHT);
                JLabel map_pane = createWindow("Map output", vision.WIDTH, vision.HEIGHT);
                int target_x, target_y;
                while (true) {
                    vision.update();
                    //updateWindow(display_pane, vision.curr_image);
                    updateWindow(display_pane, vision.filtered);
                    updateWindow(camera_pane, vision.curr_image);
                    updateWindow(colorize_pane, vision.colorized);
                    updateWindow(map_pane, vision.map.field);
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
    
    // FIELDS
    private final int camera_number;
    public BufferedImage filtered;
    public final VideoCapture camera;
    private Mat rawImage;
    public BufferedImage curr_image;
    public BufferedImage colorized;
    private Ball red_target, green_target;
    public Map map;
    
    // FILTERS
    private final FilterOp blur, colorize, objRec;
    
    public Vision(int camera_number, int width, int height){
        this.WIDTH = width;
        this.HEIGHT = height;
        map = new Map(width, height);
        
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
        
        // FILTERS
        blur = new FilterOp("blur");
        colorize = new FilterOp("colorize");
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
        colorized = FilterOp.getImage();
        objRec.apply();
        filtered = FilterOp.getImage();
    }
    
    public void update(){
        //REMOVE THIS IN THE FINAL VERSION
        map.resetField();
        
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
        colorized = FilterOp.getImage();
        objRec.apply();
        filtered = FilterOp.getImage();
        
        processFilteredImage();
    }
    
    private void processFilteredImage(){
        map.calibrateMapCalculations(filtered);
        red_target = new Ball(false);
        green_target = new Ball(false);
        int pixel, red, green, blue;
        double radius;
        for (int x = 0; x < filtered.getWidth(); x++){
            for (int y = 0; y < filtered.getHeight(); y++){
                pixel = filtered.getRGB(x, y);
                red = (pixel >> 16) & 0xFF;
                green = (pixel >> 8) & 0xFF;
                blue = (pixel) & 0xFF;
                if (red > 0){
                    radius = 50*red/256.0;
                    if (radius > red_target.radius){
                        red_target = new Ball(x, y, radius);
                    }
                }
                if (green > 0){
                    radius = 50*green/256.0;
                    if (radius > green_target.radius){
                        green_target = new Ball(x, y, radius);
                    }
                }
                if (blue > 0 && y > 0 && ((filtered.getRGB(x, y-1)) & 0xFF) == 0){
                    try{
                        map.convertWallCoordinates(x, y, HEIGHT*blue/256.0);
                    } catch (Exception exc){
                        //exc.printStackTrace();
                    }
                }
            }
        }
    }
    
    /**
     * Returns the decoded QR code or "QR code not found if no QR code found"
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
    
    public int getNextBallY(){
        if (!green_target.target && !red_target.target){
            throw new RuntimeException("No ball to target");
        } else if (green_target.radius > red_target.radius){
            return green_target.y;
        } else {
            return red_target.y;
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
            this.target = false;
            this.x = 0;
            this.y = 0;
            this.radius = 0;
        }
    }
}