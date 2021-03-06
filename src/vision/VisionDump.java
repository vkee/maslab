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

public class VisionDump {
    public static void main(String[] args){
        final VisionDump vision = new VisionDump(1, 320, 240, false);
        JLabel camera_pane = createWindow("Camera output", vision.WIDTH, vision.HEIGHT);
        JLabel colorize_pane = createWindow("Filtered output", vision.WIDTH, vision.HEIGHT);
        int ball_target_x, ball_target_y, reactor_target_x, reactor_target_y;
        double target_distance, target_radius, wall_distance, distance_left, distance_right;
        double left_ratio, right_ratio;
        while (true) {
            vision.update();

            updateWindow(camera_pane, vision.curr_image);
            updateWindow(colorize_pane, vision.colorized);
            
            reactor_target_x = vision.getNextReactorX();
            reactor_target_y = vision.getNextReactorY();
            target_distance = vision.getNextReactorDistance();
            
            ball_target_x = vision.getNextBallX();
            ball_target_y = vision.getNextBallY();
            target_radius = vision.getNextBallRadius();
            
            wall_distance = vision.getWallDistance();
            distance_left = vision.getLeftmostWallDistance();
            distance_right = vision.getRightmostWallDistance();
            
            left_ratio = 1000*(target_distance - distance_left)/reactor_target_x;
            right_ratio = 1000*(target_distance - distance_right)/(320 - reactor_target_x);
            
//            System.out.println("ball_target_x: " + ball_target_x);
//            System.out.println("ball_target_y: " + ball_target_y);
//            System.out.println("target_radius: " + target_radius);
//            
            //System.out.println("reactor_target_x: " + reactor_target_x);
            //System.out.println("reactor_target_y: " + reactor_target_y);
            //System.out.println("target_distance: " + target_distance);
            //System.out.println("left_ratio: " + left_ratio);
            
//            System.out.println("wall_distance: " + wall_distance);
            System.out.println("DistanceLeft: " + distance_left);
//            System.out.println("DistanceRight: " + distanceRight);
        }
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
    private int red_count, green_count;
    private Reactor reactor_target, reactor_left, reactor_right;
    private Reactor yellow_wall;
    private JLabel camera_pane, colorize_pane;
    
    // FILTERS
    private final FilterOp blur, colorize, eliminateTop, eliminateBottom, objRec;
    
    public VisionDump(int camera_number, int width, int height, boolean display_on){
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
        reactor_left = new Reactor(false);
        reactor_right = new Reactor(false);
        yellow_wall = new Reactor(false);
        
        red_count = 0;
        green_count = 0;
        
        // FILTERS
        blur = new FilterOp("blur");
        colorize = new FilterOp("modColorize");
        eliminateTop = new FilterOp("eliminateTop");
        eliminateBottom = new FilterOp("eliminateBottom");
        objRec = new FilterOp("modObjRec");
        
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
        eliminateBottom.apply();
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
        eliminateBottom.apply();
        colorized = FilterOp.getImage();
        objRec.apply();
        filtered = FilterOp.getImage();
        
        processFilteredImage();
        
        if (green_target.radius > 0){
            System.out.println("GREEN TARGET");
        }
        
        if (red_target.radius > 0){
            System.out.println("RED TARGET");
        }
        
        if (DISPLAY_ON){
            updateWindow(camera_pane, curr_image);
            updateWindow(colorize_pane, colorized);
        }
    }
    
    private void processFilteredImage(){
        Ball temp_red_target = new Ball(false);
        Ball temp_green_target = new Ball(false);
        reactor_target = new Reactor(false);
        int pixel, red, green, blue;
        double radius, height;
        for (int x = 0; x < WIDTH; x++){
            for (int y = 0; y < HEIGHT; y++){
                if (y >= 0.52*HEIGHT){
                    pixel = filtered.getRGB(x, y);
                    red = (pixel >> 16) & 0xFF;
                    green = (pixel >> 8) & 0xFF;
                    blue = (pixel) & 0xFF;
                    if (red > 0 && green == 0 && blue == 0){
                        radius = 50*red/256.0;
                        if (radius > temp_red_target.radius){
                            temp_red_target = new Ball(x, y, radius);
                        }
                    }
                    if (green > 0 && red == 0 && blue == 0){
                        radius = 50*green/256.0;
                        if (radius > temp_green_target.radius){
                            temp_green_target = new Ball(x, y, radius);
                        }
                    }
                } else {
                    pixel = filtered.getRGB(x, y);
                    red = (pixel >> 16) & 0xFF;
                    green = (pixel >> 8) & 0xFF;
                    blue = (pixel) & 0xFF;
                    if (green > 0 && blue > 0 && red == 0){
                        height = HEIGHT*green/256.0;
                        if (height > reactor_target.height && height >= 10){
                            reactor_target = new Reactor(x, y, height);
                        }
                    }
                    
                    
                    if (red > 120 && green > 120 && blue > 120
                            && red < 130 && green < 130 && blue < 130
                            && (reactor_left.x >= x || reactor_left.x == 0)){
                        reactor_left = new Reactor(x, y, 1);
                    }
                    
                    if (red > 185 && green > 185 && blue > 185
                            && red < 195 && green < 195 && blue < 195
                            && (reactor_right.x <= x || reactor_right.x == 0)){
                        reactor_right = new Reactor(x, y, 1);
                    }
                    
                    if (blue == 0 && red > 0 && green > 0){
                    	height = HEIGHT*green/256.0;
                    	yellow_wall = new Reactor(x, y, height);
                    }
                }
            }
        }
        
        if (temp_red_target.radius == 0 && red_count < 5){
            red_count++;
        } else {
            red_count = 0;
            red_target = temp_red_target;
        }
        
        if (temp_green_target.radius == 0 && green_count < 5){
            green_count++;
        } else {
            green_count = 0;
            green_target = temp_green_target;
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
    
    public int getNextYellowX(){
    	return yellow_wall.x;
    }
    
    public int getNextYellowY(){
    	return yellow_wall.y;
    }
    
    public double getNextYellowDistance(){
    	return (340.0/yellow_wall.height)*2.54/100;
    }
    
    public int getNextRedX(){
    	return red_target.x;
    }
    
    public int getNextRedY(){
    	return red_target.y;
    }
    
    public int getNextGreenX(){
    	return green_target.x;
    }
    
    public int getNextGreenY(){
    	return green_target.y;
    }
    
    public double getNextRedRadius(){
    	return red_target.radius;
    }
    
    public double getNextGreenRadius(){
    	return green_target.radius;
    }
    
    public int getNextBallX(){
        if (green_target.radius >= red_target.radius){
            return green_target.x;
        } else {
            return red_target.x;
        }
    }
    
    public int getNextBallY(){
        if (green_target.radius >= red_target.radius){
            return green_target.y;
        } else {
            return red_target.y;
        }
    }
    
    public double getNextBallRadius(){
        if (green_target.radius >= red_target.radius){
            return green_target.radius;
        } else {
            return red_target.radius;
        }
    }

    public int getNextBallColor(){
        if (green_target.radius >= red_target.radius){
            return 0;
        } else {
            return 1;
        }
    }
    
    public int getNextReactorX() throws RuntimeException {
        return reactor_target.x;
    }

    public int getNextReactorY() throws RuntimeException {
        return reactor_target.y;
    }

    public double getNextReactorHeight(){
        return reactor_target.height;
    }
    
    public double getNextReactorDistance(){
        return (340.0/reactor_target.height)*2.54/100;
    }
    
    public double getWallDistance(){
		int center, blue;
		int widthStrip = 0;
		for (int y = 0; y < 120; y++) {
			center = filtered.getRGB(160, y);
            blue = (center >> 0) & 0xFF;
            
            if (blue > 0) {
            	widthStrip = blue;
            	//break;
            }
		}
		return (340.0/widthStrip)*2.54/100;
    }
    
    public double getLeftmostWallDistance(){
		int center, blue;
		int widthStrip = 0;
		for (int y = 0; y < 120; y++) {
			center = filtered.getRGB(0, y);
            blue = (center >> 0) & 0xFF;
            
            if (blue > 0) {
            	widthStrip = blue;
            	//break;
            }
		}
		return (340.0/widthStrip)*2.54/100;
    }
    
    public double getLeftWallDistance(){
		int center, blue;
		int widthStrip = 0;
		for (int y = 0; y < 120; y++) {
			center = filtered.getRGB(120, y);
            blue = (center >> 0) & 0xFF;
            
            if (blue > 0) {
            	widthStrip = blue;
            	//break;
            }
		}
		return (340.0/widthStrip)*2.54/100;
    }
    
    public double getRightmostWallDistance(){
		int center, blue;
		int widthStrip = 0;
		for (int y = 0; y < 120; y++) {
			center = filtered.getRGB(319, y);
            blue = (center >> 0) & 0xFF;
            
            if (blue > 0) {
            	widthStrip = blue;
            	//break;
            }
		}
		return (340.0/widthStrip)*2.54/100;
    }
    
    public double getRightWallDistance(){
		int center, blue;
		int widthStrip = 0;
		for (int y = 0; y < 120; y++) {
			center = filtered.getRGB(200, y);
            blue = (center >> 0) & 0xFF;
            
            if (blue > 0) {
            	widthStrip = blue;
            	//break;
            }
		}
		return (340.0/widthStrip)*2.54/100;
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