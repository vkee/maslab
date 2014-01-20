package vision;

import java.awt.BorderLayout;
import java.awt.Point;
import java.awt.image.BufferedImage;
import java.util.ArrayList;
import java.util.List;

import javax.swing.ImageIcon;
import javax.swing.JFrame;
import javax.swing.JLabel;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.highgui.Highgui;
import org.opencv.highgui.VideoCapture;

import Core.Engine;
import Core.FilterOp;

public class Vision {
    public static void main(String[] args){
        final Vision vision = new Vision(1, 320, 240);
        Thread display_thread = new Thread(new Runnable(){
            public void run(){
                JLabel display_pane = createWindow("Display output", vision.WIDTH, vision.HEIGHT);
                int target_x, target_y;
                while (true) {
                    vision.update();
                    //updateWindow(display_pane, vision.curr_image);
                    updateWindow(display_pane, vision.filtered);
                    try{
                        target_x = vision.getNextBallX();
                        target_y = vision.getNextBallY();
                        System.out.println("//////////////////////");
                        System.out.println("target_x: " + target_x);
                        System.out.println("target_y: " + target_y);
                        System.out.println("//////////////////////");
                    } catch (Exception exc){
                        System.out.println("//////////////////////");
                        System.out.println("No Ball Found");
                        System.out.println("//////////////////////");
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
    private Ball red_target, green_target;
    
    // FILTERS
    private final FilterOp blur, colorize, objRec;
    
    public Vision(int camera_number, int width, int height){
        this.WIDTH = width;
        this.HEIGHT = height;
        
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
        objRec.apply();
        filtered = FilterOp.getImage();
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
        objRec.apply();
        filtered = FilterOp.getImage();
        
        processFilteredImage();
    }
    
    private void processFilteredImage(){
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
            }
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