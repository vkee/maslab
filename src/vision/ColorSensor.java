package vision;

import java.awt.BorderLayout;
import java.awt.image.BufferedImage;
import java.util.concurrent.atomic.AtomicInteger;

import javax.swing.ImageIcon;
import javax.swing.JFrame;
import javax.swing.JLabel;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.highgui.Highgui;
import org.opencv.highgui.VideoCapture;

import Core.Engine;
import Core.FilterOp;

public class ColorSensor {
    public static void main(String[] args){
        AtomicInteger red_ball_count = new AtomicInteger(0);
        AtomicInteger green_ball_count = new AtomicInteger(0);

        JLabel camera_pane = createWindow("Camera output", 32, 24);
        
        ColorSensor color_sensor = new ColorSensor(red_ball_count, green_ball_count);
        color_sensor.start();
        
        Mat rawImage = new Mat();
        BufferedImage curr_image;
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
            updateWindow(camera_pane, curr_image);
            try{
                Thread.sleep(10);
            } catch (Exception exc){
                exc.printStackTrace();
            }
        }
    }

    private final int CAMERA_NUM = 1;

    public final VideoCapture camera;
    private Thread color_sensor_thread;

    public ColorSensor(final AtomicInteger red_ball_count, final AtomicInteger green_ball_count){
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);

        camera = new VideoCapture();
        camera.open(CAMERA_NUM);
        camera.set(Highgui.CV_CAP_PROP_FRAME_WIDTH, 32);
        camera.set(Highgui.CV_CAP_PROP_FRAME_HEIGHT, 24);
        
        color_sensor_thread = new Thread(new Runnable(){
            public void run(){
                Engine.initGL(32, 24);
                FilterOp color_filter = new FilterOp("ColorSensor");
                Mat rawImage = new Mat();
                BufferedImage curr_image, filtered;
                int delay = 0;
                int center, red, green;
                while (true){
                    delay++;
                    while (!camera.read(rawImage)) {
                        try {
                            Thread.sleep(1);
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }
                    }
                    curr_image = Mat2Image.getImage(rawImage);
                    color_filter.apply(curr_image);
                    filtered = FilterOp.getImage();
                    
                    center = filtered.getRGB(16, 12);
                    red = (center >> 16) & 0xFF;
                    green = (center >> 8) & 0xFF;
                    
                    if (delay >= 50 && red > 0){
                        red_ball_count.incrementAndGet();
                        delay = 0;
                    }
                    
                    if (delay > 50 && green > 0){
                        green_ball_count.incrementAndGet();
                        delay = 0;
                    }
                    
                    try{
                        Thread.sleep(10);
                    } catch (Exception exc){
                        exc.printStackTrace();
                    }
                }
            }
        });
    }

    public void start(){
        color_sensor_thread.start();
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
}
