package controller;

import javax.swing.JLabel;

import Core.Engine;
import vision.Vision;

public class TestVisionThread {
    public static void main(String[] args){
        final Vision vision = new Vision(0, 320, 240, true);
        
        Thread display_thread = new Thread(new Runnable(){
            public void run(){             
                //JLabel camera_pane = createWindow("Camera output", vision.WIDTH, vision.HEIGHT);
                //JLabel colorize_pane = createWindow("Filtered output", vision.WIDTH, vision.HEIGHT);
                int ball_target_x, ball_target_y, reactor_target_x, reactor_target_y;
                double target_height, target_radius;
                while (true) {
                    vision.update();

                    //updateWindow(camera_pane, vision.curr_image);
                    //updateWindow(colorize_pane, vision.colorized);
                    
                    reactor_target_x = vision.getNextReactorX();
                    reactor_target_y = vision.getNextReactorY();
                    target_height = vision.getNextReacterHeight();
                    
                    ball_target_x = vision.getNextBallX();
                    ball_target_y = vision.getNextBallY();
                    target_radius = vision.getNextBallRadius();
                    
                    System.out.println("ball_target_x: " + ball_target_x);
                    System.out.println("ball_target_y: " + ball_target_y);
                    System.out.println("target_radius: " + target_radius);
                    
                    System.out.println("reactor_target_x: " + reactor_target_x);
                    System.out.println("reactor_target_y: " + reactor_target_y);
                    System.out.println("target_height: " + target_height);
                }
            }
        });
        display_thread.start();
    }
}
