package controller;

import java.awt.Point;
import java.awt.image.BufferedImage;
import java.util.LinkedList;
import java.util.List;

import javax.swing.ImageIcon;
import javax.swing.JLabel;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.highgui.Highgui;
import org.opencv.highgui.VideoCapture;

import Core.Engine;
import Core.FilterOp;
import vision.DetectionGL;
import vision.Mat2Image;
import vision.Vision;
import comm.MapleComm;
import comm.MapleIO;
import devices.actuators.Cytron;
import devices.actuators.DigitalOutput;
import devices.actuators.PWMOutput;
import devices.sensors.Encoder;
import devices.sensors.Gyroscope;
import devices.sensors.Ultrasonic;

public class TestReactor {
    private enum State { FINDING, FIRST_ALIGN, REVERSING, APPROACH, SECOND_ALIGN };

    public static void main(String[] args) {
        MapleComm comm = new MapleComm(MapleIO.SerialPortType.WINDOWS);
        Cytron motorL = new Cytron(4, 0);
        Cytron motorR = new Cytron(3, 1);
        Ultrasonic sonarA = new Ultrasonic(26, 25);
        Ultrasonic sonarB = new Ultrasonic(34, 33);
        Ultrasonic sonarC = new Ultrasonic(35, 36);
        Ultrasonic sonarD = new Ultrasonic(30, 29);
        Ultrasonic sonarE = new Ultrasonic(32, 31);
        
        Encoder encoderL = new Encoder(5, 7);
        Encoder encoderR = new Encoder(6, 8);

        //Gyroscope gyro = new Gyroscope(1, 9);

        DigitalOutput relay = new DigitalOutput(37);

        comm.registerDevice(motorL);
        comm.registerDevice(motorR);
        comm.registerDevice(sonarA);
        comm.registerDevice(sonarB);
        comm.registerDevice(sonarC);
        comm.registerDevice(sonarD);
        comm.registerDevice(sonarE);
        //comm.registerDevice(gyro);
        comm.registerDevice(relay);

        comm.registerDevice(encoderL);
        comm.registerDevice(encoderR);

        comm.initialize();

        relay.setValue(false);
        comm.transmit();

        comm.updateSensorData();

        double distanceD = sonarD.getDistance();
        double distanceE = sonarE.getDistance();
        double distanceA = sonarA.getDistance();
        double distanceB = sonarB.getDistance();
        double distanceC = sonarC.getDistance();

        final Vision vision = new Vision(1, 320, 240, true);

        PID pid_speedbc = new PID(4, 0.2, 0.08, 0.01);
        pid_speedbc.update(4, true);

        PID pid_ball = new PID(0, 0.5, 0.5, 0);
        pid_ball.update(0, true);

        PID pid_forward = new PID(240, 0.4, 0.3, 0);
        pid_forward.update(0, true);

        long start_time, end_time;
        double target_x, target_height;
        double forward, turn;
        turn = 0;

        int width = 320;
        int height = 240;

        vision.update();

        PID pidX = new PID(width/2+20, 0.3, 2, 0);
        double pidOutX = pidX.update(vision.getNextReactorX(), true);

        PID pid_align = new PID(0, 0.3, -0.2, 0);
        pid_align.update(0, true);

        PID pid_return = new PID(0.15, 0.3, -0.2, 0);
        pid_return.update(0.15, true);

        double update_value_x;

        State state = State.FINDING;

        while (true) {
            start_time = System.currentTimeMillis();

            comm.updateSensorData();

            distanceD = sonarD.getDistance();
            distanceE = sonarE.getDistance();
            distanceA = sonarA.getDistance();
            distanceB = sonarB.getDistance();
            distanceC = sonarC.getDistance();

            vision.update();

            forward = 0;
            
            target_x = vision.getNextReactorX();

            if (state == State.FINDING){
                System.out.println("FINDING");
                pidOutX = pidX.update(target_x, false);

                turn = Math.max(-0.2, Math.min(0.2, -pidOutX/width));
                forward = 0.13;

                if (Math.min(distanceD, distanceE) < 0.2){
                    state = State.FIRST_ALIGN;
                }
            } else if (state == State.FIRST_ALIGN){
                System.out.println("FIRST_ALIGN");
                turn = Math.max(-0.2, Math.min(-10*pid_align.update(distanceD - distanceE, false), 0.2));
                forward = 0;

                if (Math.abs(distanceD - distanceE) < 0.03){
                    state = State.REVERSING;
                }
            } else if (state == State.REVERSING){
                System.out.println("REVERSING");
                pidOutX = pidX.update(target_x, false);

                turn = Math.max(-0.2, Math.min(0.2, -pidOutX/width));
                forward = -0.13;

                if (Math.min(distanceD, distanceE) > 0.6){
                    state = State.APPROACH;
                }
            } else if (state == State.APPROACH){
                System.out.println("APPROACH");
                pidOutX = pidX.update(target_x, false);

                turn = Math.max(-0.2, Math.min(0.2, -pidOutX/width));
                forward = 0.13;

                if (Math.min(distanceD, distanceE) < 0.15){
                    state = State.SECOND_ALIGN;
                }
            } else if (state == State.SECOND_ALIGN){
                System.out.println("SECOND_ALIGN");
                turn = Math.max(-0.2, Math.min(-10*pid_align.update(distanceD - distanceE, false), 0.2));
                forward = 0;
            }

            System.out.println("forward: " + forward);
            System.out.println("turn: " + turn);

            motorL.setSpeed(-(forward + turn));
            motorR.setSpeed(forward - turn);

            comm.transmit();

            end_time = System.currentTimeMillis();

            try {
                Thread.sleep(40);
                if (100 + start_time - end_time > 0){
                    Thread.sleep(100 + start_time - end_time);
                } else {
                    System.out.println("TIME OVERFLOW: " + (end_time - start_time));
                }
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}
