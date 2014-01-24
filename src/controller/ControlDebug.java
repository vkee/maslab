package controller;

import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedList;
import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;

import vision.ColorSensor;
import vision.Vision;
import comm.MapleComm;
import comm.MapleIO;
import devices.actuators.Cytron;
import devices.actuators.DigitalOutput;
import devices.sensors.Encoder;
import devices.sensors.Gyroscope;
import devices.sensors.Ultrasonic;

public class ControlDebug {
    public static void main(String[] args){      
        ControlDebug robot = new ControlDebug();
        robot.loop();
    }
    
    // MAPLE
    private MapleComm comm;
    
    // VISION
    Vision vision;
    
    // CONSTANTS
    private final int WIDTH = 320;
    private final int HEIGHT = 240;
    private final int BUFF_LENGTH = 6;
    private final int CAMERA_NUM = 1;
    private final int CHANGE_THRES = 2;
    private final boolean DISPLAY_ON = true;
    
    // STATE VALUES
    double distanceL, distanceR, distanceA, distanceB, distanceC;
    long start_time, end_time;
    //double angle;
    double target_x;
    double target_y;
    
    // MOTOR INPUTS
    private double forward;
    private double turn;
    
    // SENSORS AND ACTUATORS
    //private ColorSensor color_sensor;
    private Cytron motorL, motorR;
    private Ultrasonic sonarL, sonarR, sonarA, sonarB, sonarC;
    //private Encoder encoderL, encoderR;
    private Gyroscope gyro;
    private DigitalOutput relay;
    
    // PIDS
    PID pid_align, pid_target;
    //PID pid_gyro, pid_encoder, pid_target_x, pid_target_y;
    
    // STATES
    private ControlState control_state;
    private WanderState wander_state;
    private BallCollectState ball_collect_state;
    //private DepositState deposit_state;
    
    private enum ControlState { WANDER, BALL_COLLECT, DEPOSIT };
    private enum WanderState { DEFAULT, WALL_AHEAD, TURNING, ADJACENT_LEFT, ADJACENT_RIGHT };
    private enum BallCollectState { TARGETING, APPROACHING, COLLECTING };
    private enum DepositState { APPROACHING, ALIGNING, BACKING, CENTERING, DEPOSITING };
    
    public ControlDebug(){
        comm = new MapleComm(MapleIO.SerialPortType.WINDOWS);
        
        // MOTOR INPUTS
        forward = 0;
        turn = 0;
        
        // VISION
        vision = new Vision(CAMERA_NUM, WIDTH, HEIGHT, DISPLAY_ON);
        
        // SENSORS AND ACTUATORS
        //color_sensor = new ColorSensor(red_ball_count, green_ball_count);
        //color_sensor.start();
        
        motorL = new Cytron(4, 0);
        motorR = new Cytron(10, 1);
        
        sonarA = new Ultrasonic(30, 29);
        sonarB = new Ultrasonic(32, 31);
        sonarC = new Ultrasonic(34, 33);
        sonarL = new Ultrasonic(36, 35);
        sonarR = new Ultrasonic(26, 25);
        
        //encoderL = new Encoder(5, 7);
        //encoderR = new Encoder(6, 8);
        
        //gyro = new Gyroscope(1, 9);
        
        relay = new DigitalOutput(37);
        
        // REGISTER DEVICES AND INITIALIZE
        comm.registerDevice(sonarA);
        comm.registerDevice(sonarB);
        comm.registerDevice(sonarC);
        comm.registerDevice(sonarL);
        comm.registerDevice(sonarR);
        comm.registerDevice(motorL);
        comm.registerDevice(motorR);
        //comm.registerDevice(gyro);
        //comm.registerDevice(encoderL);
        //comm.registerDevice(encoderR);
        comm.registerDevice(relay);
        
        System.out.println("Initializing...");
        comm.initialize();
        
        comm.updateSensorData();
        
        // PIDS
        pid_align = new PID(0.1, 0.5, 0.08, 0.01);
        pid_target = new PID(WIDTH/2, 0.2, 0.05, 0.01);
        //pid_gyro = new PID(0, K_PROP, K_DIFF, K_INT);
        //pid_encoder = new PID(0, K_PROP, K_DIFF, K_INT);
        //pid_target_x = new PID(WIDTH/2, K_PROP, K_DIFF, K_INT);
        //pid_target_y = new PID(0, K_PROP, K_DIFF, K_INT);
        
        pid_align.update(sonarL.getDistance(), true);
        pid_target.update(0, true);
        //pid_gyro.update(0, true);
        //pid_encoder.update(0, true);
        //pid_target_x.update(WIDTH/2, true);
        //pid_target_y.update(0, true);
        
        // DISTANCES
        distanceL = sonarL.getDistance();
        distanceR = sonarR.getDistance();
        distanceA = sonarA.getDistance();
        distanceB = sonarB.getDistance();
        distanceC = sonarC.getDistance();
        
        // STATE INITIALIZATION
        control_state = ControlState.WANDER;
        wander_state = WanderState.DEFAULT;
        ball_collect_state = BallCollectState.TARGETING;
        //deposit_state = DepositState.APPROACHING;
    }
    
    private void loop(){ 
        System.out.println("Beginning to follow wall...");
        
        // INITIALIZE SONARS
        relay.setValue(false);
        comm.transmit();
        
        comm.updateSensorData();
        
        // UPDATE DISTANCES
        distanceL = sonarL.getDistance();
        distanceR = sonarR.getDistance();
        distanceA = sonarA.getDistance();
        distanceB = sonarB.getDistance();
        distanceC = sonarC.getDistance();
        
        try {
            Thread.sleep(10);
        } catch (InterruptedException exc) {
            exc.printStackTrace();
        }
        
        while (true){
            // START TIME
            start_time = System.currentTimeMillis();
            
            comm.updateSensorData();
            
            // UPDATE DISTANCES
            distanceL = sonarL.getDistance();
            distanceR = sonarR.getDistance();
            distanceA = sonarA.getDistance();
            distanceB = sonarB.getDistance();
            distanceC = sonarC.getDistance();
            
            // UPDATE VISION
            vision.update();
            
            // ESTIMATE STATES
            estimateControlState();
            estimateWanderState();
            estimateBallCollectState();
            
            // CALCULATE MOTOR INPUTS
            updateMotorInputs();
            
            // DRIVE MOTORS
            driveMotors();

            comm.transmit();
            
            //System.out.println("distanceL: " + sonarL.getDistance());
            //System.out.println("distanceR: " + sonarR.getDistance());
            //System.out.println("distanceA: " + sonarA.getDistance());
            //System.out.println("distanceB: " + sonarB.getDistance());
            //System.out.println("distanceC: " + sonarC.getDistance());
            //System.out.println("forward: " + forward);
            //System.out.println("turn: " + turn);
            
            // END TIME
            end_time = System.currentTimeMillis();
            
            try {
                Thread.sleep(100 + end_time - start_time);
            } catch (InterruptedException exc) {
                exc.printStackTrace();
            }
        }
    }
    
    private void driveMotors(){
        motorL.setSpeed(-(forward + turn));
        motorR.setSpeed(forward - turn);
    }

    private void estimateControlState(){
        try {
            target_x = vision.getNextBallX();
            target_y = vision.getNextBallY();
            control_state = ControlState.BALL_COLLECT;
        } catch (Exception exc){
            control_state = ControlState.WANDER;
        }
    }
    
    private void updateMotorInputs(){
        if (control_state == ControlState.WANDER){
            if (wander_state == WanderState.WALL_AHEAD){
                System.out.println("WALL AHEAD");
                turn = 0.12;
                forward = 0;
            } else if (wander_state == WanderState.TURNING){
                // Maybe eliminate second condition
                System.out.println("TURNING");
                turn = 0.12;
                forward = 0;
            } else if (wander_state == WanderState.ADJACENT_LEFT){
                System.out.println("ADJACENT_LEFT");
                turn = 0.05;
                forward = 0.07;
            } else if (wander_state == WanderState.ADJACENT_RIGHT){
                System.out.println("ADJACENT_RIGHT");
                turn = -0.08;
                forward = 0;
            } else {
                System.out.println("DEFAULT");
                turn = Math.max(-0.1, Math.min(0.1, pid_align.update(distanceL, false)));
                forward = 0.08;
            }
        } else {
            if (ball_collect_state == BallCollectState.TARGETING){
                System.out.println("TARGETING");
                turn = Math.max(-0.05, Math.min(0.05, pid_target.update(target_x, false)));
                forward = 0;
            } else if (ball_collect_state == BallCollectState.APPROACHING){
                System.out.println("APPROACHING");
                turn = Math.max(-0.05, Math.min(0.05, pid_target.update(target_x, false)));
                forward = 0.08;
            } else {
                System.out.println("COLLECTING");
                turn = 0;
                forward = 0.05;
            }
        }
    }

    private void estimateWanderState(){        
        if (control_state == ControlState.WANDER){
            if (distanceC < 0.2){
                wander_state = WanderState.WALL_AHEAD;
            } else if (distanceB < 0.2 || (distanceA < 0.22 && distanceB < 0.22)){
                wander_state = WanderState.TURNING;
            } else if (distanceL < 0.13){
                wander_state = WanderState.ADJACENT_LEFT;
            } else if (distanceR < 0.1){
                wander_state = WanderState.ADJACENT_RIGHT;
            } else {
                wander_state = WanderState.DEFAULT;
            }
        }
    }
    
    private void estimateBallCollectState(){
        if (control_state == ControlState.BALL_COLLECT){
            if (Math.abs(target_x - WIDTH/2) > 20 && ball_collect_state == BallCollectState.TARGETING){
                ball_collect_state = BallCollectState.TARGETING;
            } else if (target_y < 0.75*HEIGHT && ball_collect_state != BallCollectState.COLLECTING){
                ball_collect_state = BallCollectState.APPROACHING;
            } else if (ball_collect_state != BallCollectState.TARGETING){
                ball_collect_state = BallCollectState.COLLECTING;
            }
        }
    }
}
