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
    boolean target_found;
    
    // MOTOR INPUTS
    private double forward;
    private double turn;
    
    // SENSORS AND ACTUATORS
    //private ColorSensor color_sensor;
    private Cytron motorL, motorR;
    private Ultrasonic sonarL, sonarR, sonarA, sonarB, sonarC;
    //private Encoder encoderL, encoderR;
    //private Gyroscope gyro;
    private DigitalOutput relay;
    
    // PIDS
    PID pid_align, pid_target;
    //PID pid_gyro, pid_encoder, pid_target_x, pid_target_y;
    
    // STATES
    private State state;
    
    private enum ControlState { DEFAULT, WALL_AHEAD, TURNING, ADJACENT_LEFT,
        ADJACENT_RIGHT, TARGETING_BALL, APPROACHING_BALL, COLLECTING_BALL, PULL_AWAY };
    
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
        pid_align = new PID(0.15, 0.5, 0.08, 0.01);
        pid_target = new PID(WIDTH/2, 0.5, 0.3, 0);
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
        
        // VALUES
        target_found = false;
        target_x = 0;
        target_y = 0;
        
        // STATE INITIALIZATION
        state = new State(ControlState.DEFAULT);
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
        
        long point_1, point_2;
        
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
            
            // ESTIMATE STATE
            estimateState();
            
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
            
//            try {
//                Thread.sleep(20 + end_time - start_time);
//            } catch (InterruptedException exc) {
//                exc.printStackTrace();
//            }
        }
    }
    
    private void driveMotors(){
        motorL.setSpeed(-(forward + turn));
        motorR.setSpeed(forward - turn);
    }
    
    private void updateMotorInputs(){
        if (state.state == ControlState.WALL_AHEAD){
            System.out.println("WALL AHEAD");
            turn = 0.12;
            forward = 0;
        } else if (state.state == ControlState.TURNING){
            // Maybe eliminate second condition
            System.out.println("TURNING");
            turn = 0.12;
            forward = 0;
        } else if (state.state == ControlState.ADJACENT_LEFT){
            System.out.println("ADJACENT_LEFT");
            turn = 0.05;
            forward = 0.07;
        } else if (state.state == ControlState.ADJACENT_RIGHT){
            System.out.println("ADJACENT_RIGHT");
            turn = -0.08;
            forward = 0;
        } else if (state.state == ControlState.DEFAULT){
            System.out.println("DEFAULT");
            turn = Math.max(-0.1, Math.min(0.1, pid_align.update(distanceL, false)));
            forward = 0.08;
        } else if (state.state == ControlState.TARGETING_BALL){
            System.out.println("TARGETING_BALL");
            turn = -Math.max(-0.1, Math.min(0.1, pid_target.update(target_x, false)/WIDTH));
            forward = 0;
        } else if (state.state == ControlState.APPROACHING_BALL){
            System.out.println("APPROACHING_BALL");
            turn = -Math.max(-0.05, Math.min(0.05, pid_target.update(target_x, false)/WIDTH));
            forward = 0.08;
        } else if (state.state == ControlState.COLLECTING_BALL) {
            System.out.println("COLLECTING_BALL");
            turn = 0;
            forward = 0.08;
        } else if (state.state == ControlState.PULL_AWAY){
            turn = -0.1;
            forward = -0.15;
        }
    }

    private void estimateState(){      
        try {
            target_x = vision.getNextBallX();
            target_y = vision.getNextBallY();
            target_found = true;
        } catch (Exception exc){
            target_x = 0;
            target_y = 0;
            target_found = false;
        }
        
        ControlState temp_state = ControlState.DEFAULT;
        
        if (target_found){
            if (target_y > 0.8*HEIGHT && (state.state == ControlState.APPROACHING_BALL
                    || state.state == ControlState.COLLECTING_BALL)){
                temp_state = ControlState.COLLECTING_BALL;
            } else if (Math.abs(target_x - WIDTH/2) < 20 && (state.state == ControlState.TARGETING_BALL
                    || state.state == ControlState.APPROACHING_BALL)){
                temp_state = ControlState.APPROACHING_BALL;
            } else {
                temp_state = ControlState.TARGETING_BALL;
            }
        } else {
            if (distanceC < 0.2){
                temp_state = ControlState.WALL_AHEAD;
            } else if (distanceB < 0.2 || (distanceA < 0.22 && distanceB < 0.22)){
                temp_state = ControlState.TURNING;
            } else if (distanceL < 0.13){
                temp_state = ControlState.ADJACENT_LEFT;
            } else if (distanceR < 0.1){
                temp_state = ControlState.ADJACENT_RIGHT;
            } else {
                temp_state = ControlState.DEFAULT;
            }
        }
        
        if (state.getTime() > 800){
            state.changeState(temp_state);
        }
        
        if (state.getTime() > 5000){
            state.changeState(ControlState.PULL_AWAY);
        }
    }
    
    private class State {
        private long start_time;
        public ControlState state;
        
        public State(ControlState init){
            start_time = System.currentTimeMillis();
            state = init;
        }
        
        public void changeState(ControlState new_state){
            if (state != new_state){
                start_time = System.currentTimeMillis();
                state = new_state;
            }
        }
        
        public long getTime(){
            return System.currentTimeMillis() - start_time;
        }
    }
}
