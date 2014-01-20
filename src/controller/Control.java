package controller;

import java.util.LinkedList;
import java.util.List;

import vision.Vision;
import comm.MapleComm;
import comm.MapleIO;
import devices.actuators.Cytron;
import devices.sensors.Encoder;
import devices.sensors.Gyroscope;
import devices.sensors.Ultrasonic;

public class Control {
    public static void main(String[] args){      
        Control robot = new Control();
        robot.wallFollow();
    }
    
    // MAPLE
    private MapleComm comm;
    
    // CONSTANTS
    private final int L = 0;
    private final int R = 1;
    private final int A = 2;
    private final int B = 3;
    private final int C = 4;
    private final int WIDTH = 320;
    private final int HEIGHT = 240;
    private final int BUFF_LENGTH = 6;
    private final int CAMERA_NUM = 1;
    private final int CHANGE_THRES = 5;
    private final double K_PROP = 0.2;
    private final double K_INT = 0.01;
    private final double K_DIFF = 0.01;
    
    // VISION
    Vision vision;
    
    // STATE VALUES
    double[] distance;
    double time;
    double prev_time;
    double angle;
    double encoder_diff;
    double prev_encoder_diff;
    double target_x;
    double target_y;
    
    // MOTOR INPUTS
    private double forward;
    private double turn;
    
    // SENSORS AND ACTUATORS
    private Cytron motorL, motorR;
    private Ultrasonic[] sonar;
    private Encoder[] encoder;
    private Gyroscope gyro;
    
    // PIDS
    PID pid_align, pid_gyro, pid_encoder, pid_target_x, pid_target_y;
    
    // BUFFERS
    private List<Double>[] sonar_buff;
    private double[][] sonar_buff_stats;
    private List<Double>[] encoder_buff;
    private double[][] encoder_buff_stats;
    private List<Double> gyro_buff;
    private double[] gyro_buff_stats;
    
    // STATES
    private ControlState control_state;
    private WanderState wander_state;
    private BallCollectState ball_collect_state;
    private int control_state_count;
    private int wander_state_count;
    private int ball_collect_state_count;
    
    private enum ControlState { WANDER, BALL_COLLECT };
    private enum WanderState { DEFAULT, WALL_ADJACENT, ALIGNED, WALL_AHEAD, WALL_IMMEDIATE };
    private enum BallCollectState { TARGETING, COLLECTING };
    
    public Control(){
        comm = new MapleComm(MapleIO.SerialPortType.WINDOWS);
        
        // MOTOR INPUTS
        forward = 0;
        turn = 0;
        
        // VISION
        vision = new Vision(CAMERA_NUM, WIDTH, HEIGHT);
        
        // VALUES
        distance = new double[5];
        time = 0;
        prev_time = 0;
        angle = 0;
        encoder_diff = 0;
        prev_encoder_diff = 0;
        target_x = WIDTH + 1;
        target_y = HEIGHT + 1;
        
        // SENSORS AND ACTUATORS
        motorL = new Cytron(0, 1);
        motorR = new Cytron(2, 3);
        
        Ultrasonic sonarA = new Ultrasonic(34, 33); // Fill in with different ports
        Ultrasonic sonarB = new Ultrasonic(32, 31);
        Ultrasonic sonarC = new Ultrasonic(32, 31); // Fill in with different ports
        Ultrasonic sonarL = new Ultrasonic(36, 35);
        Ultrasonic sonarR = new Ultrasonic(5, 6); // Fill in with different ports
        sonar = new Ultrasonic[]{sonarL, sonarR, sonarA, sonarB, sonarC};
        
        Encoder encoderL = new Encoder(19, 20); // Fill in with different ports
        Encoder encoderR = new Encoder(18, 17); // Fill in with different ports
        encoder = new Encoder[]{encoderL, encoderR};
        
        gyro = new Gyroscope(1, 8); // Fill in with different ports
        
        // PIDS
        pid_align = new PID(0.15, K_PROP, K_DIFF, K_INT);
        pid_gyro = new PID(0, K_PROP, K_DIFF, K_INT);
        pid_encoder = new PID(0, K_PROP, K_DIFF, K_INT);
        pid_target_x = new PID(WIDTH/2, K_PROP, K_DIFF, K_INT);
        pid_target_y = new PID(0, K_PROP, K_DIFF, K_INT);
        
        pid_align.update(sonar[L].getDistance(), true);
        pid_gyro.update(0, true);
        pid_encoder.update(0, true);
        pid_target_x.update(WIDTH/2, true);
        pid_target_y.update(0, true);
        
        // BUFFERS
        sonar_buff = new LinkedList[5];
        sonar_buff_stats = new double[5][3];
        for (int i = 0; i < 5; i++){
            sonar_buff[i] = new LinkedList<Double>();
        }
        encoder_buff = new LinkedList[2];
        encoder_buff_stats = new double[2][2];
        for (int i = 0; i < 2; i++){
            encoder_buff[i] = new LinkedList<Double>();
        }
        gyro_buff = new LinkedList<Double>();
        gyro_buff_stats = new double[2];
        
        // REGISTER DEVICES AND INITIALIZE
        comm.registerDevice(sonarA);
        comm.registerDevice(sonarB);
        comm.registerDevice(sonarC);
        comm.registerDevice(sonarL);
        comm.registerDevice(sonarR);
        comm.registerDevice(motorL);
        comm.registerDevice(motorR);
        comm.registerDevice(gyro);
        comm.registerDevice(encoderL);
        comm.registerDevice(encoderR);
        
        System.out.println("Initializing...");
        comm.initialize();
        
        // STATE INITIALIZATION
        control_state = ControlState.WANDER;
        wander_state = WanderState.DEFAULT;
        ball_collect_state = BallCollectState.TARGETING;
        control_state_count = 0;
        wander_state_count = 0;
        ball_collect_state_count = 0;
    }
    
    /*
     * NOTES:
     * Ideally the control loop will look something like this:
     *  - Spend minimal time in DEFAULT finding a wall to align with (which is not encoder or gyro optimized)
     *  - Spend majority of time in ALIGNED moving along the wall (which is encoder and gyro optimized)
     *  - If see a ball, chance to BALL_COLLECT and after briefly DEFAULT until ALIGNED
     *  - Spend minimal time in WALL_AHEAD and ideally almost no time in WALL_IMMEDIATE (which is a time sink)
     * Need to get this pattern working and optimized as above. Possibly should integrate some vision method
     * to make the aligning process faster and minimize time spent in DEFAULT. Turning might be made faster
     * using additional sonars to making turning smoother and minimize time in WALL_IMMEDIATE.
     * 
     * We need to decide on several things:
     *  - Should we do a wander from target to target strategy while avoiding walls or should we follow along walls?
     *    Some ways to wander might be pick the farthest wall and move towards it in hopes of finding more balls
     *    along the way or to move towards a tower/ball (toward the center of a detected blob)
     *  - What measures can we take for robustness? Where should we use encoders, gyros, ultrasonics and what should
     *    be the protocols? We need to make sure the robot does not fail and also that it does not get stuck in anything
     *    glitchy like rapid state transitions. I think the best way is to have our robot complete point-tasks such as
     *    collecting a specific ball or going to a specific tower.
     */
    private void wallFollow(){ 
        System.out.println("Beginning to follow wall...");
        comm.updateSensorData();
        
        // UPDATE DISTANCES
        distance[L] = sonar[L].getDistance();
        distance[R] = sonar[R].getDistance();
        distance[A] = sonar[A].getDistance();
        distance[B] = sonar[B].getDistance();
        distance[C] = sonar[C].getDistance();
        
        // BUFFER INITIALIZATION
        sonarBuffInit(L, distance[L]);
        sonarBuffInit(R, distance[R]);
        sonarBuffInit(A, distance[A]);
        sonarBuffInit(B, distance[B]);
        sonarBuffInit(C, distance[C]);
        double dist_var, exp_L, exp_B;
        
        // STATE INITIALIZATION
        WanderState prev_wander_state = WanderState.DEFAULT;
        
        // INITIALIZE
        motorL.setSpeed(0);
        motorR.setSpeed(0);

        comm.transmit();
        
        try {
            Thread.sleep(10);
        } catch (InterruptedException exc) {
            exc.printStackTrace();
        }
        
        while (true){
            System.out.println("NEW ITERATION:");
            comm.updateSensorData();
            
            // UPDATE DISTANCES
            distance[L] = sonar[L].getDistance();
            distance[R] = sonar[R].getDistance();
            distance[A] = sonar[A].getDistance();
            distance[B] = sonar[B].getDistance();
            distance[C] = sonar[C].getDistance();
            
            sonarBuffUpdate(L, distance[L]);
            sonarBuffUpdate(R, distance[R]);
            sonarBuffUpdate(A, distance[A]);
            sonarBuffUpdate(B, distance[B]);
            sonarBuffUpdate(C, distance[C]);
            
            // UPDATE STATE VALUES
            time = System.currentTimeMillis();
            angle += (time - prev_time)*gyro.getOmega();
            encoder_diff = encoder[L].getTotalAngularDistance() - encoder[R].getTotalAngularDistance();
            
            exp_L = sonar_buff_stats[L][0];
            exp_B = sonar_buff_stats[B][0];
            dist_var = sonar_buff_stats[L][2];
            
            System.out.println("dist_var: " + dist_var);
            System.out.println("distanceB: " + distance[B]);
            System.out.println("distanceL: " + distance[L]);
            
            // UPDATE VISION
            vision.update();
            
            // ESTIMATE WANDER STATE
            prev_wander_state = wander_state;
            estimateWanderState();
            
            if (prev_wander_state != wander_state){
                wander_state_count = 0;
//                if (wander_state == WanderState.ALIGNED){
//                    angle = 0;
//                    prev_encoder_diff = encoderL.getTotalAngularDistance() - encoderR.getTotalAngularDistance();
//                }
            }
            
            wander_state_count++;
            
            // CALCULATE MOTOR INPUTS
            if (wander_state == WanderState.DEFAULT){
                turn = Math.max(-0.05, Math.min(0.05, pid_align.update(exp_L, false)));
                forward = 0.1;
                System.out.println("Default");
            } else if (wander_state == WanderState.ALIGNED){
                turn = Math.max(-0.05, Math.min(0.05, pid_align.update(exp_L, false)));
                //turn = 0.4*Math.max(-0.05, Math.min(0.05, pid_gyro.update(angle, false)));
                //turn += 0.4*Math.max(-0.05, Math.min(0.05, pid_align.update(dist_exp, false)));
                //turn += 0.4*Math.max(-0.05, Math.min(0.05, pid_encoder.update(encoder_diff - prev_encoder_diff, false)));
                forward = 0.1;
                System.out.println("Aligned");
            } else if (wander_state == WanderState.WALL_AHEAD){
                turn = 0.1;
                forward = (exp_B - 0.15)/1.5;
                System.out.println("Wall Ahead");
            } else if (wander_state == WanderState.WALL_IMMEDIATE){
                System.out.println("Wall Immediate");
                turn = 0.1;
                forward = 0;
            }

            System.out.println("forward: " + forward);
            System.out.println("turn: " + turn);
            
            motorL.setSpeed(forward + turn);
            motorR.setSpeed(forward - turn);

            comm.transmit();
            
            try {
                Thread.sleep(10);
            } catch (InterruptedException exc) {
                exc.printStackTrace();
            }
        }
    }
    
    private void updateMotorInputs(){
        
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
    
    private void estimateBallCollectState(){
        
    }
    
    private void estimateWanderState(){
        if (wander_state_count > 15){
            double exp_L = sonar_buff_stats[L][0];
            double exp_B = sonar_buff_stats[B][0];
            double dist_var = sonar_buff_stats[L][2];
            
            if (exp_B < 0.15){
                wander_state = WanderState.WALL_IMMEDIATE;
            } else if (exp_B < 0.3){
                wander_state = WanderState.WALL_AHEAD;
            } else if (dist_var < 0.005 && Math.abs(exp_L - 0.15) < 0.01){
                wander_state = WanderState.ALIGNED;
            } else {
                wander_state = WanderState.DEFAULT;
            }
        }
    }
    
    private void sonarBuffInit(int label, double distance){
        for (int i = 0; i < BUFF_LENGTH; i++){
            sonar_buff[label].add(distance);
        }
        sonar_buff_stats[label][0] = distance;
        sonar_buff_stats[label][1] = distance*distance;
        sonar_buff_stats[label][2] = 0;
    }
    
    private void sonarBuffUpdate(int label, double distance){
        if (sonar_buff[label].size() != BUFF_LENGTH){
            System.out.println("Buffer " + label + " not yet initialized");
        } else {
            double dist_exp = sonar_buff_stats[label][0];
            double dist_sqexp = sonar_buff_stats[label][1];
            double dist_var = sonar_buff_stats[label][2];
            double prev_dist = sonar_buff[label].get(0);
            dist_exp = (BUFF_LENGTH*dist_exp + distance - prev_dist)/BUFF_LENGTH;
            dist_sqexp = (BUFF_LENGTH*dist_sqexp + distance*distance - prev_dist*prev_dist)/BUFF_LENGTH;
            dist_var = dist_sqexp - dist_exp*dist_exp;
            
            sonar_buff[label].add(distance);
            sonar_buff[label].remove(0);
            
            sonar_buff_stats[label][0] = dist_exp;
            sonar_buff_stats[label][1] = dist_sqexp;
            sonar_buff_stats[label][2] = dist_var;
        }
    }
}
