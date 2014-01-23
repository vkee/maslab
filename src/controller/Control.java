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
import devices.sensors.Encoder;
import devices.sensors.Gyroscope;
import devices.sensors.Ultrasonic;

public class Control {
    public static void main(String[] args){      
        Control robot = new Control();
        robot.loop();
    }
    
    // MAPLE
    private MapleComm comm;
    
    // CONSTANTS
    private final int L = 0;
    private final int R = 1;
    private final int A = 2;
    private final int B = 3;
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
    final AtomicInteger green_ball_count;
    final AtomicInteger red_ball_count;
    
    // MOTOR INPUTS
    private double forward;
    private double turn;
    
    // SENSORS AND ACTUATORS
    private ColorSensor color_sensor;
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
    private DepositState deposit_state;
    private int control_state_count;
    private int wander_state_count;
    private int ball_collect_state_count;
    private int deposit_state_count;
    
    private enum ControlState { WANDER, BALL_COLLECT, DEPOSIT };
    private enum WanderState { DEFAULT, WALL_ADJACENT, ALIGNED, WALL_AHEAD, WALL_IMMEDIATE };
    private enum BallCollectState { TARGETING, APPROACHING, COLLECTING };
    private enum DepositState { APPROACHING, ALIGNING, BACKING, CENTERING, DEPOSITING };
    
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
        green_ball_count = new AtomicInteger(0);
        red_ball_count = new AtomicInteger(0);
        
        // SENSORS AND ACTUATORS
        color_sensor = new ColorSensor(red_ball_count, green_ball_count);
        color_sensor.start();
        
        motorL = new Cytron(4, 0);
        motorR = new Cytron(10, 1);
        
        Ultrasonic sonarA = new Ultrasonic(32, 31); // Fill in with different ports
        Ultrasonic sonarB = new Ultrasonic(34, 33);
        Ultrasonic sonarL = new Ultrasonic(36, 35);
        Ultrasonic sonarR = new Ultrasonic(26, 25); // Fill in with different ports
        sonar = new Ultrasonic[]{sonarL, sonarR, sonarA, sonarB};
        
        Encoder encoderL = new Encoder(5, 7); // Fill in with different ports
        Encoder encoderR = new Encoder(6, 8); // Fill in with different ports
        encoder = new Encoder[]{encoderL, encoderR};
        
        gyro = new Gyroscope(1, 9); // Fill in with different ports
        
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
        sonar_buff = new LinkedList[4];
        sonar_buff_stats = new double[4][4];
        for (int i = 0; i < 4; i++){
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
        deposit_state = DepositState.APPROACHING;
        control_state_count = 0;
        wander_state_count = 0;
        ball_collect_state_count = 0;
        deposit_state_count = 0;
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
    private void loop(){ 
        System.out.println("Beginning to follow wall...");
        comm.updateSensorData();
        
        // UPDATE DISTANCES
        distance[L] = sonar[L].getDistance();
        distance[R] = sonar[R].getDistance();
        distance[A] = sonar[A].getDistance();
        distance[B] = sonar[B].getDistance();
        
        // BUFFER INITIALIZATION
        sonarBuffInit(L, distance[L]);
        sonarBuffInit(R, distance[R]);
        sonarBuffInit(A, distance[A]);
        sonarBuffInit(B, distance[B]);
        
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
            comm.updateSensorData();
            
            // UPDATE DISTANCES
            distance[L] = sonar[L].getDistance();
            distance[R] = sonar[R].getDistance();
            distance[A] = sonar[A].getDistance();
            distance[B] = sonar[B].getDistance();
            
            sonarBuffUpdate(L, distance[L]);
            sonarBuffUpdate(R, distance[R]);
            sonarBuffUpdate(A, distance[A]);
            sonarBuffUpdate(B, distance[B]);
            
            // UPDATE STATE VALUES
            time = System.currentTimeMillis();
            angle += (time - prev_time)*gyro.getOmega();
            encoder_diff = encoder[L].getTotalAngularDistance() - encoder[R].getTotalAngularDistance();
            
            // UPDATE VISION
            vision.update();
            
            // ESTIMATE STATES
            estimateControlState();
            estimateWanderState();
            estimateBallCollectState();
            
            // CALCULATE MOTOR INPUTS
            updateMotorInputs();
            
            motorL.setSpeed(forward + turn);
            motorR.setSpeed(forward - turn);

            comm.transmit();
            
            // PRINT
            printInputs();
            
            try {
                Thread.sleep(10);
            } catch (InterruptedException exc) {
                exc.printStackTrace();
            }
        }
    }
    
    private void printInputs(){
        System.out.println("/////////////////////////////");
        double dist_var = sonar_buff_stats[L][2];
        
        System.out.println("dist_var: " + dist_var);
        System.out.println("distanceB: " + distance[L]);
        System.out.println("distanceL: " + distance[R]);
        System.out.println("distanceB: " + distance[A]);
        System.out.println("distanceL: " + distance[B]);
        
        System.out.println("forward: " + forward);
        System.out.println("turn: " + turn);
        System.out.println("/////////////////////////////");
    }
    
    private void updateMotorInputs(){
        if (control_state == ControlState.BALL_COLLECT){
            if (ball_collect_state == BallCollectState.TARGETING){
                System.out.println("BALL_COLLECT: TARGETING");
                turn = Math.max(-0.05, Math.min(0.05, pid_target_x.update(target_x, false)));
                forward = 0;
            } else if (ball_collect_state == BallCollectState.APPROACHING){
                System.out.println("BALL_COLLECT: APPROACHING");
                turn = Math.max(-0.05, Math.min(0.05, pid_target_x.update(target_x, false)));
                forward = 0.1;
            } else {
                System.out.println("BALL_COLLECT: COLLECTING");
                turn = Math.max(-0.05, Math.min(0.05, pid_target_x.update(target_x, false)));
                forward = Math.max(0, Math.min(0.05, pid_target_y.update(target_y, false)));
            }
        } else {
            // ACCOUNT FOR MED_A
            
            double med_A = sonar_buff_stats[A][3];
            double med_B = sonar_buff_stats[B][3];
            double med_L = sonar_buff_stats[L][3];
            
            if (wander_state == WanderState.DEFAULT){
                turn = Math.max(-0.05, Math.min(0.05, pid_align.update(med_L, false)));
                forward = 0.1;
            } else if (wander_state == WanderState.ALIGNED){
                turn = Math.max(-0.05, Math.min(0.05, pid_align.update(med_L, false)));
                //turn = 0.4*Math.max(-0.05, Math.min(0.05, pid_gyro.update(angle, false)));
                //turn += 0.4*Math.max(-0.05, Math.min(0.05, pid_align.update(med_L, false)));
                //turn += 0.4*Math.max(-0.05, Math.min(0.05, pid_encoder.update(encoder_diff - prev_encoder_diff, false)));
                forward = 0.1;
            } else if (wander_state == WanderState.WALL_AHEAD){
                turn = 0.1;
                forward = (med_B - 0.15)/1.5;
            } else if (wander_state == WanderState.WALL_IMMEDIATE){
                turn = 0.1;
                forward = 0;
            } else {
                turn = Math.max(-0.05, Math.min(0.05, pid_align.update(med_L, false)));
                //turn = 0.4*Math.max(-0.05, Math.min(0.05, pid_gyro.update(angle, false)));
                //turn += 0.4*Math.max(-0.05, Math.min(0.05, pid_align.update(med_L, false)));
                //turn += 0.4*Math.max(-0.05, Math.min(0.05, pid_encoder.update(encoder_diff - prev_encoder_diff, false)));
                forward = 0.1;
            }
        }
    }
    
    private void estimateControlState(){
        // INCLUDE CODE TO DEAL WITH DEPOSIT STATE
        
        ControlState temp_state = ControlState.WANDER;
        
        try {
            target_x = vision.getNextBallX();
            target_y = vision.getNextBallY();
            temp_state = ControlState.BALL_COLLECT;
        } catch (Exception exc){
            temp_state = ControlState.WANDER;
        }
        
        if (temp_state != control_state){
            control_state_count++;
        } else {
            control_state_count = Math.max(0, control_state_count-1);
        }
        
        if (control_state_count >= CHANGE_THRES){
            control_state = temp_state;
            control_state_count = 0;
        }
    }
    
    private void estimateBallCollectState(){
        if (control_state == ControlState.BALL_COLLECT){
            BallCollectState temp_state = BallCollectState.TARGETING;
            
            if (Math.abs(target_x - WIDTH/2) < 10){
                temp_state = BallCollectState.COLLECTING;
            } else {
                temp_state = BallCollectState.TARGETING;
            }
            
            if (temp_state != ball_collect_state){
                ball_collect_state_count++;
            } else {
                ball_collect_state_count = Math.max(0, ball_collect_state_count-1);
            }
            
            if (ball_collect_state_count >= CHANGE_THRES){
                ball_collect_state = temp_state;
                ball_collect_state_count = 0;
            }
        }
    }
    
    /*
     * Add cleaner logic that is right-left symmetric (other
     * than what is necessary for left wall-following)
     */
    private void estimateWanderState(){        
        if (control_state == ControlState.WANDER){
            // ACCOUNT FOR MED_A
            double med_L = sonar_buff_stats[L][3];
            double med_A = sonar_buff_stats[A][3];
            double med_B = sonar_buff_stats[B][3];
            double dist_var = sonar_buff_stats[L][2];
            WanderState temp_state = WanderState.DEFAULT;

            if (med_B < 0.15){
                temp_state = WanderState.WALL_IMMEDIATE;
            } else if (med_B < 0.3){
                temp_state = WanderState.WALL_AHEAD;
            } else if (dist_var < 0.005 && Math.abs(med_L - 0.15) < 0.01){
                temp_state = WanderState.ALIGNED;
            } else if (med_L < 0.3){
                temp_state = WanderState.WALL_ADJACENT;
            } else {
                temp_state = WanderState.DEFAULT;
            }

            if (temp_state != wander_state){
                wander_state_count++;
            } else {
                wander_state_count = Math.max(0, wander_state_count-1);
            }

            if (wander_state_count >= CHANGE_THRES){
                wander_state = temp_state;
                wander_state_count = 0;
                if (wander_state == WanderState.ALIGNED){
                    angle = 0;
                    prev_encoder_diff = encoder[L].getTotalAngularDistance() - encoder[R].getTotalAngularDistance();
                }
            }
        }
    }
    
    private void estimateDepositState(){
        if (control_state == ControlState.DEPOSIT){
            // FINISH ESTIMATION OF DEPOSIT STATE
        }
    }
    
    private void sonarBuffInit(int label, double distance){
        for (int i = 0; i < BUFF_LENGTH; i++){
            sonar_buff[label].add(distance);
        }
        sonar_buff_stats[label][0] = distance;
        sonar_buff_stats[label][1] = distance*distance;
        sonar_buff_stats[label][2] = 0;
        sonar_buff_stats[label][3] = distance;
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
            
            List<Double> sorted = new ArrayList<Double>();
            Collections.copy(sonar_buff[label], sorted);
            Collections.sort(sorted);
            double median = 0.5*(sorted.get((int) BUFF_LENGTH/2) + sorted.get((int) BUFF_LENGTH/2 + 1));
            
            sonar_buff[label].add(distance);
            sonar_buff[label].remove(0);
            
            sonar_buff_stats[label][0] = dist_exp;
            sonar_buff_stats[label][1] = dist_sqexp;
            sonar_buff_stats[label][2] = dist_var;
            sonar_buff_stats[label][3] = median;
        }
    }
    
    private void lowerRamp(double angle){
        // Servo lowers the ramp to the specified angle
    }
    
    private void releaseGreenBall(){
        // Servo releases a green ball and resets after a delay
    }
    
    private void releaseRedBall(){
        //Servo releases all red balls and resets after a delay
    }
}
