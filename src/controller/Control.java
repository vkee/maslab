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

public class Control {
    public static void main(String[] args){      
        Control robot = new Control();
        robot.loop();
    }
    
    // MAPLE
    private MapleComm comm;
    
    // CONSTANTS
    private final int WIDTH = 320;
    private final int HEIGHT = 240;
    private final int BUFF_LENGTH = 6;
    private final int CAMERA_NUM = 1;
    private final int CHANGE_THRES = 5;
    private final double K_PROP = 0.2;
    private final double K_INT = 0.01;
    private final double K_DIFF = 0.01;
    private final boolean DISPLAY_ON = true;
    
    // VISION
    Vision vision;
    
    // STATE VALUES
    double distanceL, distanceR, distanceA, distanceB;
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
    //private ColorSensor color_sensor;
    private Cytron motorL, motorR;
    private Ultrasonic sonarL, sonarR, sonarA, sonarB;
    //private Encoder encoderL, encoderR;
    private Gyroscope gyro;
    private DigitalOutput relay;
    
    // PIDS
    PID pid_align, pid_gyro, pid_encoder, pid_target_x, pid_target_y;
    
    // BUFFERS
    private List<Double> sonar_buffL, sonar_buffR, sonar_buffA, sonar_buffB;
    private double[] sonar_buff_statsL, sonar_buff_statsR, sonar_buff_statsA, sonar_buff_statsB;
    private List<Double> encoder_buffL, encoder_buffR;
    private double[] encoder_buff_statsL, encoder_buff_statsR;
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
        vision = new Vision(CAMERA_NUM, WIDTH, HEIGHT, DISPLAY_ON);
        
        // VALUES
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
        //color_sensor = new ColorSensor(red_ball_count, green_ball_count);
        //color_sensor.start();
        
        motorL = new Cytron(4, 0);
        motorR = new Cytron(10, 1);
        
        sonarL = new Ultrasonic(36, 35);
        sonarR = new Ultrasonic(26, 25);
        sonarA = new Ultrasonic(32, 31);
        sonarB = new Ultrasonic(34, 33);
        
        //encoderL = new Encoder(5, 7);
        //encoderR = new Encoder(6, 8);
        
        gyro = new Gyroscope(1, 9);
        
        relay = new DigitalOutput(37);
        
        // REGISTER DEVICES AND INITIALIZE
        comm.registerDevice(sonarA);
        comm.registerDevice(sonarB);
        comm.registerDevice(sonarL);
        comm.registerDevice(sonarR);
        comm.registerDevice(motorL);
        comm.registerDevice(motorR);
        comm.registerDevice(gyro);
        //comm.registerDevice(encoderL);
        //comm.registerDevice(encoderR);
        comm.registerDevice(relay);
        
        System.out.println("Initializing...");
        comm.initialize();
        
        comm.updateSensorData();
        
        // PIDS
        pid_align = new PID(0.15, K_PROP, K_DIFF, K_INT);
        pid_gyro = new PID(0, K_PROP, K_DIFF, K_INT);
        pid_encoder = new PID(0, K_PROP, K_DIFF, K_INT);
        pid_target_x = new PID(WIDTH/2, K_PROP, K_DIFF, K_INT);
        pid_target_y = new PID(0, K_PROP, K_DIFF, K_INT);
        
        pid_align.update(sonarL.getDistance(), true);
        pid_gyro.update(0, true);
        pid_encoder.update(0, true);
        pid_target_x.update(WIDTH/2, true);
        pid_target_y.update(0, true);
        
        // DISTANCES
        distanceL = sonarL.getDistance();
        distanceR = sonarR.getDistance();
        distanceA = sonarA.getDistance();
        distanceB = sonarB.getDistance();
        
        // BUFFERS
        sonarBuffInit();
        
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
        
        // INITIALIZE SONARS
        relay.setValue(false);
        comm.transmit();
        
        comm.updateSensorData();
        
        // UPDATE DISTANCES
        distanceL = sonarL.getDistance();
        distanceR = sonarR.getDistance();
        distanceA = sonarA.getDistance();
        distanceB = sonarB.getDistance();
        
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
            distanceL = sonarL.getDistance();
            distanceR = sonarR.getDistance();
            distanceA = sonarA.getDistance();
            distanceB = sonarB.getDistance();
            
            sonarBuffUpdate(sonarL, sonar_buffL, sonar_buff_statsL);
            sonarBuffUpdate(sonarR, sonar_buffR, sonar_buff_statsR);
            sonarBuffUpdate(sonarA, sonar_buffA, sonar_buff_statsA);
            sonarBuffUpdate(sonarB, sonar_buffB, sonar_buff_statsB);
            
            // UPDATE STATE VALUES
            time = System.currentTimeMillis();
            angle += (time - prev_time)*gyro.getOmega();
            //encoder_diff = encoderL.getTotalAngularDistance() - encoderR.getTotalAngularDistance();
            
            // UPDATE VISION
            vision.update();
            
            // ESTIMATE STATES
            estimateControlState();
            estimateWanderState();
            estimateBallCollectState();
            
            // CALCULATE MOTOR INPUTS
            updateMotorInputs();
            
            motorL.setSpeed(-(forward + turn));
            motorR.setSpeed(-(forward - turn));

            comm.transmit();
            
            // PRINT
            //printInputs();
            System.out.println("forward: " + forward);
            System.out.println("turn: " + turn);
            
            try {
                Thread.sleep(10);
            } catch (InterruptedException exc) {
                exc.printStackTrace();
            }
        }
    }
    
    private void printInputs(){
        System.out.println("/////////////////////////////");
        double dist_var = sonar_buff_statsL[2];
        
        System.out.println("dist_var: " + dist_var);
        System.out.println("distanceL: " + sonarL.getDistance());
        System.out.println("distanceR: " + sonarR.getDistance());
        System.out.println("distanceA: " + sonarA.getDistance());
        System.out.println("distanceB: " + sonarB.getDistance());
        
        System.out.println("forward: " + forward);
        System.out.println("turn: " + turn);
        System.out.println("/////////////////////////////");
    }
    
    private void updateMotorInputs(){
        if (control_state == ControlState.BALL_COLLECT){
            if (ball_collect_state == BallCollectState.TARGETING){
                System.out.println("BALL_COLLECT: TARGETING");
                turn = Math.max(-0.02, Math.min(0.02, pid_target_x.update(target_x, false)));
                forward = 0;
            } else if (ball_collect_state == BallCollectState.APPROACHING){
                System.out.println("BALL_COLLECT: APPROACHING");
                turn = Math.max(-0.02, Math.min(0.02, pid_target_x.update(target_x, false)));
                forward = 0.06;
            } else {
                System.out.println("BALL_COLLECT: COLLECTING");
                turn = Math.max(-0.02, Math.min(0.02, pid_target_x.update(target_x, false)));
                forward = Math.max(0, Math.min(0.04, pid_target_y.update(target_y, false)));
            }
        } else {
            // ACCOUNT FOR MED_A
            
            double med_A = sonar_buff_statsA[3];
            double med_B = sonar_buff_statsB[3];
            double med_L = sonar_buff_statsL[3];
            
            if (wander_state == WanderState.DEFAULT){
                System.out.println("WANDER: DEFAULT");
                turn = Math.max(-0.02, Math.min(0.02, pid_align.update(med_L, false)));
                forward = 0.06;
            } else if (wander_state == WanderState.ALIGNED){
                System.out.println("WANDER: ALIGNED");
                turn = Math.max(-0.02, Math.min(0.02, pid_align.update(med_L, false)));
                //turn = 0.4*Math.max(-0.05, Math.min(0.05, pid_gyro.update(angle, false)));
                //turn += 0.4*Math.max(-0.05, Math.min(0.05, pid_align.update(med_L, false)));
                //turn += 0.4*Math.max(-0.05, Math.min(0.05, pid_encoder.update(encoder_diff - prev_encoder_diff, false)));
                forward = 0.06;
            } else if (wander_state == WanderState.WALL_AHEAD){
                System.out.println("WANDER: WALL_AHEAD");
                turn = 0.04;
                forward = (med_B - 0.15)/3;
            } else if (wander_state == WanderState.WALL_IMMEDIATE){
                System.out.println("WANDER: WALL_IMMEDIATE");
                turn = 0.04;
                forward = 0;
            } else {
                System.out.println("WANDER: WALL_ADJACENT");
                turn = Math.max(-0.02, Math.min(0.02, pid_align.update(med_L, false)));
                //turn = 0.4*Math.max(-0.05, Math.min(0.05, pid_gyro.update(angle, false)));
                //turn += 0.4*Math.max(-0.05, Math.min(0.05, pid_align.update(med_L, false)));
                //turn += 0.4*Math.max(-0.05, Math.min(0.05, pid_encoder.update(encoder_diff - prev_encoder_diff, false)));
                forward = 0.06;
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
            double med_L = sonar_buff_statsL[3];
            double med_A = sonar_buff_statsA[3];
            double med_B = sonar_buff_statsB[3];
            double dist_var = sonar_buff_statsL[2];
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
                    //prev_encoder_diff = encoderL.getTotalAngularDistance() - encoderR.getTotalAngularDistance();
                }
            }
        }
    }
    
    private void estimateDepositState(){
        if (control_state == ControlState.DEPOSIT){
            // FINISH ESTIMATION OF DEPOSIT STATE
        }
    }
    
    private void sonarBuffInit(){
        sonar_buffL = new LinkedList<Double>();
        sonar_buffR = new LinkedList<Double>();
        sonar_buffA = new LinkedList<Double>();
        sonar_buffB = new LinkedList<Double>();
        for (int i = 0; i < BUFF_LENGTH; i++){
            sonar_buffL.add(sonarL.getDistance());
            sonar_buffR.add(sonarR.getDistance());
            sonar_buffA.add(sonarA.getDistance());
            sonar_buffB.add(sonarB.getDistance());
        }
        
        sonar_buff_statsL = new double[4];
        sonar_buff_statsR = new double[4];
        sonar_buff_statsA = new double[4];
        sonar_buff_statsB = new double[4];
        
        sonar_buff_statsL[0] = sonarL.getDistance();
        sonar_buff_statsL[1] = sonarL.getDistance()*sonarL.getDistance();
        sonar_buff_statsL[2] = 0;
        sonar_buff_statsL[3] = sonarL.getDistance();
        
        sonar_buff_statsR[0] = sonarR.getDistance();
        sonar_buff_statsR[1] = sonarR.getDistance()*sonarR.getDistance();
        sonar_buff_statsR[2] = 0;
        sonar_buff_statsR[3] = sonarR.getDistance();
        
        sonar_buff_statsA[0] = sonarA.getDistance();
        sonar_buff_statsA[1] = sonarA.getDistance()*sonarA.getDistance();
        sonar_buff_statsA[2] = 0;
        sonar_buff_statsA[3] = sonarA.getDistance();
        
        sonar_buff_statsB[0] = sonarB.getDistance();
        sonar_buff_statsB[1] = sonarB.getDistance()*sonarB.getDistance();
        sonar_buff_statsB[2] = 0;
        sonar_buff_statsB[3] = sonarB.getDistance();
        
        // IMPLEMENT ENCODER AND GYRO BUFFERS
    }
    
    private void sonarBuffUpdate(Ultrasonic sonar, List<Double> buffer, double[] buffer_stats){
        if (buffer.size() != BUFF_LENGTH){
            System.out.println("Buffer not yet initialized");
        } else {
            double distance = sonar.getDistance();
            double dist_exp = buffer_stats[0];
            double dist_sqexp = buffer_stats[1];
            double dist_var = buffer_stats[2];
            double prev_dist = buffer.get(0);
            dist_exp = (BUFF_LENGTH*dist_exp + distance - prev_dist)/BUFF_LENGTH;
            dist_sqexp = (BUFF_LENGTH*dist_sqexp + distance*distance - prev_dist*prev_dist)/BUFF_LENGTH;
            dist_var = dist_sqexp - dist_exp*dist_exp;
            
            List<Double> sorted = new ArrayList<Double>();
            for (Double num : buffer){
                sorted.add(num);
            }
            Collections.sort(sorted);
            double median = 0.5*(sorted.get((int) BUFF_LENGTH/2 - 1) + sorted.get((int) BUFF_LENGTH/2));
            
            buffer.add(distance);
            buffer.remove(0);
            
            buffer_stats[0] = dist_exp;
            buffer_stats[1] = dist_sqexp;
            buffer_stats[2] = dist_var;
            buffer_stats[3] = median;
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
