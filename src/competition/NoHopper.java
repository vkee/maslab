package competition;

import java.util.LinkedList;
import java.util.List;

import BotClient.BotClient;
import Core.Engine;
import vision.Vision;
import comm.MapleComm;
import comm.MapleIO;
import competition.Hopper;
import controller.PID;
import devices.actuators.Cytron;
import devices.actuators.DigitalOutput;
import devices.actuators.PWMOutput;
import devices.sensors.DigitalInput;
import devices.sensors.Encoder;
import devices.sensors.Ultrasonic;

public class NoHopper {
    public static void main(String[] args){   
        final Vision vision = new Vision(1, 320, 240, true);
        final double[] vision_vals = new double[10];
        for (int i = 0; i < 10; i++){
        	vision_vals[i] = 0;
        }
        Thread run_thread = new Thread(new Runnable(){
            public void run(){
                NoHopper robot = new NoHopper(vision_vals);
                robot.loop();
            }
        });
        run_thread.start();
        long start_time, end_time;
        
        while (true){
            start_time = System.currentTimeMillis();
            vision.update();
            synchronized (vision_vals){
            	vision_vals[0] = vision.getNextBallX();
                vision_vals[1] = vision.getNextBallY();
                vision_vals[2] = vision.getNextBallRadius();
                vision_vals[3] = vision.getWallDistance();
                vision_vals[4] = vision.getLeftmostWallDistance();
                vision_vals[5] = vision.getNextBallColor();
                vision_vals[6] = vision.getNextReactorX();
                vision_vals[7] = vision.getNextReactorDistance();
                vision_vals[8] = vision.getRightmostWallDistance();
                vision_vals[9] = vision.getLeftWallDistance();
            }
            end_time = System.currentTimeMillis();
            try {
                if (60 + start_time - end_time > 0){
                    Thread.sleep(60 + start_time - end_time);
                } else {
                    //System.out.println("TIME OVERFLOW: " + (end_time - start_time));
                }
            } catch (Exception exc){
                System.out.println("TIME OVERFLOW: " + (end_time - start_time));
            }
        }
    }
    
    // BOT CLIENT
    //BotClient botclient;
    
    // VISION
    double[] vision_vals;
    
    // MAPLE
    private final MapleComm comm;
    
    // CONSTANTS
    private final int WIDTH = 320;
    private final int HEIGHT = 240;
    private final int BUFF_LENGTH = 15;
    private final int CAMERA_NUM = 1;
    private final int CHANGE_THRES = 2;
    private final boolean DISPLAY_ON = true;
    
    // STATE VALUES
    double distanceD, distanceE, distanceA, distanceB, distanceC;
    double cam_dist, left_dist, right_dist, left_dist_close;
    long start_time, end_time;
    int target_x, target_y;
    double target_radius;
    double K_encoder;
    double orient_time;
    long intake_time;
    long reset_time;
    boolean encoder_flag;
    long encoder_flag_time;
    boolean validA, validB, validC, validD, validE;
    long ball_absent_time;
    int prev_ball_color;
    int ball_color;
    int num_red_balls;
    int num_green_balls;
    int reactor_x;
    double reactor_dist;
    int num_balls;
    
    final List<Integer> ball_colors;
    Thread ball_sort_thread;
    
    List<Double> left_ratios, right_ratios;
    
    // BUFFERS
    List<Double> buffD, buffE, buffA, buffB, buffC;
    List<Double> buff_encoder;
    
    // MOTOR INPUTS
    private double forward;
    private double turn;
    
    // SENSORS AND ACTUATORS
    private Cytron motorL, motorR;
    private Ultrasonic sonarD, sonarE, sonarA, sonarB, sonarC;
    private Encoder encoderL, encoderR;
    private DigitalOutput power_sonars;
    private Cytron ball_intake;
    private final Hopper hopper;
    
    // PIDS
    PID pid_dist, pid_speedwf;
    PID pid_target, pid_approach;
    PID pid_reactor_align, pid_reactor_dist;
    
    // STATES
    private State state;
    
    private enum ControlState { DEFAULT, WALL_AHEAD, FOLLOW, PULL_AWAY, LEFT_FAR, FORWARD, RANDOM_ORIENT, APPROACH,
        COLLECT, REACTOR_FAR_LEFT, REACTOR_FAR_RIGHT, REACTOR_APPROACH, REACTOR_IMMEDIATE, REACTOR_ALIGNED };
    
    public NoHopper(double[] vision_vals){
		//botclient = new BotClient("18.150.7.174:6667","b3MpHHs4J1",false);
        comm = new MapleComm(MapleIO.SerialPortType.WINDOWS);
        
        // VISION
        this.vision_vals = vision_vals;
        
        // MOTOR INPUTS
        forward = 0;
        turn = 0;
        
        // SENSORS AND ACTUATORS        
        motorL = new Cytron(4, 0);
        motorR = new Cytron(3, 1);
        
        sonarA = new Ultrasonic(26, 25);
        sonarB = new Ultrasonic(34, 33);
        sonarC = new Ultrasonic(35, 36);
        sonarD = new Ultrasonic(30, 29);
        sonarE = new Ultrasonic(32, 31);
        
        encoderL = new Encoder(5, 7);
        encoderR = new Encoder(6, 8);
        power_sonars = new DigitalOutput(37);
        ball_intake = new Cytron(23, 2);
        
        // REGISTER DEVICES AND INITIALIZE
        comm.registerDevice(sonarD);
        comm.registerDevice(sonarE);
        comm.registerDevice(sonarA);
        comm.registerDevice(sonarB);
        comm.registerDevice(sonarC);
        
        comm.registerDevice(motorL);
        comm.registerDevice(motorR);
        comm.registerDevice(ball_intake);
        
        comm.registerDevice(encoderL);
        comm.registerDevice(encoderR);
        
        comm.registerDevice(power_sonars);
        
        ball_colors = new LinkedList<Integer>();
        hopper = new Hopper(comm, 24, 27, 28, 14, ball_colors);
        
        System.out.println("Initializing...");
        comm.initialize();
        
        comm.updateSensorData();
        
        // VALUES
        distanceD = sonarD.getDistance();
        distanceE = sonarE.getDistance();
        distanceA = sonarA.getDistance();
        distanceB = sonarB.getDistance();
        distanceC = sonarC.getDistance();
        
        cam_dist = 0;
        left_dist = 0;
        right_dist = 0;
        left_dist_close = 0;
        orient_time = 1500 + 1000*Math.random();
        intake_time = 0;
        reset_time = System.currentTimeMillis();
        
        validA = true;
        validB = true;
        validC = true;
        validD = true;
        validE = true;
        
        ball_absent_time = 0;
        prev_ball_color = 0;
        ball_color = 0;
        
        num_red_balls = 0;
        num_green_balls = 0;
        reactor_x = 0;
        reactor_dist = 0;
        
        num_balls = 0;
        
        left_ratios = new LinkedList<Double>();
        right_ratios = new LinkedList<Double>();
        
        for (int i = 0; i < 10; i++){
            left_ratios.add(5.0);
            right_ratios.add(5.0);
        }
        
        // PIDS
        pid_dist = new PID(0.2, 0.3, 100, 0); // PID for wall following turn on distance  
        pid_dist.update(Math.min(distanceA, distanceB), true);
        
        pid_speedwf = new PID(11, 0.2, -0.08, 0.01);
        pid_speedwf.update(11, true);
        
        pid_target = new PID(WIDTH/2, 0.2, 2, 0); // PID for ball targeting turn on displacement from center
        pid_target.update(WIDTH/2, true);
        
        pid_approach = new PID(WIDTH/2, 0.2, -2, 0);
        pid_approach.update(WIDTH/2, true);
        
        pid_reactor_align = new PID(WIDTH/2, 0.3, 0, 0);
        pid_reactor_dist = new PID(0.05, 1.5, -0.2, 0);
        pid_reactor_align.update(0, true);
        pid_reactor_dist.update(0.05, true);
        
        // BUFFERS
        buffD = new LinkedList<Double>();
        buffE = new LinkedList<Double>();
        buffA = new LinkedList<Double>();
        buffB = new LinkedList<Double>();
        buffC = new LinkedList<Double>();
        buff_encoder = new LinkedList<Double>();
        
        for (int i = 0; i < BUFF_LENGTH; i++){
            buffD.add(Math.random()*distanceD);
            buffE.add(Math.random()*distanceE);
            buffA.add(Math.random()*distanceA);
            buffB.add(Math.random()*distanceB);
            buffC.add(Math.random()*distanceC);
            buff_encoder.add(Math.random()*2*Math.PI);
        }    
        
        encoder_flag = false;
        encoder_flag_time = 0;
        
        K_encoder = 1;
        
        // STATE INITIALIZATION
        state = new State(ControlState.DEFAULT);
    }
    
    private void loop(){
//		while( !botclient.gameStarted() ) {}
//    	
		state.changeState(ControlState.FOLLOW);
		
		reset_time = System.currentTimeMillis();
		
        System.out.println("Beginning to follow wall...");
        
        // INITIALIZE SONARS
        power_sonars.setValue(false);
        comm.transmit();
        
        comm.updateSensorData();
        
        // UPDATE VISION
        synchronized (vision_vals){
            target_x = (int) vision_vals[0];
            target_y = (int) vision_vals[1];
            target_radius = vision_vals[2];
            cam_dist = vision_vals[3];
            left_dist = vision_vals[4];
            ball_color = (int) vision_vals[5];
            reactor_x = (int) vision_vals[6];
            reactor_dist = vision_vals[7];
            right_dist = vision_vals[8];
            left_dist_close = vision_vals[9];
        }
        
        // UPDATE DISTANCES
        distanceD = sonarD.getDistance();
        distanceE = sonarE.getDistance();
        distanceA = sonarA.getDistance();
        distanceB = sonarB.getDistance();
        distanceC = sonarC.getDistance();
        
        try {
            Thread.sleep(10);
        } catch (InterruptedException exc) {
            exc.printStackTrace();
        }
        
        //while (botclient.gameStarted()){
        while (true){
        	System.out.println("updating");
        	synchronized (comm){
                comm.updateSensorData();
        	}
            
            start_time = System.currentTimeMillis();
            
            prev_ball_color = ball_color;
            
            // UPDATE VISION
            synchronized (vision_vals){
                target_x = (int) vision_vals[0];
                target_y = (int) vision_vals[1];
                target_radius = vision_vals[2];
                cam_dist = vision_vals[3];
                left_dist = vision_vals[4];
                ball_color = (int) vision_vals[5];
                reactor_x = (int) vision_vals[6];
                reactor_dist = vision_vals[7];
                right_dist = vision_vals[8];
                left_dist_close = vision_vals[9];
            }
            
            // UPDATE DISTANCES
            distanceD = sonarD.getDistance();
            distanceE = sonarE.getDistance();
            distanceA = sonarA.getDistance();
            distanceB = sonarB.getDistance();
            distanceC = sonarC.getDistance();
            
            // UPDATE BUFFERS
            updateSonarBuffers(false);
            updateEncoderFlag();
            
            // ESTIMATE STATE
            estimateState();
            
            // UPDATE MOTORS
            updateMotors();
            
            // UPDATE HOPPER
            //updateHopper();
            
            print();
            synchronized (comm){
                comm.transmit();
            }
            
            end_time = System.currentTimeMillis();
            
            try {
                if (40 - end_time + start_time >= 0){
                    Thread.sleep(40 - end_time + start_time);
                } else {
                    System.out.println("TIME OVERFLOW: " + (end_time - start_time));
                }
            } catch (Exception exc){
                exc.printStackTrace();
            }
        }
        
       // botclient.close();
    }
    
    private void updateMotors(){
        double temp_turn = 0;
        double temp_forward = 0.1;
        double update_value_x;
        
        if (state.state == ControlState.REACTOR_FAR_LEFT){
            System.out.println("REACTOR_FAR_LEFT");
            if (state.getTime() < 2000){
                turn = 0.15;
                forward = 0;
            } else if (state.getTime() < 4000){
                turn = 0;
                forward = 0.15;
            } else {
                turn = -0.15;
                forward = 0;
            }
        } else if (state.state == ControlState.REACTOR_FAR_RIGHT){
            System.out.println("REACTOR_FAR_RIGHT");
            if (state.getTime() < 2000){
                turn = -0.15;
                forward = 0;
            } else if (state.getTime() < 4000){
                turn = 0;
                forward = 0.15;
            } else {
                turn = 0.15;
                forward = 0;
            }
        } else if (state.state == ControlState.REACTOR_APPROACH){
            System.out.println("REACTOR_APPROACH");
            double align = pid_reactor_align.update(reactor_x, false)/WIDTH;
            turn = Math.min(0.2, Math.max(-0.2, -align));
            forward = 0.13;
        } else if (state.state == ControlState.REACTOR_IMMEDIATE){
            System.out.println("REACTOR_IMMEDIATE");
            //double align = pid_align.update(vision.getNextReactorX(), false)/width;
            //turn = Math.min(0.2, Math.max(-0.2, -align));
            turn = 0;
            forward = getTurnStateEstimate()*1.5;
        } else if (state.state == ControlState.REACTOR_ALIGNED){
//            System.out.println("REACTOR_ALIGNED");
//            double align = pid_reactor_align.update(reactor_x, false)/WIDTH;
//            turn = Math.min(0.2, Math.max(-0.2, -align));
//            forward = 0;
            if (state.getTime() < 500){
                turn = 0;
                forward = 0.2;
            } else if (state.getTime() < 1000){
                turn = 0;
                forward = 0;
            } else if (state.getTime() < 3000){
                turn = 0;
                forward = 0;
                hopper.sorterGreen();
                try {
                    Thread.sleep(2000);
                } catch (Exception exc) {
                    exc.printStackTrace();
                }
            } else if (state.getTime() < 5000){
                hopper.rampHigh();
                comm.transmit();
                hopper.pacmanOpen();
                try {
                    Thread.sleep(2000);
                } catch (Exception exc) {
                    exc.printStackTrace();
                }
            } else if (state.getTime() < 7000){
                motorL.setSpeed(0.15);
                motorR.setSpeed(-0.15);
                comm.transmit();
                try {
                    Thread.sleep(2000);
                } catch (Exception exc) {
                    exc.printStackTrace();
                }
            } else if (state.getTime() < 9000){
                hopper.rampLow();
                motorL.setSpeed(0);
                motorR.setSpeed(0);
                comm.transmit();
                hopper.pacmanClose();
                
                hopper.pacmanOpen();
                try {
                    Thread.sleep(2000);
                } catch (Exception exc) {
                    exc.printStackTrace();
                }
                hopper.pacmanClose();
                hopper.sorterBlocking();
            }
        } else if (state.state == ControlState.APPROACH){
            System.out.println("BALL_COLLECT: APPROACH");
            
            if (target_radius > 0){
                update_value_x = WIDTH/2 + ((target_x - WIDTH/2)*12/target_radius);
            } else {
                update_value_x = target_x;
            }
            
            turn = Math.max(-0.25, Math.min(0.25, -pid_target.update(target_x, false)/WIDTH));
            forward = 0.17;
        } else if (state.state == ControlState.COLLECT){
            System.out.println("BALL_COLLECT: COLLECT");
            turn = 0;
            forward = 0.13;
        } else {
            if (state.state == ControlState.WALL_AHEAD){
                System.out.println("WALL_FOLLOW: WALL AHEAD");
                temp_turn = 0.12;
                temp_forward = 0;
            } else if (state.state == ControlState.FOLLOW){
                System.out.println("WALL_FOLLOW: FOLLOW");
                temp_turn = pid_dist.update(getAlignStateEstimate(), false);
            } else if (state.state == ControlState.PULL_AWAY){
                System.out.println("WALL_FOLLOW: PULL_AWAY");
                temp_turn = 0;
                temp_forward = -0.15;
            } else if (state.state == ControlState.RANDOM_ORIENT){
                System.out.println("WALL_FOLLOW: RANDOM_ORIENT");
                temp_turn = -0.15;
                temp_forward = 0;
            } else if (state.state == ControlState.DEFAULT){
                System.out.println("WALL_FOLLOW: DEFAULT");
                temp_turn = -0.05;
                temp_forward = 0.1;
//            } else if (state.state == ControlState.LEFT_FAR){
//                System.out.println("WALL_FOLLOW: LEFT_FAR");
//                temp_turn = -0.15;
//                temp_forward = 0;
//            } else if (state.state == ControlState.FORWARD){
//                System.out.println("WALL_FOLLOW: FORWARD");
//                temp_turn = 0;
//                temp_forward = 0.1;
            }

            double abs_speed = Math.abs(encoderL.getAngularSpeed()) + Math.abs(encoderR.getAngularSpeed()); 
            K_encoder = Math.max(pid_speedwf.update(abs_speed, false), 0.5);
            
            turn = K_encoder*temp_turn;
            forward = K_encoder*temp_forward;
            //turn = 1.6*temp_turn;
            //forward = 1.8*temp_forward;
        }
        
        motorL.setSpeed(-(forward + turn));
        motorR.setSpeed(forward - turn);
    }

    private void estimateState(){
        ControlState temp_state = state.state;
        ControlState prev_state = state.state;
        
        double left_ratio, right_ratio;
        boolean left_far, right_far;
        
        if (reactor_x != 0){
            left_ratio = (reactor_dist - left_dist)/reactor_x;
            right_ratio = (reactor_dist - right_dist)/(WIDTH - reactor_x);
        } else {
            left_ratio = 1;
            right_ratio = 1;
        }
        
        left_ratios.add(left_ratio);
        left_ratios.remove(0);
        left_far = true;
        for (Double val : left_ratios){
            if (val < 1.3){
                left_far = false;
            }
        }
        
        right_ratios.add(right_ratio);
        right_ratios.remove(0);
        right_far = true;
        for (Double val : right_ratios){
            if (val < 1.3){
                right_far = false;
            }
        }
        
        if (target_radius == 0 && ball_absent_time == 0){
        	ball_absent_time = System.currentTimeMillis();
        } else if (target_radius != 0){
        	ball_absent_time = 0;
        }
        
        if (intake_time > 0 && System.currentTimeMillis() > intake_time + 12000){
            intake_time = 0;
            ball_intake.setSpeed(0);
        }
        
        if (System.currentTimeMillis() > 30000 && reactor_x != 0){
            if (left_dist < right_dist && left_far){
                state.changeState(ControlState.REACTOR_FAR_LEFT);
            } else if (right_dist < left_dist && right_far){
                state.changeState(ControlState.REACTOR_FAR_RIGHT);
            } else if (reactor_x != 0 && getTurnStateEstimate() < 0.05){
                state.changeState(ControlState.REACTOR_ALIGNED);
            } else if (reactor_x != 0 && getTurnStateEstimate() < 0.1){
                state.changeState(ControlState.REACTOR_IMMEDIATE);
            } else if (reactor_x != 0) {
                state.changeState(ControlState.REACTOR_APPROACH);
            }
        } else if (state.state == ControlState.APPROACH && !(getTurnStateEstimate() < 0.1
        		|| getAlignStateEstimate() < 0.1) && !(ball_absent_time > 0
        				&& System.currentTimeMillis() - ball_absent_time > 300)){
            if (target_y > 180){
                temp_state = ControlState.COLLECT;
            } else {
                temp_state = ControlState.APPROACH;
            }
        } else if ((state.state == ControlState.COLLECT && state.getTime() >= 1000)
                || state.state != ControlState.COLLECT || (getTurnStateEstimate() < 0.1
                		|| getAlignStateEstimate() < 0.1)){
            if (getTurnStateEstimate() < 0.25){
                temp_state = ControlState.WALL_AHEAD;
            } else if (getAlignStateEstimate() < 0.5){
                temp_state = ControlState.FOLLOW;
//            } else if (Math.min(left_dist, left_dist_close) > 0.5){
//            	temp_state = ControlState.LEFT_FAR;
            } else {
                temp_state = ControlState.DEFAULT;
            }
            
            if (target_radius != 0){
                temp_state = ControlState.APPROACH;
            }
        }
        
        // MAKE EXCEPTIONS FOR SCORING STATES
        if (prev_state != ControlState.REACTOR_ALIGNED || prev_state != ControlState.REACTOR_APPROACH
                || prev_state != ControlState.REACTOR_FAR_LEFT || prev_state != ControlState.REACTOR_FAR_RIGHT
                || prev_state != ControlState.REACTOR_IMMEDIATE){
            if (encoder_flag || state.getTime() > 8000){
                state.changeState(ControlState.PULL_AWAY);
            } else if (state.getTime() > 100 && state.state != ControlState.PULL_AWAY
                    && state.state != ControlState.RANDOM_ORIENT && state.state != ControlState.LEFT_FAR
                    && state.state != ControlState.FORWARD){
                state.changeState(temp_state);
            } else if (getTurnStateEstimate() < 0.1 || getAlignStateEstimate() < 0.1){
                state.changeState(temp_state);
            }
        }
        
        if (state.getTime() > 1000 && state.state == ControlState.PULL_AWAY){
            state.changeState(ControlState.RANDOM_ORIENT);
            orient_time = 1500 + 1000*Math.random();
        }
        
        if (state.getTime() > orient_time && state.state == ControlState.RANDOM_ORIENT){
            state.changeState(temp_state);
        }
        
        if (state.state == ControlState.REACTOR_FAR_LEFT && state.getTime() > 4500){
            state.changeState(ControlState.REACTOR_APPROACH);
        } else if (state.state == ControlState.REACTOR_FAR_LEFT){
            state.changeState(ControlState.REACTOR_FAR_LEFT);
        }
        
        if (state.state == ControlState.REACTOR_FAR_RIGHT && state.getTime() > 4500){
            state.changeState(ControlState.REACTOR_APPROACH);
        } else if (state.state == ControlState.REACTOR_FAR_RIGHT){
            state.changeState(ControlState.REACTOR_FAR_RIGHT);
        }
        
        if (state.state == ControlState.REACTOR_ALIGNED && state.getTime() > 10000){
            state.changeState(ControlState.RANDOM_ORIENT);
            orient_time = 1500 + 1000*Math.random();
        } else if (state.state == ControlState.REACTOR_ALIGNED){
            state.changeState(ControlState.REACTOR_ALIGNED);
        }
        
        if (state.state == ControlState.APPROACH && prev_state != ControlState.APPROACH){
            intake_time = System.currentTimeMillis();
            ball_intake.setSpeed(-0.25);
//            ball_colors.add(ball_color);
        }
        
//        if ((state.state == ControlState.APPROACH || state.state == ControlState.COLLECT) &&
//                (prev_ball_color != ball_color)){
//            ball_colors.add(ball_color);
//        }
        
//        if (state.getTime() > 500 && state.state == ControlState.LEFT_FAR){
//            state.changeState(ControlState.FORWARD);
//        }
//        
//        if (state.getTime() > 1800 && state.state == ControlState.FORWARD){
//            state.changeState(temp_state);
//        }
    }
    
    private double getTurnStateEstimate(){
    	double dist = Math.min(cam_dist, Math.min(left_dist, left_dist_close/1.1));
    	if (validC){
    		dist = Math.min(dist, distanceC);
    	}
    	if (validD){
    		dist = Math.min(dist, distanceD);
    	}
    	if (validE){
    		dist = Math.min(dist, distanceE);
    	}
    	return dist;
    }
    
    private double getAlignStateEstimate(){
    	double dist = 0.75*left_dist;
    	if (validA){
    		dist = Math.min(dist, distanceA);
    	}
    	if (validB){
    		dist = Math.min(dist, distanceB);
    	}
    	if (validC){
    		dist = Math.min(dist, distanceC/1.2);
    	}
    	return dist;
    }
    
    private void updateEncoderFlag(){
        double ang_dist = Math.abs(encoderL.getTotalAngularDistance()) + Math.abs(encoderR.getTotalAngularDistance());
        System.out.println("Angular Distance: " + ang_dist);
        buff_encoder.remove(0);
        buff_encoder.add(ang_dist);
        double init = buff_encoder.get(0);
        
        // TUNE CONSTANTS (Math.PI/2)
        boolean is_const = true;
        for (Double val : buff_encoder){
            if (Math.abs(val - init) > Math.PI/2){
                is_const = false;
            }
        }
        
        if (is_const){
            if (encoder_flag_time == 0){
                encoder_flag_time = System.currentTimeMillis();
            } else if (System.currentTimeMillis() > encoder_flag_time + 5000){
                encoder_flag = true;
            }
        } else {
            encoder_flag = false;
            encoder_flag_time = 0;
        }
        
    }
    
    private void updateHopper(){
        System.out.println("BALL LIST LENGTH: " + ball_colors.size());
        if (ball_colors.size() != 0){
            if (ball_colors.get(0) == 0){
                System.out.println("GREEN BALL READY");
            } else {
                System.out.println("RED BALL READY");
            }
        }
        
        if (hopper.ballQueued()){
            motorL.setSpeed(0);
            motorR.setSpeed(0);
            comm.transmit();
            if (ball_colors.size() != 0){
                if (ball_colors.get(0) == 0){
                    hopper.fastGreenSort();
                    ball_colors.remove(0);
                } else {
                    hopper.fastRedSort();
                    ball_colors.remove(0);
                }
            }
        }
    }
    
    private void print(){
        System.out.println("distanceD: " + sonarD.getDistance());
        System.out.println("distanceE: " + sonarE.getDistance());
        System.out.println("distanceA: " + sonarA.getDistance());
        System.out.println("distanceB: " + sonarB.getDistance());
        System.out.println("distanceC: " + sonarC.getDistance());
        System.out.println("Camera: " + cam_dist);
        System.out.println("Left: " + left_dist);
        //System.out.println("SIDE: " + Math.min(distanceA, distanceB));
        //System.out.println("FRONT: " + Math.min(distanceD, distanceE));
        //System.out.println("forward: " + forward);
        //System.out.println("turn: " + turn);
    }
    
    private void updateSonarBuffers(boolean reset_relay){
        boolean const_values = true;
        double init = buffA.get(0);
        for (Double val : buffA){
            if (Math.abs(init - val) > 0.0000001 && val > 0.01){
                const_values = false;
            }
        }
        validA = !const_values;
        
        const_values = true;
        init = buffB.get(0);
        for (Double val : buffB){
            if (Math.abs(init - val) > 0.0000001 && val > 0.01){
                const_values = false;
            }
        }
        validB = !const_values;
        
        const_values = true;
        init = buffC.get(0);
        for (Double val : buffC){
            if (Math.abs(init - val) > 0.0000001 && val > 0.01){
                const_values = false;
            }
        }
        validC = !const_values;
        
        const_values = true;
        init = buffD.get(0);
        for (Double val : buffD){
            if (Math.abs(init - val) > 0.0000001 && val > 0.01){
                const_values = false;
            }
        }
        validD = !const_values;
        
        const_values = true;
        init = buffE.get(0);
        for (Double val : buffE){
            if (Math.abs(init - val) > 0.0000001 && val > 0.01){
                const_values = false;
            }
        }
        validE = !const_values;
        
        if (!validA){
        	System.out.println("LOST SONAR A");
        }
        if (!validB){
        	System.out.println("LOST SONAR B");
        }
        if (!validC){
        	System.out.println("LOST SONAR C");
        }
        if (!validD){
        	System.out.println("LOST SONAR D");
        }
        if (!validE){
        	System.out.println("LOST SONAR E");
        }
        
        buffD.remove(0);
        buffD.add(distanceD);
        buffE.remove(0);
        buffE.add(distanceE);
        buffA.remove(0);
        buffA.add(distanceA);
        buffB.remove(0);
        buffB.add(distanceB);
        buffC.remove(0);
        buffC.add(distanceC);
        
        if (reset_relay){
        	if ((!validA || !validB || !validC || !validD || !validE)
        			&& System.currentTimeMillis() > reset_time + 15000){
        		System.out.println("RESETTING RELAY");
        		power_sonars.setValue(true);
        		motorL.setSpeed(0);
        		motorR.setSpeed(0);
        		ball_intake.setSpeed(0);

        		try {
        			Thread.sleep(2500);
        		} catch (Exception exc){
        			exc.printStackTrace();
        		}
        		comm.transmit();

        		power_sonars.setValue(false);

        		comm.transmit();

        		try {
        			Thread.sleep(2500);
        		} catch (Exception exc){
        			exc.printStackTrace();
        		}

        		reset_time = System.currentTimeMillis();
        	}
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
