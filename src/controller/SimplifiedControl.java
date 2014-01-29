package controller;

import java.util.LinkedList;
import java.util.List;

import Core.Engine;
import vision.Vision;
import comm.MapleComm;
import comm.MapleIO;
import devices.actuators.Cytron;
import devices.actuators.DigitalOutput;
import devices.actuators.PWMOutput;
import devices.sensors.Encoder;
import devices.sensors.Ultrasonic;

public class SimplifiedControl {
    public static void main(String[] args){      
        SimplifiedControl robot = new SimplifiedControl();
        robot.loop();
    }
    
    // VISION
    final Vision vision;
    
    // MAPLE
    private MapleComm comm;
    
    // CONSTANTS
    private final int WIDTH = 320;
    private final int HEIGHT = 240;
    private final int BUFF_LENGTH = 10;
    private final int CAMERA_NUM = 1;
    private final int CHANGE_THRES = 2;
    private final boolean DISPLAY_ON = true;
    
    // STATE VALUES
    double distanceD, distanceE, distanceA, distanceB, distanceC;
    long start_time, end_time;
    int target_x, target_y;
    double target_radius;
    double K_encoder;
    
    // BUFFERS
    List<Double> buffD, buffE, buffA, buffB, buffC;
    
    // MOTOR INPUTS
    private double forward;
    private double turn;
    
    // SENSORS AND ACTUATORS
    private Cytron motorL, motorR;
    private Ultrasonic sonarD, sonarE, sonarA, sonarB, sonarC;
    private Encoder encoderL, encoderR;
    private DigitalOutput relay;
    
    // PIDS
    PID pid_dist, pid_speedwf;
    PID pid_target, pid_approach;
    
    // STATES
    private State state;
    
    private enum ControlState { DEFAULT, WALL_AHEAD, FOLLOW, PULL_AWAY, APPROACH, COLLECT };
    
    public SimplifiedControl(){
        comm = new MapleComm(MapleIO.SerialPortType.WINDOWS);
        
        // VISION
        vision = new Vision(1, 320, 240, true);
        
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
        relay = new DigitalOutput(37);
        
        // REGISTER DEVICES AND INITIALIZE
        comm.registerDevice(sonarD);
        comm.registerDevice(sonarE);
        comm.registerDevice(sonarA);
        comm.registerDevice(sonarB);
        comm.registerDevice(sonarC);
        
        comm.registerDevice(motorL);
        comm.registerDevice(motorR);
        
        comm.registerDevice(encoderL);
        comm.registerDevice(encoderR);
        
        comm.registerDevice(relay);
        
        System.out.println("Initializing...");
        comm.initialize();
        
        comm.updateSensorData();
        
        // VALUES
        distanceD = sonarD.getDistance();
        distanceE = sonarE.getDistance();
        distanceA = sonarA.getDistance();
        distanceB = sonarB.getDistance();
        distanceC = sonarC.getDistance();
        
        // PIDS
        pid_dist = new PID(0.22, 0.3, 100, 0);  
        pid_dist.update(Math.min(distanceA, distanceB), true);
        
        pid_speedwf = new PID(10, 0.2, -0.08, 0.01);
        pid_speedwf.update(10, true);
        
        pid_target = new PID(WIDTH/2, 0.3, -2, 0);
        pid_target.update(WIDTH/2, true);
        
        pid_approach = new PID(WIDTH/2, 0.3, -2, 0);
        pid_approach.update(WIDTH/2, true);
        
        // BUFFERS
        buffD = new LinkedList<Double>();
        buffE = new LinkedList<Double>();
        buffA = new LinkedList<Double>();
        buffB = new LinkedList<Double>();
        buffC = new LinkedList<Double>();
        
        for (int i = 0; i < BUFF_LENGTH; i++){
            buffD.add(distanceD);
            buffE.add(distanceE);
            buffA.add(distanceA);
            buffB.add(distanceB);
            buffC.add(distanceC);
        }
        
        K_encoder = 1;
        
        // STATE INITIALIZATION
        state = new State(ControlState.FOLLOW);
    }
    
    private void loop(){ 
        System.out.println("Beginning to follow wall...");
        
        // INITIALIZE SONARS
        relay.setValue(false);
        comm.transmit();
        
        comm.updateSensorData();
        
        // UPDATE VISION
        vision.update();
        target_x = vision.getNextBallX();
        target_y = vision.getNextBallY();
        target_radius = vision.getNextBallRadius();
        
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
        
        while (true){            
            comm.updateSensorData();
            
            start_time = System.currentTimeMillis();
            
            // UPDATE VISION
            vision.update();
            target_x = vision.getNextBallX();
            target_y = vision.getNextBallY();
            target_radius = vision.getNextBallRadius();
            
            // UPDATE DISTANCES
            distanceD = sonarD.getDistance();
            distanceE = sonarE.getDistance();
            distanceA = sonarA.getDistance();
            distanceB = sonarB.getDistance();
            distanceC = sonarC.getDistance();
            
            // UPDATE BUFFERS
            //updateSonarBuffers();
            
            // ESTIMATE STATE
            estimateState();
            
            // UPDATE MOTORS
            updateMotors();
            
            print();
            comm.transmit();
            
            end_time = System.currentTimeMillis();
            
            try {
                if (60 - end_time + start_time >= 0){
                    Thread.sleep(60 - end_time + start_time);
                } else {
                    System.out.println("TIME OVERFLOW: " + (end_time - start_time));
                }
            } catch (Exception exc){
                exc.printStackTrace();
            }
        }
    }
    
    private void updateMotors(){
        double temp_turn = 0;
        double temp_forward = 0.1;
        double update_value_x;
        
        if (state.state == ControlState.APPROACH){
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
                temp_turn = pid_dist.update(Math.min(distanceA, distanceB), false);
            } else if (state.state == ControlState.PULL_AWAY){
                System.out.println("WALL_FOLLOW: PULL_AWAY");
                temp_turn = -0.1;
                temp_forward = -0.08;
            } else if (state.state == ControlState.DEFAULT){
                System.out.println("WALL_FOLLOW: DEFAULT");
                temp_turn = -0.05;
                temp_forward = 0.1;
            }

            double abs_speed = Math.abs(encoderL.getAngularSpeed()) + Math.abs(encoderR.getAngularSpeed()); 
            K_encoder = Math.max(pid_speedwf.update(abs_speed, false), 0.5);
            
            //turn = K_encoder*temp_turn;
            //forward = K_encoder*temp_forward;
            turn = 1.7*temp_turn;
            forward = 1.7*temp_forward;
        }
        
        motorL.setSpeed(-(forward + turn));
        motorR.setSpeed(forward - turn);
    }

    private void estimateState(){
        ControlState temp_state = state.state;
        
        if (state.state == ControlState.APPROACH){
            if (target_y > 180){
                temp_state = ControlState.COLLECT;
            } else {
                temp_state = ControlState.APPROACH;
            }
        } else if ((state.state == ControlState.COLLECT && state.getTime() >= 1000)
                || state.state != ControlState.COLLECT){
            if (Math.min(distanceD, distanceE) < 0.25 || distanceC < 0.2){
                temp_state = ControlState.WALL_AHEAD;
            } else if (distanceA < 2*distanceB && distanceB < 2*distanceA
                    && distanceA < 0.7 && distanceB < 0.7){
                temp_state = ControlState.FOLLOW;
            } else {
                temp_state = ControlState.DEFAULT;
            }
            
            if (target_radius != 0){
                temp_state = ControlState.APPROACH;
            }
        }
        
        if ((state.getTime() > 100 && state.state != ControlState.PULL_AWAY) ||
                (state.getTime() > 2000 && state.state == ControlState.PULL_AWAY)){
            state.changeState(temp_state);
        }

        if (state.getTime() > 8000){
            state.changeState(ControlState.PULL_AWAY);
        }
    }
    
    private void print(){
        System.out.println("distanceD: " + sonarD.getDistance());
        System.out.println("distanceE: " + sonarE.getDistance());
        System.out.println("distanceA: " + sonarA.getDistance());
        System.out.println("distanceB: " + sonarB.getDistance());
        System.out.println("distanceC: " + sonarC.getDistance());
        //System.out.println("SIDE: " + Math.min(distanceA, distanceB));
        //System.out.println("FRONT: " + Math.min(distanceD, distanceE));
        //System.out.println("forward: " + forward);
        //System.out.println("turn: " + turn);
    }
    
    private void updateSonarBuffers(){
        boolean const_values = true;
        double init = buffD.get(0);
        for (Double val : buffD){
            if (Math.abs(val - init) > 0.0000000001 && val > 0.01){
                const_values = false;
            }
        }
        if (!const_values){
            init = buffE.get(0);
            for (Double val : buffE){
                if (Math.abs(val - init) > 0.0000000001 && val > 0.01){
                    const_values = false;
                }
            }
        }
        if (!const_values){
            init = buffA.get(0);
            for (Double val : buffA){
                if (Math.abs(val - init) > 0.0000000001 && val > 0.01){
                    const_values = false;
                }
            }
        }
        if (!const_values){
            init = buffB.get(0);
            for (Double val : buffB){
                if (Math.abs(val - init) > 0.0000000001 && val > 0.01){
                    const_values = false;
                }
            }
        }
        if (!const_values){
            init = buffC.get(0);
            for (Double val : buffC){
                if (Math.abs(val - init) > 0.000000001 && val > 0.01){
                    const_values = false;
                }
            }
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
        
        if (const_values){
            System.out.println("RESETTING RELAY");
            relay.setValue(true);
            motorL.setSpeed(0);
            motorR.setSpeed(0);
            try {
                Thread.sleep(50);
            } catch (Exception exc){
                exc.printStackTrace();
            }
            comm.transmit();
            
            relay.setValue(false);
            
            comm.transmit();
            
            try {
                Thread.sleep(1000);
            } catch (Exception exc){
                exc.printStackTrace();
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
