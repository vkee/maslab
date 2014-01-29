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

public class BallCollectControl {
    public static void main(String[] args){      
        BallCollectControl robot = new BallCollectControl();
        robot.loop();
    }
    
    // MAPLE
    private MapleComm comm;
    
    // VISION
    final Vision vision;
    
    // CONSTANTS
    private final int WIDTH = 320;
    private final int HEIGHT = 240;
    private final int BUFF_LENGTH = 10;
    private final int CAMERA_NUM = 1;
    private final int CHANGE_THRES = 2;
    private final boolean DISPLAY_ON = true;
    
    // STATE VALUES
    double distanceD, distanceE, distanceA, distanceB, distanceC;
    int target_x, target_y;
    double target_radius;
    long start_time, end_time;
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
    PID pid_target, pid_approach, pid_speedbc;
    
    // STATES
    private State state;
    
    private enum ControlState { TARGET, APPROACH, COLLECT };
    
    public BallCollectControl(){
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
        
        target_x = 0;
        target_y = 0;
        target_radius = 0;
        
        // PIDS
        pid_target = new PID(WIDTH/2, 0.3, -2, 0);
        pid_target.update(WIDTH/2, true);
        
        pid_approach = new PID(WIDTH/2, 0.3, -2, 0);
        pid_approach.update(WIDTH/2, true);
        
        pid_speedbc = new PID(10, 0.15, -0.04, 0.01);
        pid_speedbc.update(10, true);
        
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
        state = new State(ControlState.APPROACH);
    }
    
    private void loop(){ 
        System.out.println("Beginning to follow wall...");
        
        // INITIALIZE SONARS
        relay.setValue(false);
        comm.transmit();
        
        comm.updateSensorData();
        
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
            
            // UPDATE DISTANCES
            distanceD = sonarD.getDistance();
            distanceE = sonarE.getDistance();
            distanceA = sonarA.getDistance();
            distanceB = sonarB.getDistance();
            distanceC = sonarC.getDistance();
            
            // UPDATE VISION
            vision.update();
            target_x = vision.getNextBallX();
            target_y = vision.getNextBallY();
            target_radius = vision.getNextBallRadius();
            
            // UPDATE BUFFERS
            //updateSonarBuffers();
            
            // ESTIMATE STATE
            estimateState();
            
            // UPDATE MOTORS
            updateMotors();
            
            //print();
            comm.transmit();
            
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
    }
    
    private void updateMotors(){
        double temp_turn = 0;
        double temp_forward = 0.1;
        double update_value_x;
        
        if (state.state == ControlState.TARGET){
            System.out.println("TARGET");
            
            if (target_radius > 0){
                update_value_x = WIDTH/2 + ((target_x - WIDTH/2)*12/target_radius);
            } else {
                update_value_x = target_x;
            }
            
            temp_turn = Math.max(-0.25, Math.min(0.25, -pid_target.update(target_x, false)/WIDTH));
            temp_forward = 0;
        } else if (state.state == ControlState.APPROACH){
            System.out.println("APPROACH");
            
            if (target_radius > 0){
                update_value_x = WIDTH/2 + ((target_x - WIDTH/2)*12/target_radius);
            } else {
                update_value_x = target_x;
            }
            
            temp_turn = Math.max(-0.25, Math.min(0.25, -pid_target.update(target_x, false)/WIDTH));
            temp_forward = 0.17;
        } else if (state.state == ControlState.COLLECT){
            System.out.println("COLLECT");
            temp_forward = 0.13;
        }

        double abs_speed = Math.abs(-encoderL.getAngularSpeed()) + Math.abs(encoderR.getAngularSpeed()); 
        K_encoder = 1 + Math.max(-0.3, Math.min(0.3, pid_speedbc.update(abs_speed, false)));
        
        turn = temp_turn;
        //forward = K_encoder*temp_forward;
        //forward = 0.18;
        forward = temp_forward;
        
        //motorL.setSpeed(-(forward + turn));
        //motorR.setSpeed(forward - turn);
    }

    private void estimateState(){
        if (Math.abs(target_x - WIDTH/2) < 5 && state.state == ControlState.TARGET){
            state.changeState(ControlState.APPROACH);
        } else if (target_y > 180 && state.state == ControlState.APPROACH){
            state.changeState(ControlState.COLLECT);
        }
    }
    
    private void print(){
        //System.out.println("distanceD: " + sonarD.getDistance());
        //System.out.println("distanceE: " + sonarE.getDistance());
        //System.out.println("distanceA: " + sonarA.getDistance());
        //System.out.println("distanceB: " + sonarB.getDistance());
        //System.out.println("distanceC: " + sonarC.getDistance());
        //System.out.println("SIDE: " + Math.min(distanceA, distanceB));
        //System.out.println("FRONT: " + Math.min(distanceD, distanceE));
        System.out.println("target_x: " + target_x);
        System.out.println("target_y:" + target_y);
        System.out.println("forward: " + forward);
        System.out.println("turn: " + turn);
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
