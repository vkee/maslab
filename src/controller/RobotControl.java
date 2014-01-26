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

public class RobotControl {
    public static void main(String[] args){      
        RobotControl robot = new RobotControl();
        robot.loop();
    }
    
    // MAPLE
    private MapleComm comm;
    
    // VISION
    Thread vision_thread;
    final Vision vision;
    
    // CONSTANTS
    private final int WIDTH = 320;
    private final int HEIGHT = 240;
    private final int BUFF_LENGTH = 10;
    private final int CAMERA_NUM = 1;
    private final int CHANGE_THRES = 2;
    private final boolean DISPLAY_ON = true;
    
    // STATE VALUES
    double distanceL, distanceR, distanceA, distanceB, distanceC;
    long start_time, end_time;
    //double target_x, target_y;
    //boolean target_found;
    double K_encoder;
    
    // BUFFERS
    List<Double> buffL, buffR, buffA, buffB, buffC;
    
    // MOTOR INPUTS
    private double forward;
    private double turn;
    
    // SENSORS AND ACTUATORS
    private Cytron motorL, motorR;
    private Ultrasonic sonarL, sonarR, sonarA, sonarB, sonarC;
    private Encoder encoderL, encoderR;
    private DigitalOutput relay;
    //private PWMOutput roller;
    
    // PIDS
    PID pid_align, pid_speedwf, pid_speedbc;
    
    // STATES
    private State state;
    
    private enum ControlState { DEFAULT, WALL_AHEAD, TURNING, ADJACENT_LEFT,
        ADJACENT_RIGHT, TARGETING_BALL, APPROACHING_BALL, COLLECTING_BALL, PULL_AWAY };
    
    public RobotControl(){
        comm = new MapleComm(MapleIO.SerialPortType.WINDOWS);
        
        // MOTOR INPUTS
        forward = 0;
        turn = 0;
        
        // VISION
        vision = new Vision(CAMERA_NUM, WIDTH, HEIGHT, DISPLAY_ON);
        
        vision_thread = new Thread(new Runnable(){
            public void run(){
                Engine.initGL(WIDTH, HEIGHT);
                
                long start_time, end_time;
                
                while (true){
                    start_time = System.currentTimeMillis();
                    vision.update();
                    end_time = System.currentTimeMillis();
                    try {
                        if (75 + start_time - end_time > 0){
                            Thread.sleep(75 + start_time - end_time);
                        }
                    } catch (Exception exc){
                        exc.printStackTrace();
                    }
                }
            }
        });
        
        // SENSORS AND ACTUATORS        
        motorL = new Cytron(4, 0);
        motorR = new Cytron(10, 1);
        
        sonarA = new Ultrasonic(30, 29);
        sonarB = new Ultrasonic(32, 31);
        sonarC = new Ultrasonic(34, 33);
        sonarL = new Ultrasonic(35, 36);
        sonarR = new Ultrasonic(26, 25);
        
        encoderL = new Encoder(5, 7);
        encoderR = new Encoder(6, 8);
        relay = new DigitalOutput(37);
        
        // REGISTER DEVICES AND INITIALIZE
        comm.registerDevice(sonarL);
        comm.registerDevice(sonarR);
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
        
        // PIDS
        pid_align = new PID(0.15, 0.5, 0.08, 0.01);  
        pid_align.update(sonarL.getDistance(), true);
        
        pid_speedwf = new PID(10, 0.2, 0.08, 0.01);
        pid_speedwf.update(10, true);
        
        pid_speedbc = new PID(5, 0.2, 0.08, 0.01);
        pid_speedbc.update(5, true);
        
        // BUFFERS
        buffL = new LinkedList<Double>();
        buffR = new LinkedList<Double>();
        buffA = new LinkedList<Double>();
        buffB = new LinkedList<Double>();
        buffC = new LinkedList<Double>();
        
        // VALUES
        distanceL = sonarL.getDistance();
        distanceR = sonarR.getDistance();
        distanceA = sonarA.getDistance();
        distanceB = sonarB.getDistance();
        distanceC = sonarC.getDistance();
        
        for (int i = 0; i < BUFF_LENGTH; i++){
            buffL.add(distanceL);
            buffR.add(distanceR);
            buffA.add(distanceA);
            buffB.add(distanceB);
            buffC.add(distanceC);
        }
        
        K_encoder = 1;
        
        // STATE INITIALIZATION
        state = new State(ControlState.DEFAULT);
    }
    
    private void loop(){ 
        System.out.println("Beginning to follow wall...");
        
        // START VISION
        vision_thread.start();
        
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
            comm.updateSensorData();
            
            start_time = System.currentTimeMillis();
            
            // UPDATE DISTANCES
            distanceL = sonarL.getDistance();
            distanceR = sonarR.getDistance();
            distanceA = sonarA.getDistance();
            distanceB = sonarB.getDistance();
            distanceC = sonarC.getDistance();
            
            // UPDATE BUFFERS
            updateSonarBuffers();
            
            // ESTIMATE STATE
            estimateState();
            
            // UPDATE MOTORS
            updateMotors();
            
            print();
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
        
        if (state.state == ControlState.WALL_AHEAD){
            System.out.println("WALL AHEAD");
            temp_turn = 0.12;
            temp_forward = 0;
        } else if (state.state == ControlState.TURNING){
            System.out.println("TURNING");
            temp_turn = 0.12;
            temp_forward = 0;
        } else if (state.state == ControlState.ADJACENT_LEFT){
            System.out.println("ADJACENT_LEFT");
            temp_turn = 0.05;
            temp_forward = 0.08;
        } else if (state.state == ControlState.ADJACENT_RIGHT){
            System.out.println("ADJACENT_RIGHT");
            temp_turn = -0.08;
            temp_forward = 0;
        } else if (state.state == ControlState.DEFAULT){
            System.out.println("DEFAULT");
            temp_turn = Math.max(-0.1, Math.min(0.1, pid_align.update(distanceL, false)));
            temp_forward = 0.08;
        } else if (state.state == ControlState.PULL_AWAY){
            System.out.println("PULL_AWAY");
            if (distanceR > distanceL){
                temp_turn = -0.1;
            } else {
                temp_turn = 0.1;
            }
            temp_forward = -0.08;
        }

        double abs_speed = Math.abs(encoderL.getAngularSpeed()) + Math.abs(encoderR.getAngularSpeed()); 
        K_encoder = Math.max(pid_speedwf.update(abs_speed, false), 0.5);
        
        turn = K_encoder*temp_turn;
        forward = K_encoder*temp_forward;
        
        motorL.setSpeed(-(forward + turn));
        motorR.setSpeed(forward - turn);
    }

    private void estimateState(){
        ControlState temp_state = ControlState.DEFAULT;
        
        // TUNE CONDITIONS
        if (distanceC < 0.2){
            temp_state = ControlState.WALL_AHEAD;
        } else if (distanceB < 0.15 || (distanceA < 0.2 && distanceB < 0.2)){
            temp_state = ControlState.TURNING;
        } else if (distanceL < 0.13){
            temp_state = ControlState.ADJACENT_LEFT;
        } else if (distanceR < 0.1){
            temp_state = ControlState.ADJACENT_RIGHT;
        } else {
            temp_state = ControlState.DEFAULT;
        }

        // TUNE CUTOFFS
        if ((state.getTime() > 400 && state.state != ControlState.PULL_AWAY) ||
                (state.getTime() > 2000 && state.state == ControlState.PULL_AWAY)){
            state.changeState(temp_state);
        }

        // TUNE CUTOFF
        if (state.getTime() > 8000){
            state.changeState(ControlState.PULL_AWAY);
        }
    }
    
    private void print(){
        System.out.println("distanceL: " + sonarL.getDistance());
        System.out.println("distanceR: " + sonarR.getDistance());
        System.out.println("distanceA: " + sonarA.getDistance());
        System.out.println("distanceB: " + sonarB.getDistance());
        System.out.println("distanceC: " + sonarC.getDistance());
        //System.out.println("forward: " + forward);
        //System.out.println("turn: " + turn);
    }
    
    private void updateSonarBuffers(){
        boolean const_values = true;
        double init = buffL.get(0);
        for (Double val : buffL){
            if (Math.abs(val - init) > 0.0000000001){
                const_values = false;
            }
        }
        if (!const_values){
            init = buffR.get(0);
            for (Double val : buffR){
                if (Math.abs(val - init) > 0.0000000001){
                    const_values = false;
                }
            }
        }
        if (!const_values){
            init = buffA.get(0);
            for (Double val : buffA){
                if (Math.abs(val - init) > 0.0000000001){
                    const_values = false;
                }
            }
        }
        if (!const_values){
            init = buffB.get(0);
            for (Double val : buffB){
                if (Math.abs(val - init) > 0.0000000001){
                    const_values = false;
                }
            }
        }
        if (!const_values){
            init = buffC.get(0);
            for (Double val : buffC){
                if (Math.abs(val - init) > 0.000000001){
                    const_values = false;
                }
            }
        }
        
        buffL.remove(0);
        buffL.add(distanceL);
        buffR.remove(0);
        buffR.add(distanceR);
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
                Thread.sleep(250);
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
