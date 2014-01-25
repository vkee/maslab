package controller;

import comm.MapleComm;
import comm.MapleIO;
import devices.actuators.Cytron;
import devices.actuators.DigitalOutput;
import devices.actuators.PWMOutput;
import devices.sensors.Encoder;
import devices.sensors.Ultrasonic;

public class ControlMock {
    public static void main(String[] args){      
        ControlMock robot = new ControlMock();
        robot.loop();
    }
    
    // MAPLE
    private MapleComm comm;
    
    // VISION
    //Vision vision;
    
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
    //double target_x, target_y;
    //boolean target_found;
    double prev_encoderL, prev_encoderR;
    
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
    PID pid_align, pid_encoder;
    
    // STATES
    private State state;
    
    private enum ControlState { DEFAULT, WALL_AHEAD, TURNING, ADJACENT_LEFT,
        ADJACENT_RIGHT, TARGETING_BALL, APPROACHING_BALL, COLLECTING_BALL, PULL_AWAY };
    
    public ControlMock(){
        comm = new MapleComm(MapleIO.SerialPortType.WINDOWS);
        
        // MOTOR INPUTS
        forward = 0;
        turn = 0;
        
        // VISION
        //vision = new Vision(CAMERA_NUM, WIDTH, HEIGHT, DISPLAY_ON);
        
        // SENSORS AND ACTUATORS        
        motorL = new Cytron(4, 0);
        motorR = new Cytron(10, 1);
        
        sonarA = new Ultrasonic(30, 29);
        sonarB = new Ultrasonic(32, 31);
        sonarC = new Ultrasonic(34, 33);
        sonarL = new Ultrasonic(36, 35);
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
        
        pid_encoder = new PID(1, 0.5, 0.08, 0.01);
        pid_encoder.update(0.5, true);
        
        // VALUES
        distanceL = sonarL.getDistance();
        distanceR = sonarR.getDistance();
        distanceA = sonarA.getDistance();
        distanceB = sonarB.getDistance();
        distanceC = sonarC.getDistance();
        
        prev_encoderL = 0;
        prev_encoderR = 0;
        
        // STATE INITIALIZATION
        state = new State(ControlState.DEFAULT);
    }
    
    private void loop(){ 
        System.out.println("Beginning to follow wall...");
        
        // INITIALIZE SONARS
        relay.setValue(false);
        //roller.setValue(1);
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
            
            // UPDATE VISION
            //vision.update();
            
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
            
            end_time = System.currentTimeMillis();
            
            try {
                if (40 - end_time + start_time >= 0){
                    Thread.sleep(30 - end_time + start_time);
                }
            } catch (Exception exc){
                exc.printStackTrace();
            }
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
            forward = 0.08;
        } else if (state.state == ControlState.ADJACENT_RIGHT){
            System.out.println("ADJACENT_RIGHT");
            turn = -0.08;
            forward = 0;
        } else if (state.state == ControlState.DEFAULT){
            System.out.println("DEFAULT");
            turn = Math.max(-0.1, Math.min(0.1, pid_align.update(distanceL, false)));
            forward = 0.08;
        } else if (state.state == ControlState.PULL_AWAY){
            System.out.println("PULL_AWAY");
            if (distanceR > distanceL){
                turn = -0.1;
            } else {
                turn = 0.1;
            }
            forward = -0.08;
        }
        turn = turn*1.25;
        forward = forward*1.5;
    }

    private void estimateState(){
        ControlState temp_state = ControlState.DEFAULT;
        
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

        if ((state.getTime() > 400 && state.state != ControlState.PULL_AWAY) ||
                (state.getTime() > 1000 && state.state == ControlState.PULL_AWAY)){
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
