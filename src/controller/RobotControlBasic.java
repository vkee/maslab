package controller;

import java.util.LinkedList;
import java.util.List;

import comm.MapleComm;
import comm.MapleIO;
import devices.actuators.Cytron;
import devices.actuators.DigitalOutput;
import devices.sensors.Encoder;
import devices.sensors.Gyroscope;
import devices.sensors.Ultrasonic;

public class RobotControlBasic {
    public static void main(String[] args){      
        RobotControlBasic robot = new RobotControlBasic();
        robot.wallFollow();
    }
    
    double forward;
    double turn;
    
    MapleComm comm;
    Cytron motorL, motorR;
    Ultrasonic sonarA, sonarB, sonarL, sonarR;
    DigitalOutput relay;
    
    ControlState control_state;
    MapState map_state;
    int map_state_count;
    
    private enum ControlState { WALL_FOLLOW, BALL_COLLECT };
    private enum MapState { DEFAULT, ALIGNED, WALL_AHEAD, WALL_IMMEDIATE };
    
    public RobotControlBasic(){
        comm = new MapleComm(MapleIO.SerialPortType.WINDOWS);
        forward = 0;
        turn = 0;
        
        motorL = new Cytron(0, 1);
        motorR = new Cytron(2, 3);
        
        sonarA = new Ultrasonic(32, 31);
        sonarB = new Ultrasonic(34, 33);
        sonarL = new Ultrasonic(36, 35);
        sonarR = new Ultrasonic(26, 25);
        
        relay = new DigitalOutput(37);
        
        comm.registerDevice(sonarB);
        comm.registerDevice(sonarL);
        comm.registerDevice(motorL);
        comm.registerDevice(motorR);
        comm.registerDevice(relay);
        
        System.out.println("Initializing...");
        comm.initialize();
        
        control_state = ControlState.WALL_FOLLOW;
        map_state = MapState.DEFAULT;
        map_state_count = 0;
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
        
        // Initialize Sonars
        relay.setValue(false);
        
        // Values
        double distanceL = sonarL.getDistance();
        double distanceB = sonarB.getDistance();
        MapState prev_map_state = MapState.DEFAULT;
        
        // Distance Window
        List<Double> prev_dists = new LinkedList<Double>();
        int WIN_LENGTH = 6;
        for (int i = 0; i < WIN_LENGTH; i++){
            prev_dists.add(distanceB);
        }
        double dist_exp = distanceL;
        double dist_sqexp = distanceL*distanceL;
        double prev_dist = distanceL;
        double dist_var = 0;
        
        // PID
        PID pid_align = new PID(0.15, 0.2, 0.01, 0.01);
        turn = Math.max(-0.05, Math.min(0.05, pid_align.update(dist_exp, false)));
        forward = 0.1;
        
        // Initialize
        motorL.setSpeed(forward + turn);
        motorR.setSpeed(forward - turn);

        comm.transmit();
        
        try {
            Thread.sleep(10);
        } catch (InterruptedException exc) {
            exc.printStackTrace();
        }
        
        while (true){
            System.out.println("NEW ITERATION:");
            comm.updateSensorData();
            
            // UPDATE VALUES
            distanceL = sonarL.getDistance();
            distanceB = sonarB.getDistance();

            // UPDATE DISTANCE WINDOW
            prev_dist = prev_dists.get(0);
            dist_exp = (WIN_LENGTH*dist_exp + distanceL - prev_dist)/WIN_LENGTH;
            dist_sqexp = (WIN_LENGTH*dist_sqexp + distanceL*distanceL - prev_dist*prev_dist)/WIN_LENGTH;
            dist_var = dist_sqexp - dist_exp*dist_exp;
            prev_dists.add(distanceL);
            prev_dists.remove(0);
            
            System.out.println("dist_var: " + dist_var);
            System.out.println("distanceB: " + distanceB);
            System.out.println("distanceL: " + distanceL);

            prev_map_state = map_state;

            if (distanceB < 0.15){
                map_state = MapState.WALL_IMMEDIATE;
            } else if (distanceB < 0.3){
                map_state = MapState.WALL_AHEAD;
            } else if (dist_var < 0.005 && Math.abs(distanceL - 0.15) < 0.01){
                map_state = MapState.ALIGNED;
            } else {
                map_state = MapState.DEFAULT;
            }
            
            if (prev_map_state != map_state){
                map_state_count = 0;
            }
            
            map_state_count++;
            
            if (map_state == MapState.DEFAULT){
                turn = Math.max(-0.05, Math.min(0.05, pid_align.update(dist_exp, false)));
                forward = 0.1;
                System.out.println("Default");
            } else if (map_state == MapState.ALIGNED){
                turn = Math.max(-0.05, Math.min(0.05, pid_align.update(dist_exp, false)));
                forward = 0.1;
                System.out.println("Aligned");
            } else if (map_state == MapState.WALL_AHEAD){
                turn = 0.1;
                forward = (distanceB - 0.15)/1.5;
                System.out.println("Wall Ahead");
            } else if (map_state == MapState.WALL_IMMEDIATE){
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
}
