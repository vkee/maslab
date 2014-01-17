package controller;

import comm.MapleComm;
import comm.MapleIO;

import devices.actuators.Cytron;
import devices.sensors.Gyroscope;
import devices.sensors.Ultrasonic;

public class RobotController {
    public static void main(String[] args){      
        RobotController robot = new RobotController();
        robot.wallFollow();
    }
    
    double forward;
    double turn;
    
    MapleComm comm;
    Cytron motorL, motorR;
    //Ultrasonic sonarA, sonarB, sonarC, sonarL, sonarR;
    Ultrasonic sonarB, sonarL;
    Gyroscope gyro;
    
    ControlState control_state;
    MapState map_state;
    int map_state_count;
    
    private enum ControlState { WALL_FOLLOW, BALL_COLLECT };
    private enum MapState { DEFAULT, ALIGNED, WALL_AHEAD, WALL_IMMEDIATE };
    
    public RobotController(){
        comm = new MapleComm(MapleIO.SerialPortType.WINDOWS);
        forward = 0;
        turn = 0;
        
        motorL = new Cytron(2, 1);
        motorR = new Cytron(7, 6);
        
        //sonarA = new Ultrasonic(34, 33);
        sonarB = new Ultrasonic(32, 31);
        //sonarC = new Ultrasonic(32, 31);
        sonarL = new Ultrasonic(30, 29); // Fill in with different ports
        //sonarR = new Ultrasonic(5, 6); // Fill in with different ports
        
        //gyro = new Gyroscope(1, 8);
        
        //comm.registerDevice(sonarA);
        comm.registerDevice(sonarB);
        //comm.registerDevice(sonarC);
        comm.registerDevice(sonarL);
        //comm.registerDevice(sonarR);
        comm.registerDevice(motorL);
        comm.registerDevice(motorR);
        
        System.out.println("Initializing...");
        comm.initialize();
        //comm.updateSensorData();
        
        control_state = ControlState.WALL_FOLLOW;
        map_state = MapState.DEFAULT;
        map_state_count = 0;
    }
    
    private void wallFollow(){ 
        System.out.println("Beginning to follow wall...");
        comm.updateSensorData();
        
        // Values
        //double prev_time = 0;
        double prev_dist = sonarL.getDistance();
        //double time = 0;
        //double angle = 0;
        double distanceL = sonarL.getDistance();
        double distanceB = sonarB.getDistance();
        MapState prev_map_state = MapState.DEFAULT;
        
        // PID
        PID pid_align = new PID(0.15, 0.2, 0.01, 0.01);
        //PID pid_gyro = new PID(0, 0.2, 0.01, 0.01);
        //pid_gyro.update(0, true);
        turn = Math.max(-0.05, Math.min(0.05, pid_align.update(0.5*(prev_dist + distanceL), false)));
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
            
            //time = System.currentTimeMillis();
            //angle += (time - prev_time)*gyro.getOmega();
            distanceL = sonarL.getDistance();
            distanceB = sonarB.getDistance();

            System.out.println("distanceB: " + distanceB);
            System.out.println("distanceL: " + distanceL);
            
            prev_map_state = map_state;
            
            if (map_state_count > 30){
                if (distanceB < 0.15){
                    map_state = MapState.WALL_IMMEDIATE;
                } else if (distanceB < 0.3){
                    map_state = MapState.WALL_AHEAD;
                } else if (Math.abs(distanceL - prev_dist) < 0.002 && Math.abs(distanceL - 0.15) < 0.01){
                    map_state = MapState.ALIGNED;
                    //angle = 0;
                } else {
                    map_state = MapState.DEFAULT;
                }
            }
            
            if (prev_map_state != map_state){
                map_state_count = 0;
            }

            switch (map_state){
            case DEFAULT:
                turn = Math.max(-0.05, Math.min(0.05, pid_align.update(0.5*(prev_dist + distanceL), false)));
                forward = 0.1;
                System.out.println("Default");
            case ALIGNED:
                turn = Math.max(-0.05, Math.min(0.05, pid_align.update(0.5*(prev_dist + distanceL), false)));
                //turn = 0.5*Math.max(-0.05, Math.min(0.05, pid_gyro.update(angle, false)));
                //turn += 0.5*Math.max(-0.05, Math.min(0.05, pid_align.update(0.5*(prev_dist + distanceL), false)));
                forward = 0.1;
                System.out.println("Aligned");
            case WALL_AHEAD:
                turn = 0.1;
                forward = (distanceB - 0.15)/1.5;
                System.out.println("Wall Ahead");
            case WALL_IMMEDIATE:
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
