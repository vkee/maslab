package controller;

import java.util.LinkedList;
import java.util.List;

import vision.Vision;
import comm.MapleComm;
import comm.MapleIO;
import competition.Hopper;
import controller.ReactorAlignControl.ReactorState;
import devices.actuators.Cytron;
import devices.actuators.DigitalOutput;
import devices.sensors.Encoder;
import devices.sensors.Ultrasonic;

public class ReactorAlignControl {    
    enum ReactorState { REACTOR_FAR_LEFT, REACTOR_FAR_RIGHT, REACTOR_APPROACH, REACTOR_IMMEDIATE, REACTOR_ALIGNED };
    
	public static void main(String[] args) {
	    ReactorAlignControl robot = new ReactorAlignControl();
	    robot.loop();
	}

    private void loop(){
        MapleComm comm = new MapleComm(MapleIO.SerialPortType.WINDOWS);
        List<Integer> ball_colors = new LinkedList<Integer>();
        Hopper hopper = new Hopper(comm,24,27,28, 14, ball_colors);
        Cytron motorL = new Cytron(4, 0);
        Cytron motorR = new Cytron(3, 1);
        Ultrasonic sonarA = new Ultrasonic(26, 25);
        Ultrasonic sonarB = new Ultrasonic(34, 33);
        Ultrasonic sonarC = new Ultrasonic(35, 36);
        Ultrasonic sonarD = new Ultrasonic(30, 29);
        Ultrasonic sonarE = new Ultrasonic(32, 31);
        
        Encoder encoderL = new Encoder(5, 7);
        Encoder encoderR = new Encoder(6, 8);

        DigitalOutput relay = new DigitalOutput(37);

        comm.registerDevice(motorL);
        comm.registerDevice(motorR);
        comm.registerDevice(sonarA);
        comm.registerDevice(sonarB);
        comm.registerDevice(sonarC);
        comm.registerDevice(sonarD);
        comm.registerDevice(sonarE);
        
        comm.registerDevice(encoderL);
        comm.registerDevice(encoderR);
        
        comm.registerDevice(relay);
        comm.initialize();
        
        relay.setValue(false);
        comm.transmit();

        comm.updateSensorData();

        int width = 320;
        int height = 240;
        
        final Vision vision = new Vision(1, 320, 240, true);

        List<Double> left_ratios = new LinkedList<Double>();
        
        for (int i = 0; i < 10; i++){
            left_ratios.add(5.0);
        }
        
        List<Double> right_ratios = new LinkedList<Double>();
        
        for (int i = 0; i < 10; i++){
            right_ratios.add(5.0);
        }
        
        vision.update();
        
        TestState state = new TestState(ReactorState.REACTOR_APPROACH);
        
        double distanceD = sonarD.getDistance();
        double distanceE = sonarE.getDistance();
        
        double distance = 0;
        double cam_dist = 0;

        double distance_left = vision.getLeftmostWallDistance();
        double distance_right = vision.getRightmostWallDistance();
        double distance_reactor = vision.getNextReactorDistance();
        double target_x = vision.getNextReactorX();
        
        double left_ratio = (distance_reactor - distance_left)/target_x;
        double right_ratio = (distance_reactor - distance_right)/(320 - target_x);
        
        double forward = 0;
        double turn = 0;
        
        PID pid_align = new PID(width/2, 0.3, 0, 0);
        PID pid_distance = new PID(0.05, 1.5, -0.2, 0);
        pid_align.update(0, true);
        pid_distance.update(0.05, true);
        
        hopper.rampClose();
        hopper.pacmanClose();
        hopper.sorterBlocking();

        boolean left_far, right_far;
        
        while (true) {
            comm.updateSensorData();
            
            vision.update();
            
            cam_dist = vision.getWallDistance();
            distanceD = sonarD.getDistance();
            distanceE = sonarE.getDistance();
            
            // COMPUTE DISTANCE
//            distance = 2*cam_dist;
//            if (distanceE < 4*cam_dist/3.0 && distanceE > 2*cam_dist/3.0){
//                distance = distanceE;
//            }
//            if (distanceD < distance && distanceD < 4*cam_dist/3.0 && distanceD > 2*cam_dist/3.0){
//                distance = distanceD;
//            }
//            if (distance > 4*cam_dist/3.0){
//                distance = cam_dist;
//            }
            distance = Math.min(distanceD, distanceE);
            
            distance_reactor = vision.getNextReactorDistance();
            distance_left = vision.getLeftmostWallDistance();
            distance_right = vision.getRightmostWallDistance();
            target_x = vision.getNextReactorX();
            
            if (target_x != 0){
                left_ratio = 1000*(distance_reactor - distance_left)/target_x;
                right_ratio = 1000*(distance_reactor - distance_right)/(320 - target_x);
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
            
            System.out.println("target_x: " + target_x);
            System.out.println("left_ratio: " + left_ratio);
            
//            System.out.println("DistanceReactor: " + distanceReactor);
//            System.out.println("DistanceLeft: " + distanceLeft);
//            System.out.println("DistanceRight: " + distanceRight);
//            System.out.println("Distance: " + distance);
//            System.out.println("EncoderL: " + (-encoderL.getAngularSpeed()));
//            System.out.println("EncoderR: " + encoderR.getAngularSpeed());
//            System.out.println("Left: " + vision.getLeftmostWallDistance());
//            System.out.println("Reactor: " + vision.getNextReactorDistance());
//            double ratio = (vision.getNextReactorDistance() - vision.getLeftmostWallDistance())/vision.getNextReactorX();
//            System.out.println("Wall Ratio: " + (1000*ratio));
            
            if ((distance_left < distance_right && left_far) || state.state == ReactorState.REACTOR_FAR_LEFT){
                state.changeState(ReactorState.REACTOR_FAR_LEFT);
            } else if ((distance_right < distance_left && right_far) || state.state == ReactorState.REACTOR_FAR_RIGHT){
                state.changeState(ReactorState.REACTOR_FAR_RIGHT);
            } else if (target_x != 0 && distance < 0.05){
                state.changeState(ReactorState.REACTOR_ALIGNED);
            } else if (target_x != 0 && distance < 0.1){
                state.changeState(ReactorState.REACTOR_IMMEDIATE);
            } else if (target_x != 0) {
                state.changeState(ReactorState.REACTOR_APPROACH);
            } else {
                System.out.println("REACTOR NOT IN SIGHT");
            }
            
            if (state.state == ReactorState.REACTOR_FAR_LEFT){
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
            } else if (state.state == ReactorState.REACTOR_FAR_RIGHT){
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
            } else if (state.state == ReactorState.REACTOR_APPROACH){
                System.out.println("REACTOR_APPROACH");
                double align = pid_align.update(vision.getNextReactorX(), false)/width;
                turn = Math.min(0.2, Math.max(-0.2, -align));
                forward = 0.13;
            } else if (state.state == ReactorState.REACTOR_IMMEDIATE){
                System.out.println("REACTOR_IMMEDIATE");
                //double align = pid_align.update(vision.getNextReactorX(), false)/width;
                //turn = Math.min(0.2, Math.max(-0.2, -align));
                turn = 0;
                forward = distance*1.5;
            } else if (state.state == ReactorState.REACTOR_ALIGNED){
                System.out.println("REACTOR_ALIGNED");
                double align = pid_align.update(vision.getNextReactorX(), false)/width;
                turn = Math.min(0.2, Math.max(-0.2, -align));
                forward = 0;
                if (turn < 0.05 || true) {
                    motorL.setSpeed(0);
                    motorR.setSpeed(0);
                    comm.transmit();
                    break;
                }
            }

            System.out.println("forward: " + forward);
            System.out.println("turn: " + turn);

            motorL.setSpeed(-(forward + turn));
            motorR.setSpeed(forward - turn);
            comm.transmit();
        }
        
        motorL.setSpeed(-0.2);
        motorR.setSpeed(0.2);
        comm.transmit();
        try {
            Thread.sleep(500);
        } catch (Exception exc){
            exc.printStackTrace();
        }
        motorL.setSpeed(0);
        motorR.setSpeed(0);
        comm.transmit();
        try {
            Thread.sleep(500);
        } catch (Exception exc){
            exc.printStackTrace();
        }
        
        hopper.sorterGreen();
        try {
            Thread.sleep(2000);
        } catch (Exception exc) {
            exc.printStackTrace();
        }
        hopper.rampHigh();
        comm.transmit();
        hopper.pacmanOpen();
        try {
            Thread.sleep(2000);
        } catch (Exception exc) {
            exc.printStackTrace();
        }
        motorL.setSpeed(0.15);
        motorR.setSpeed(-0.15);
        comm.transmit();
        try {
            Thread.sleep(2000);
        } catch (Exception exc) {
            exc.printStackTrace();
        }
        
        //LOW
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

    class TestState {
        private long start_time;
        public ReactorState state;
        
        public TestState(ReactorState init){
            start_time = System.currentTimeMillis();
            state = init;
        }
        
        public void changeState(ReactorState new_state){
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
