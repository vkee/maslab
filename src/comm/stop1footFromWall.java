package comm;

import controller.PID;
import devices.actuators.Cytron;
import devices.sensors.Encoder;
import devices.sensors.Gyroscope;
import devices.sensors.Ultrasonic;

public class stop1footFromWall {
    public static void main(String[] args) {
        new stop1footFromWall();
    }
    
    public stop1footFromWall() {
        MapleComm comm = new MapleComm(MapleIO.SerialPortType.WINDOWS);
        Gyroscope gyro = new Gyroscope(1, 8);
        Ultrasonic sonar = new Ultrasonic(32, 31);
        Cytron motor1 = new Cytron(2, 1);
        Cytron motor2 = new Cytron(7, 6);
        
        comm.registerDevice(encoder);
        comm.registerDevice(gyro);
        comm.registerDevice(sonar);
        comm.registerDevice(motor1);
        comm.registerDevice(motor2);
        
        System.out.println("Initializing");
        comm.initialize();
        comm.updateSensorData();
        
        PID pid_forward = new PID(0.3, 0.05, 0.0, 0.0);
        
        double forward = Math.max(-0.1, Math.min(0.1, pid_forward.update(sonar.getDistance(), true)));
        
        PID pid_turn = new PID(0,0.03,0.0,0.0);
        
        double turn = Math.max(-0.04, Math.min(0.04,pid_turn.update(gyro.getOmega(), true)));
        
        motor1.setSpeed(forward + turn);
        motor2.setSpeed(forward - turn);
        comm.transmit();
        
        while (true) {
            comm.updateSensorData();
            
            forward = Math.max(-0.1, Math.min(0.1, pid_forward.update(sonar.getDistance(), false)));
            turn = Math.max(-0.04, Math.min(0.04,pid_turn.update(gyro.getOmega(), false)));
            
            motor1.setSpeed(forward + turn);
            motor2.setSpeed(forward - turn);
            comm.transmit();
            
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}
