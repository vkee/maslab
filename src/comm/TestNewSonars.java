package comm;

import java.util.LinkedList;
import java.util.List;

import comm.MapleComm;
import comm.MapleIO;
import devices.actuators.Cytron;
import devices.actuators.DigitalOutput;
import devices.actuators.PWMOutput;
import devices.sensors.DigitalInput;
import devices.sensors.Encoder;
import devices.sensors.Gyroscope;
import devices.sensors.Infrared;
import devices.sensors.Ultrasonic;

public class TestNewSonars {

    public static void main(String[] args) {
        MapleComm comm = new MapleComm(MapleIO.SerialPortType.WINDOWS);
        Cytron motorL = new Cytron(4, 0);
        Cytron motorR = new Cytron(3, 1);
        
        Ultrasonic sonarA = new Ultrasonic(26, 25);
        Ultrasonic sonarB = new Ultrasonic(34, 33);
        Ultrasonic sonarC = new Ultrasonic(35, 36);
        Ultrasonic sonarD = new Ultrasonic(30, 29);
        Ultrasonic sonarE = new Ultrasonic(32, 31);
        
        DigitalOutput roller = new DigitalOutput(12);
        
        DigitalInput infrared = new DigitalInput(10);

        Encoder encoderL = new Encoder(5, 7);
        Encoder encoderR = new Encoder(6, 8);
        
        DigitalOutput sonarPower = new DigitalOutput(37); // orig 37
        
        comm.registerDevice(motorL);
        comm.registerDevice(motorR);
        comm.registerDevice(sonarA);
        comm.registerDevice(sonarB);
        comm.registerDevice(sonarC);
        comm.registerDevice(sonarD);
        comm.registerDevice(sonarE);
        comm.registerDevice(sonarPower);
        //comm.registerDevice(roller);
        comm.registerDevice(infrared);

        comm.registerDevice(encoderL);
        comm.registerDevice(encoderR);
        
        comm.initialize();

        System.out.println("Now I'm here");
//        roller.setValue(false);
//        try {
//            Thread.sleep(1000);
//        } catch (InterruptedException e) {
//            e.printStackTrace();
//        }
//        roller.setValue(true);
//        try {
//            Thread.sleep(1000);
//        } catch (InterruptedException e) {
//            e.printStackTrace();
//        }
        
        sonarPower.setValue(false);
        
        System.out.println("Here");


        comm.transmit();

        comm.updateSensorData();

//      double time = System.currentTimeMillis();
//      double prevTime = System.currentTimeMillis();
//      double gyroError = 0;
//      while (System.currentTimeMillis() - time < 5000) {
//          comm.updateSensorData();
//          gyroError += gyro.getOmega() * (System.currentTimeMillis() - prevTime)/5000;
//          prevTime = System.currentTimeMillis();
//      }
        
        boolean ball_present = true;
        
        double distanceD = sonarD.getDistance();
        double distanceE = sonarE.getDistance();
        double distanceA = sonarA.getDistance();
        double distanceB = sonarB.getDistance();
        double distanceC = sonarC.getDistance();

        //double angle;
        
        while (true) {
            comm.updateSensorData();
            
            ball_present = infrared.getValue();
            
            distanceD = sonarD.getDistance();
            distanceE = sonarE.getDistance();
            distanceA = sonarA.getDistance();
            distanceB = sonarB.getDistance();
            distanceC = sonarC.getDistance();
            
            System.out.println("DistanceD: " + distanceD);
            System.out.println("DistanceE: " + distanceE);
            System.out.println("DistanceA: " + distanceA);
            System.out.println("DistanceB: " + distanceB);
            System.out.println("DistanceC: " + distanceC);
            
            System.out.println("IR Reading: " + ball_present);
            
            //motorL.setSpeed(-0.1);
            //motorR.setSpeed(0.1);
            
//          angle = gyro.getOmega() - gyroError;
//          System.out.println(angle);
            
            comm.transmit();
            
            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}
