package controller;

import jssc.SerialPort;

public class KitBotModel {
	private SerialPort serialPort;
    private byte motorA = 0;
    private byte motorB = 0;
    private double sonar1Distance = 0;
    private double sonar2Distance = 0;
    
	public KitBotModel() {
		try {
			serialPort = new SerialPort("COM7");
            serialPort.openPort();
            serialPort.setParams(115200, 8, 1, 0);
        }
        catch (Exception ex){
            System.out.println(ex);
        }
	}
	
	public void updateData() {
		try {
			String data = serialPort.readString();
			System.out.println(data);
			if (data != null && data.contains("Sonar1: ")) {
				System.out.println(data);
				//System.out.println(data.substring(8,data.indexOf(".")));
				sonar1Distance = Double.parseDouble(data.substring(8,data.indexOf(".")));
				//System.out.println("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX");
			}
			if (data != null && data.contains("Sonar2: ")) {
				System.out.println(data.substring(8,data.indexOf(".")));
				sonar2Distance = Double.parseDouble(data.substring(8,data.indexOf(".")));
				//System.out.println("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX");
			}
			
		} catch (Exception e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	
	public double getSonar1Distance() {
		return sonar1Distance;
	}
	
	public double getSonar2Distance() {
		return sonar2Distance;
	}
	
	public void setMotors( double powerLeft, double powerRight ) {
		motorA = (byte)(-powerRight*127);
		motorB = (byte)(-powerLeft*127);
		modified();
	}
	
	public void modified() {
		try {
			byte[] data = new byte[4];
			data[0] = 'S';		// Start signal "S"
			data[1] = motorA;	// Motor A data
			data[2] = motorB;	// Motor B data
			data[3] = 'E';		// End signal "E"
			serialPort.writeBytes(data);
		} catch ( Exception ex ) {
			System.out.println(ex);
		}
	}
	
	public void finalize() {
		try {
			serialPort.closePort();
		} catch ( Exception ex ) {
			System.out.println(ex);
		}
	}
}
