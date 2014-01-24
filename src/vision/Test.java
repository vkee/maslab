package vision;

import java.awt.image.BufferedImage;
import java.io.File;

import maslabGLvision.TestBed;

public class Test {

	public static void main(String[] args) {
		BufferedImage original = TestBed.loadImage( new File("images\\colourSensorTest.png") );
		int center = original.getRGB(16, 12);
        int red = (center >> 16) & 0xFF;
        int green = (center >> 8) & 0xFF;
        int blue = (center >> 0) & 0xFF;
        System.out.println(red);
        System.out.println(green);
        System.out.println(blue);
        if (red > 1.7*blue && red > 1.7*green) {
        	red = 1;
        	green = 0;
        	System.out.println("RED");
        } else if(green > 1.7*blue && green > 1.7*red) {
        	red = 0;
        	green = 1;
        	System.out.println("GREEN");
        } else {
        	red = 0;
        	green = 0;
        	System.out.println("NOTHING");
        }
	}

}
