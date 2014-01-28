package maslabGLvision;

import java.awt.image.BufferedImage;
import java.io.File;

import Core.Engine;
import Core.FilterOp;

public class ColorTestingVision {
	public static void main( String[] args ) {
		TestBed tester = new TestBed(600,450);
		
		Engine.initGL(600, 450);
		FilterOp blur = new FilterOp("blur");
		FilterOp op = new FilterOp("colorTesting");
		
		BufferedImage original = TestBed.loadImage( new File("images\\spectrum.png") );
		tester.setImage(original);
		
		try{ Thread.sleep(1000); } catch ( Exception e ) {}
		
		blur.apply(original);
		op.apply();
		BufferedImage filtered = FilterOp.getImage(); 
		tester.setImage(filtered);
	}
}
