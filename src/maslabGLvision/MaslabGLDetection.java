package maslabGLvision;

import java.awt.image.BufferedImage;
import java.io.File;

import Core.Engine;
import Core.FilterOp;

public class MaslabGLDetection {
	
	public BufferedImage filterImage(BufferedImage original) {
		TestBed tester = new TestBed(320,240);

		Engine.initGL(320,240);
		FilterOp blur = new FilterOp("blur");
		FilterOp colorize = new FilterOp("colorize");
		FilterOp objRec = new FilterOp("objectRecognition");

		original = TestBed.loadImage( new File("images\\maslab.png") );
		tester.setImage(original);

		BufferedImage filtered;
		
		filtered = blur.apply(original);
		filtered = colorize.apply(filtered);
		filtered = objRec.apply(filtered);
		
		tester.setImage(filtered);
		
		return filtered;
	}
}
