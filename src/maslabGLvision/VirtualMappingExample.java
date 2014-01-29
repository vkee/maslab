package maslabGLvision;

import java.awt.image.BufferedImage;
import java.io.File;

import Core.Engine;
import Core.FilterOp;

public class VirtualMappingExample {
	public static void main( String[] args ) {
		TestBed tester = new TestBed(500,500);
		
		Engine.initGL(500,500);
		FilterOp blur = new FilterOp("blur");
		FilterOp modColorize = new FilterOp("modColorize");
		FilterOp eliminateTop = new FilterOp("eliminateTop");
		FilterOp eliminateBottom = new FilterOp("eliminateBottom");
		FilterOp modObjRec = new FilterOp("modObjRec");
		FilterOp centreRec = new FilterOp("centreRecognition");
		FilterOp edge = new FilterOp("edge");
		FilterOp wallDetection = new FilterOp("wallDetection");
		FilterOp wallFilter = new FilterOp("wallFilter");
		FilterOp wallFilterRefine = new FilterOp("wallFilterRefine");
		
		BufferedImage original = TestBed.loadImage( new File("images\\reactor1.jpg") );
		//BufferedImage original = TestBed.loadImage( new File("images\\tealband.png") );
		tester.setImage(original);
		
		int frames = 0;
		long prev = System.currentTimeMillis();
		while ( true ) {
			blur.apply(original);
			modColorize.apply();
			eliminateTop.apply();
			eliminateBottom.apply();
			modObjRec.apply();
			//centreRec.apply();
//		    wallDetection.apply(original);
//		    wallFilter.apply();
//		    wallFilterRefine.apply();
			BufferedImage filtered = FilterOp.getImage(); 
			tester.setImage(filtered);
//			int centre = filtered.getRGB(0, 0);
//			int red = (centre >> 16) & 0xFF;
//            int green = (centre >> 8) & 0xFF;
//            int blue = (centre >> 0) & 0xFF;
//            System.out.println(red);
//            System.out.println(green);
//            System.out.println(blue);
			frames++;
			long curr = System.currentTimeMillis();
			if ( curr-prev>1000 ) {
				System.out.println("FPS: " +frames);
				frames = 0;
				prev = curr;
			}
		}
	}
}
