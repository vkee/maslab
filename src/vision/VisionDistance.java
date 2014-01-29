package vision;

import java.awt.image.BufferedImage;
import java.io.File;

import org.opencv.core.Core;
import org.opencv.highgui.VideoCapture;

import maslabGLvision.TestBed;
import Core.Engine;
import Core.FilterOp;

public class VisionDistance {

	public static void main(String[] Args) {
		// Load the OpenCV library
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);

        // Setup the camera
        VideoCapture camera = new VideoCapture();
        camera.open(1);
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
			//blur.apply(original);
			modColorize.apply(original);
			eliminateTop.apply();
			eliminateBottom.apply();
			modObjRec.apply();
			//centreRec.apply();
//		    wallDetection.apply(original);
//		    wallFilter.apply();
//		    wallFilterRefine.apply();
			BufferedImage filtered = FilterOp.getImage(); 
			tester.setImage(filtered);
			
			int center, red, green, blue;
			int width = 0;
			for (int y = 0; y <= 240; y++) {
				center = filtered.getRGB(160, y);
				red = (center >> 16) & 0xFF;
                green = (center >> 8) & 0xFF;
                blue = (center >> 0) & 0xFF;
                
                if (blue > 0) {
                	width = blue;
                	break;
                }
			}
			System.out.println(340.0/width);
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
