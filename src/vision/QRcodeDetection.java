package vision;

import java.awt.image.BufferedImage;
import java.io.File;






import maslabGLvision.TestBed;

import com.google.zxing.ChecksumException;
import com.google.zxing.FormatException;
import com.google.zxing.NotFoundException;
import com.google.zxing.client.j2se.BufferedImageLuminanceSource;
import com.google.zxing.common.BitMatrix;
import com.google.zxing.common.DecoderResult;
import com.google.zxing.common.DetectorResult;
import com.google.zxing.common.HybridBinarizer;
import com.google.zxing.qrcode.decoder.Decoder;
import com.google.zxing.qrcode.detector.Detector;

public class QRcodeDetection {

	public static void main(String[] args) {

		TestBed tester = new TestBed(220,220);
		BufferedImage original = TestBed.loadImage( new File("images\\qr.png") );
		BufferedImageLuminanceSource source = new BufferedImageLuminanceSource(original);
		HybridBinarizer hb = new HybridBinarizer(source);
		
		try {
			BitMatrix image = hb.getBlackMatrix();
			Detector detector = new Detector(image);
			DetectorResult detected = detector.detect();
			Decoder decoder = new Decoder();
			DecoderResult decoded = decoder.decode(detected.getBits());
			System.out.println(decoded.getText());
		} catch (NotFoundException | ChecksumException | FormatException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		}
		
		
		
	}

}
