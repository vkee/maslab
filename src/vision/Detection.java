package vision;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class Detection {    
    static {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
    }
    
//    public static Pair getCoordinates(Mat srcImage, int lowerHue, int upperHue){
//        
//    }
    
    public static Mat detectHueRange(Mat srcImage, int lowerHue, int upperHue){
        
        // REPLACE WITH OPENCV METHOD OF THRESHOLDING WITH INRANGE
        //Scalar hsv_min = new Scalar(lowerHue, 100, 100, 0);
        //Scalar hsv_max = new Scalar(upperHue, 255, 255, 0);
        //Core.inRange(HSV, hsv_min, hsv_max, ThresIm);
        
        Mat HSV = new Mat();
        Imgproc.cvtColor(srcImage, HSV, Imgproc.COLOR_BGR2HSV);
        Mat ThresIm = new Mat(HSV.height(), HSV.width(), CvType.CV_8UC3);
//        Scalar hsv_min = new Scalar(lowerHue, 100, 100, 0);
//        Scalar hsv_max = new Scalar(upperHue, 255, 255, 0);
//        Core.inRange(HSV, hsv_min, hsv_max, ThresIm);
        for (int x = 0; x < HSV.height(); x++){
            for (int y = 0; y < HSV.width(); y++){
                if (((HSV.get(x, y)[0] >= 356 || HSV.get(x, y)[0] <= 4) &&
                        HSV.get(x,y)[1] <= 255 && HSV.get(x,y)[1] >= 180 &&
                        HSV.get(x,y)[2] <= 255 && HSV.get(x, y)[2] >= 10)){
//                        || ((HSV.get(x, y)[0] >= 80 && HSV.get(x, y)[0] <= 160) &&
//                        HSV.get(x, y)[1] <= 255 && HSV.get(x, y)[1] >= 10 &&
//                        HSV.get(x, y)[2] <= 255 && HSV.get(x, y)[2] >= 150)){
                    ThresIm.put(x, y, new byte[]{ (byte) 255, (byte) 255, (byte) 255});
                }
            }
        }
        Imgproc.medianBlur(ThresIm, ThresIm, 13);
        return ThresIm;
    }
}

