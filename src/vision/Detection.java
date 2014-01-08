package vision;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

public class Detection {    
    static {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
    }
    
    public static Pair nextCenter(Mat processedImage){
        Moments result = Imgproc.moments(processedImage);
        double momX10 = result.get_m10();
        double momY01 = result.get_m01();
        double area = result.get_mu20();
        int posX = (int) (momX10/area);
        int posY = (int) (momY01/area);
        return new Pair(posX, posY);
    }
    
    public static Mat detectHueRange(Mat srcImage){
        
        // REPLACE WITH OPENCV METHOD OF THRESHOLDING WITH INRANGE
        //Scalar hsv_min = new Scalar(lowerHue, 100, 100, 0);
        //Scalar hsv_max = new Scalar(upperHue, 255, 255, 0);
        //Core.inRange(HSV, hsv_min, hsv_max, ThresIm);
        
        Mat HSV = new Mat();
        Imgproc.cvtColor(srcImage, HSV, Imgproc.COLOR_BGR2HSV);
        Mat ThresIm = new Mat(HSV.height(), HSV.width(), CvType.CV_8UC1);
        //Scalar hsv_min = new Scalar(lowerHue, 100, 100, 0);
        //Scalar hsv_max = new Scalar(upperHue, 255, 255, 0);
        //Core.inRange(HSV, hsv_min, hsv_max, ThresIm);
        for (int x = 0; x < HSV.height(); x++){
            for (int y = 0; y < HSV.width(); y++){
                if (((HSV.get(x, y)[0] >= 356 || HSV.get(x, y)[0] <= 4) &&
                        HSV.get(x,y)[1] <= 255 && HSV.get(x,y)[1] >= 180 &&
                        HSV.get(x,y)[2] <= 255 && HSV.get(x, y)[2] >= 10)){
                    //|| ((HSV.get(x, y)[0] >= 80 && HSV.get(x, y)[0] <= 160) &&
                    //HSV.get(x, y)[1] <= 255 && HSV.get(x, y)[1] >= 10 &&
                    //HSV.get(x, y)[2] <= 255 && HSV.get(x, y)[2] >= 150)){
                    ThresIm.put(x, y, new byte[]{ (byte) 255 });
                }
            }
        }
        Imgproc.medianBlur(ThresIm, ThresIm, 13);
        return ThresIm;
    }
}

