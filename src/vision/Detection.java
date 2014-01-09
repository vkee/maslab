package vision;

import java.awt.Point;
import java.util.ArrayList;
import java.util.List;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

public class Detection {    
    static {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
    }

    // findContours, threshold, split, find centers using contours
    public static org.opencv.core.Point nextCenter(Mat processedImage, int centerX, int centerY, int threshold){
        double nextCenterX = 0;
        double nextCenterY = 0;
        double centerCost = centerX*centerX + centerY*centerY;
        List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(processedImage, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);
        for (MatOfPoint boundary : contours){
            List<org.opencv.core.Point> boundaryPoints = boundary.toList();
            if (boundaryPoints.size() >= threshold){
                org.opencv.core.Point center = findCenter(boundaryPoints);
                double cost = (centerX - center.x)*(centerX - center.x) + (centerY - center.y)*(centerY - center.y);
                if (cost < centerCost){
                    centerCost = cost;
                    nextCenterX = center.x;
                    nextCenterY = center.y;
                }
            }
        }
        return new org.opencv.core.Point(nextCenterX, nextCenterY);
    }
    
    public static org.opencv.core.Point findCenter(List<org.opencv.core.Point> boundaryPoints){
        double sumX = 0;
        double sumY = 0;
        for (org.opencv.core.Point point : boundaryPoints){
            sumX += point.x;
            sumY += point.y;
        }
        double avgX = sumX/boundaryPoints.size();
        double avgY = sumY/boundaryPoints.size();
        return new org.opencv.core.Point(avgX, avgY);
    }
    
    // May want to incorporate a second edge-based detection for more reliability
    public static Mat detectHueRange(Mat srcImage){
        Mat HSV = new Mat();
        Imgproc.cvtColor(srcImage, HSV, Imgproc.COLOR_BGR2HSV);
        Mat ThresIm_1 = new Mat(HSV.height(), HSV.width(), CvType.CV_8UC1);
        Mat ThresIm_2 = new Mat(HSV.height(), HSV.width(), CvType.CV_8UC1);
        Mat ThresIm_3 = new Mat(HSV.height(), HSV.width(), CvType.CV_8UC1);
        Mat ThresIm = new Mat(HSV.height(), HSV.width(), CvType.CV_8UC1);
        Scalar hsv_min_1 = new Scalar(0, 170, 10, 0);
        Scalar hsv_max_1 = new Scalar(10, 255, 255, 0);
        Scalar hsv_min_2 = new Scalar(350, 170, 10, 0);
        Scalar hsv_max_2 = new Scalar(360, 255, 255, 0);
        Scalar hsv_min_3 = new Scalar(50, 100, 30, 0);
        Scalar hsv_max_3 = new Scalar(80, 255, 255, 0);
        Core.inRange(HSV, hsv_min_1, hsv_max_1, ThresIm_1);
        Core.inRange(HSV, hsv_min_2, hsv_max_2, ThresIm_2);
        Core.inRange(HSV, hsv_min_3, hsv_max_3, ThresIm_3);
        Core.bitwise_or(ThresIm_1, ThresIm_2, ThresIm);
        Imgproc.medianBlur(ThresIm, ThresIm, 13);
        Imgproc.medianBlur(ThresIm_3, ThresIm_3, 13);
        Core.bitwise_or(ThresIm, ThresIm_3, ThresIm);      
        return ThresIm;
    }
    
    public static Mat detectHueRange2(Mat srcImage){
        Mat HSV = new Mat();
        Imgproc.cvtColor(srcImage, HSV, Imgproc.COLOR_BGR2HSV);
        Mat ThresIm = new Mat(HSV.height(), HSV.width(), CvType.CV_8UC1);
        for (int x = 0; x < HSV.height(); x++){
            for (int y = 0; y < HSV.width(); y++){
                if (((HSV.get(x, y)[0] >= 356 || HSV.get(x, y)[0] <= 4) &&
                        HSV.get(x,y)[1] <= 255 && HSV.get(x,y)[1] >= 180 &&
                        HSV.get(x,y)[2] <= 255 && HSV.get(x, y)[2] >= 10)){
                    ThresIm.put(x, y, new byte[]{ (byte) 255 });
                }
            }
        }
        Imgproc.medianBlur(ThresIm, ThresIm, 13);
        return ThresIm;
    }
    
    public static Mat convertC(Mat srcImage){
        Mat dstImage = new Mat(srcImage.height(), srcImage.width(), CvType.CV_8UC3);
        for (int x = 0; x < srcImage.height(); x++){
            for (int y = 0; y < srcImage.width(); y++){
                if (srcImage.get(x, y)[0] > 0){
                    dstImage.put(x, y, new byte[]{(byte) 255, (byte) 255, (byte) 255});
                } else{
                    dstImage.put(x, y, new byte[]{(byte) 0, (byte) 0, (byte) 0});
                }
            }
        }
        return dstImage;
    }
}

