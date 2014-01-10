package vision;

import java.util.ArrayList;
import java.util.List;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

public class Detection {    
    static {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
    }
    
    public static void drawLines(Mat binary, List<org.opencv.core.Point> lines){
        for (org.opencv.core.Point line : lines){
            double rho = line.x;
            double theta = line.y;
            double m = -Math.cos(theta)/Math.sin(theta);
            double b = rho/Math.sin(theta);
            for (int col = 0; col < binary.width(); col++){
                int row = (int) (m*col + b);
                if (row >= 0 && row < binary.height()){
                    binary.put(row, col, new byte[]{(byte) 255});
                }
            }
        }
    }
    
    public static List<org.opencv.core.Point> findWallEdges(Mat lines, Mat srcImage, int threshold){
        Mat HSV = new Mat();
        Mat blueImage = new Mat();
        Imgproc.cvtColor(srcImage, HSV, Imgproc.COLOR_BGR2HSV);
        Scalar hsv_min = new Scalar(0, 0, 10, 0);
        Scalar hsv_max = new Scalar(360, 255, 255, 0);
        Core.inRange(HSV, hsv_min, hsv_max, blueImage);
        Imgproc.medianBlur(blueImage, blueImage, 13);
        List<org.opencv.core.Point> wallEdges = new ArrayList<org.opencv.core.Point>();
        for (int y = 0; y < lines.width(); y++){
            double rho = lines.get(0, y)[0];
            double theta = lines.get(0, y)[1];
            double m = -Math.cos(theta)/Math.sin(theta);
            double b = rho/Math.sin(theta);
            int count = 0;
            for (int col = 0; col < blueImage.width(); col++){
                int row = (int) (m*col + b);
                if (blueNeighbors(row, col, blueImage)){
                    count += 1;
                }
            }
            if (count >= threshold){
                wallEdges.add(new org.opencv.core.Point(rho, theta));
            }
        }
        return wallEdges;
    }
    
    public static boolean blueNeighbors(int row, int col, Mat blueImage){
        for (int x = -1; x < 2; x++){
            for (int y = -1; y < 2; y++){
                int test_row = Math.min(Math.max(row + x, 0), blueImage.height() - 1);
                int test_col = Math.min(Math.max(col + y, 0), blueImage.width() - 1);
                if (blueImage.get(test_row, test_col)[0] > 0){
                    return true;
                }
            }
        }
        return false;
    }
    
    // find wall representation
    public static Mat detectEdges(Mat srcImage, double lowThres, double ratio){
        Mat src_gray = new Mat();
        Mat edges = new Mat();
        Mat lines = new Mat();
        Imgproc.cvtColor(srcImage, src_gray, Imgproc.COLOR_BGR2GRAY);
        Imgproc.blur(src_gray, src_gray, new Size(3, 3));
        Imgproc.Canny(src_gray, edges, lowThres, ratio*lowThres);
        Imgproc.HoughLinesP(edges, lines, 1, Math.PI/180.0, 50, 50, 10);
        return lines;
    }
    
 // find wall representation
    public static Mat contourImage(Mat srcImage, double lowThres, double ratio){
        Mat src_gray = new Mat();
        Mat edges = new Mat();
        Imgproc.cvtColor(srcImage, src_gray, Imgproc.COLOR_BGR2GRAY);
        //Imgproc.blur(src_gray, src_gray, new Size(3, 3));
        Imgproc.Canny(src_gray, edges, lowThres, ratio*lowThres);
        return edges;
    }
    
    // make sure it is a ball first
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
                // check if second condition works or is even necessary
                if (cost < centerCost && validBall(center, boundaryPoints, 10000)){
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
    
    public static boolean validBall(org.opencv.core.Point center, List<org.opencv.core.Point> boundaryPoints, double threshold){
        double linSum = 0;
        double sqrSum = 0;
        int n = boundaryPoints.size();
        for (org.opencv.core.Point point : boundaryPoints){
            linSum += (center.x - point.x)*(center.x - point.x) + (center.y - point.y)*(center.y - point.y);
            sqrSum += ((center.x - point.x)*(center.x - point.x) + (center.y - point.y)*(center.y - point.y))
                    *((center.x - point.x)*(center.x - point.x) + (center.y - point.y)*(center.y - point.y));
        }
        double var = (sqrSum/n) - (linSum/n)*(linSum/n);
        return (1000000*var/(n*n*n*n) <= threshold);
    }
    
    // May want to incorporate a second edge-based detection for more reliability
    // may want to ensure use three channel image
    public static Mat detectHueRange(Mat srcImage){
        Mat HSV = new Mat();
        Imgproc.cvtColor(srcImage, HSV, Imgproc.COLOR_BGR2HSV);
        Mat ThresIm_1 = new Mat(HSV.height(), HSV.width(), CvType.CV_8UC1);
        Mat ThresIm_2 = new Mat(HSV.height(), HSV.width(), CvType.CV_8UC1);
        Mat ThresIm_3 = new Mat(HSV.height(), HSV.width(), CvType.CV_8UC1);
        Mat ThresIm = new Mat(HSV.height(), HSV.width(), CvType.CV_8UC1);
        
        // try to switch to three channel images
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

