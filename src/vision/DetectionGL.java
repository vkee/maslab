package vision;

import java.awt.Point;
import java.awt.image.BufferedImage;
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

import Core.FilterOp;

public class DetectionGL {
    public static Point nextCenter(BufferedImage centerImage, int width, int height){
        double centerX = width/2.0;
        double centerY = height/2.0;
        int nextCenterX = 0;
        int nextCenterY = 0;
        double centerCost = centerX*centerX + centerY*centerY;
        for (int x = 0; x < centerImage.getWidth(); x++){
            for (int y = 0; y < centerImage.getHeight(); y++){
                int pixel = centerImage.getRGB(x, y);
                int red = (pixel >> 16) & 0xFF;
                int green = (pixel >> 8) & 0xFF;
                if (red > 0 && green > 0 && (centerX - x)*(centerX - x) +
                        (centerY - y)*(centerY - y) < centerCost){
                    nextCenterX = x;
                    nextCenterY = y;
                }
            }
        }
        return new Point(nextCenterX, nextCenterY);
    }
    
    public static List<Point> centers(BufferedImage centerImage){
        List<Point> centers = new ArrayList<Point>();
        for (int x = 0; x < centerImage.getWidth(); x++){
            for (int y = 0; y < centerImage.getHeight(); y++){
                int pixel = centerImage.getRGB(x, y);
                int red = (pixel >> 16) & 0xFF;
                if (red > 0){
                    centers.add(new Point(x, y));
                }
            }
        }
        return centers;
    }
}

