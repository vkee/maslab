package vision;

import java.awt.image.BufferedImage;

import comm.BotClientMap.Point;

public class Map {
    /*
     * NOTES:
     * Have vision class process necessary information then pass it to an
     * instance of this class which is initialized with input map. Find
     * way to reliably aim the camera directly ahead.
     */
    
    // CONSTANTS
    private final double CAMERA_DIST = 50;
    private final double CAMERA_HEIGHT = 4;
    private final double WALL_BLUE_HEIGHT = 2;
    private final double WALL_WHITE_HEIGHT = 4;
    public final double BBOX_X = 320; // temporarily set to all
    public final double BBOX_Y = 240; // temporarily set to all
    private final int WIDTH;
    private final int HEIGHT;
    
    // FIELDS
    public BufferedImage field;
    
    public Map(int width, int height, BufferedImage init){
        WIDTH = width;
        HEIGHT = height;
        field = init;
        resetField();
    }
    
//    public void calibrateMapCalculations(BufferedImage colorized){
//        int pixel, blue;
//        double ratio_1, ratio_2;
//        int num = 0;
//        double exp_lin = 0;
//        double exp_sq = 0;
//        for (int x = 0; x < colorized.getWidth(); x++){
//            for (int y = 1; y < colorized.getHeight(); y++){
//                pixel = colorized.getRGB(x, y);
//                blue = (pixel) & 0xFF;
//                if (blue > 0 && ((colorized.getRGB(x, y-1)) & 0xFF) == 0){
//                    ratio_1 = Math.abs(WALL_BLUE_HEIGHT/blue);
//                    ratio_2 = Math.abs(WALL_WHITE_HEIGHT - CAMERA_HEIGHT)/(y - (HEIGHT/2));
//                    num++;
//                    exp_lin += (ratio_1/ratio_2);
//                    exp_sq += (ratio_1/ratio_2)*(ratio_1/ratio_2);
//                }
//            }
//        }
//        exp_lin = exp_lin/num;
//        exp_sq = exp_sq/num;
//        double var = exp_sq - (exp_lin*exp_lin);
//        System.out.println("Expected ratio: " + exp_lin);
//        System.out.println("Variance: " + var);
//    }
    
    public void resetField(){
        for (int x = 0; x < WIDTH; x++){
            for (int y = 0; y < HEIGHT; y ++){
                field.setRGB(x, y, 0);
            }
        }
    }
    
    public void updateDisplay(double x, double y){
        int plot_x = (int) (WIDTH/2 + x);
        int plot_y = (int) (HEIGHT/2 + y);
        int white = (int) 0xFFFFFFFF;
        field.setRGB(plot_x, plot_y, white);
        System.out.println("Plotting: (" + plot_x + ", " + plot_y + ")");
    }
    
    public Point convertWallCoordinates(int x, int y, double height) throws RuntimeException {
        double ratio_1, ratio_2, ratio;
        ratio_1 = Math.abs(WALL_BLUE_HEIGHT/height);
        //ratio_2 = Math.abs((WALL_WHITE_HEIGHT - CAMERA_HEIGHT)/(y - (HEIGHT/2)));
        
//        if (ratio_1/ratio_2 > 1.25 || ratio_1/ratio_2 < 0.8){
//            throw new RuntimeException("Incorrect ratio values");
//        }
        
        //ratio = 0.5*(ratio_1 + ratio_2);
        ratio = ratio_1;
        updateDisplay(CAMERA_DIST*ratio, y*ratio);
        return new Point(CAMERA_DIST*ratio, y*ratio);
    }
}
