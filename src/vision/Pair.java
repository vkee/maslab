package vision;
public class Pair {
    private int x;
    private int y;
    private double radius;
    
    public Pair(int x, int y){
        this.x = x;
        this.y = y;
        this.radius = 0;
    }
    
    public Pair(int x, int y, int radius){
        this.x = x;
        this.y = y;
        this.radius = radius;
    }
    
    public double getAngle(){
        return 0;
    }
    
    public double getDistance(){
        return 0;
    }
    
    public int getX(){
        return x;
    }
    
    public int getY(){
        return y;
    }
}
