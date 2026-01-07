package org.firstinspires.ftc.teamcode.pedroPathing;

public class Constants_number {
    public class Pair{
        double x;
        double y;
        public Pair(double x, double y){
            this.x = x;
            this.y = y;
        }
    }
    public Pair BLUE_GOAL = new Pair(12.727, 135.751), RED_GOAL = new Pair(132.688, 137.165);
    public Pair BLUE_BOX = new Pair(105.349, 32.995), RED_BOX = new Pair(38.652, 33.230);
    public double get_angle_to_blue(double x, double y, double angle) {
        return angle - Math.atan2(BLUE_GOAL.y - y, BLUE_GOAL.x - x);
    }


}
