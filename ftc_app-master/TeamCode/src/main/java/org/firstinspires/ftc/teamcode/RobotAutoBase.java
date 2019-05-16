package org.firstinspires.ftc.teamcode;

/**
 * Created by isaac.blandin on 5/10/19.
 */

public abstract class RobotAutoBase extends Robot {

    public int getYEnc(){
        return rfDrive.getCurrentPosition();
    }

    public int getXEnc(){
        return lfDrive.getCurrentPosition();
    }

    /**
     * updates the robot's field position in feet on the x and y coordinates relative to the starting point
     */
    public void updatePos(){

        double deltaX = (getXEnc() - lastX) / ODOM_PPR * ODOM_CIRC;
        double deltaY = (getYEnc() - lastY) / ODOM_PPR * ODOM_CIRC;

        lastX = getXEnc();
        lastY = getYEnc();

        double theta = Math.toRadians(getGlobal()%360);

        xPos += deltaY * Math.cos(theta + Math.PI/4) + deltaX * Math.cos(theta);
        yPos += deltaY * Math.sin(theta + Math.PI/4) + deltaX * Math.sin(theta);

    }

    /**
     * moves the robot from the current field position to a target field coordinate in feet
     *
     * @param powerT max speed the robot can move
     * @param x target x-coordinate
     * @param y target y-coordinate
     */
    public void driveToCoord(double powerT, double x, double y){

        double power;
        double totalDist = Math.sqrt(Math.pow((x-xPos), 2) + Math.pow((y-yPos), 2));
        double distanceLeft = 1.1;

        while(distanceLeft > 1){

            distanceLeft = Math.sqrt(Math.pow((x-xPos), 2) + Math.pow((y-yPos), 2));

            double slope = (y-yPos)/(x-xPos);
            double direction = Math.atan(slope);

            if (Math.abs(totalDist - distanceLeft) < 1){
                power = 0.15 + (powerT - 0.15) * (totalDist - distanceLeft);
            } else {
                power = powerT;
            }



        }

    }
}
