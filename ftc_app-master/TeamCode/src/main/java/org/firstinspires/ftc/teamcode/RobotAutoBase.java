package org.firstinspires.ftc.teamcode;

/**
 * Created by isaac.blandin on 5/10/19.
 */

public abstract class RobotAutoBase extends Robot {

    //pulls value from forward odometry wheel
    public int getYEnc(){
        return rfDrive.getCurrentPosition();
    }

    //pulls value from lateral odometry wheel
    public int getXEnc(){
        return lfDrive.getCurrentPosition();
    }

    /**
     * updates the robot's field position in feet on the x and y coordinates relative to the starting point
     */
    public void updatePos(){

        //calculate change in x and y encoder values and convert to inches traveled
        double deltaX = (getXEnc() - lastX) / ODOM_PPR * ODOM_CIRC;
        double deltaY = (getYEnc() - lastY) / ODOM_PPR * ODOM_CIRC;

        //set current position to last position
        lastX = getXEnc();
        lastY = getYEnc();

        //gets current gyro heading and converts to radians
        double theta = Math.toRadians(getGlobal()%360);

        //adjusts distance traveled by the angle of the robot, so orientation does not matter
        xPos += deltaY * Math.cos(theta + Math.PI/4) + deltaX * Math.cos(theta);
        yPos += deltaY * Math.sin(theta + Math.PI/4) + deltaX * Math.sin(theta);

    }

    /**
     * moves the robot from the current field position to a target field coordinate in feet
     *
     * @param powerT max speed the robot can move
     * @param x target field x-coordinate
     * @param y target field y-coordinate
     */
    public void driveToCoord(double powerT, double x, double y){

        double power;
        double totalDist = Math.sqrt(Math.pow((x-xPos), 2) + Math.pow((y-yPos), 2));
        double distanceLeft = 1.1;

        while(distanceLeft > 0.2){

            //update the robot's field coordinate
            updatePos();
            distanceLeft = Math.sqrt(Math.pow((x-xPos), 2) + Math.pow((y-yPos), 2));

            //find direct angle to target coordinate
            double slope = (y-yPos)/(x-xPos);
            double direction = Math.atan2(x-xPos, y-yPos);
            //get radian value of the robots angle
            double current = Math.toRadians(getGlobal() % 360);

            //handle acceleration and deceleration for first and last foot of travel
            if (Math.abs(totalDist - distanceLeft) < 1){
                power = 0.15 + (powerT - 0.15) * (totalDist - distanceLeft);
            } else if (distanceLeft < 1){
                power = 0.15 + (powerT - 0.15) * distanceLeft;
            }else {
                power = powerT;
            }

            //apply values to vector-based holonomic drive
            drive(direction + current, power, 0);

        }

    }
}
