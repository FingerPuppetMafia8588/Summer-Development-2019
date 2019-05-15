package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cDevice;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Created by isaac.blandin on 5/7/19.
 */

public abstract class Robot extends RobotBase{

    //declares drive motors
    protected DcMotor rfDrive;
    protected DcMotor lfDrive;
    protected DcMotor rbDrive;
    protected DcMotor lbDrive;

    // declares gyro and gyro variables
    protected BNO055IMU imu;
    protected Orientation lastAngles = new Orientation();
    protected Orientation angles;
    protected double globalAngle;
    protected int heading;




    //final variables for moving robot to distance
    protected final double WHEEL_DIAMTER = 4;
    protected final double WHEEL_CIRC = WHEEL_DIAMTER*Math.PI;
    protected final double ORBITAL20_PPR = 537.6;
    protected final double DRIVE_GEAR_RATIO = 1;

    protected final double TURN_RATIO = 0.7;

    //field positioning variables

    protected double x = 0;
    protected double y = 0;

    protected double lastX = 0;
    protected double lastY = 0;

    protected final double ODOM_DIAMATER = 2;
    protected final double ODOM_CIRC = ODOM_DIAMATER*Math.PI;

    protected final double ODOM_PPR = 360;
    /**
     * sets all of the initialization values of the robot
     *
     * @param robotRunType tells whether the opmode is TeleOp or Autonomous in order to change needed
     *                     functions such as servo positions
     */
    public void initRobot (RobotRunType robotRunType){

        // set up drive motors
        rfDrive = hardwareMap.dcMotor.get("right_front");
        lfDrive = hardwareMap.dcMotor.get("left_front");
        rbDrive = hardwareMap.dcMotor.get("right_back");
        lbDrive = hardwareMap.dcMotor.get("left_back");

        lfDrive.setDirection(DcMotor.Direction.REVERSE);
        lbDrive.setDirection(DcMotor.Direction.REVERSE);

        rfDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lfDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rbDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lbDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rfDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lfDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rbDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lbDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //initialize gyro
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        //post to telemetry when gyro is calibrating
        telemetry.addData("Mode", "Calibrating");
        telemetry.update();

        //post to telemetry when gyro is calibrated
        while (!isStopRequested() && !imu.isGyroCalibrated()){
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calibration", imu.getCalibrationStatus().toString());
        telemetry.update();

        if (robotRunType == RobotRunType.AUTONOMOUS){


        }

    }

    protected static class Wheels {
        public double lf, lr, rf, rr;

        public Wheels(double lf, double rf, double lr, double rr) {
            this.lf = lf;
            this.rf = rf;
            this.lr = lr;
            this.rr = rr;
        }
    }

    /**
     * method used to drive a holonomic drive train given a vector for direction and power
     *
     * @param direction angular vector in radians which gives the direction to move
     * @param velocity speed in which the robot should be moving
     * @param rotationVelocity speed which the robot should be rotating
     * @return Wheels - creates new Wheels class
     */
    protected Wheels getWheels(double direction, double velocity, double rotationVelocity) {
        final double vd = velocity;
        final double td = direction;
        final double vt = rotationVelocity;

        double s = Math.sin(td + Math.PI / 4.0);
        double c = Math.cos(td + Math.PI / 4.0);
        double m = Math.max(Math.abs(s), Math.abs(c));
        s /= m;
        c /= m;

        final double v1 = vd * s + vt;
        final double v2 = vd * c - vt;
        final double v3 = vd * c + vt;
        final double v4 = vd * s - vt;

        // Ensure that none of the values go over 1.0. If none of the provided values are
        // over 1.0, just scale by 1.0 and keep all values.
        double scale = ma(1.0, v1, v2, v3, v4);

        return new Wheels(v1 / scale, v2 / scale, v3 / scale, v4 / scale);
    }

    /**
     * method to scale values to within a certain range
     *
     * @param xs values to be used to create the scale
     * @return double - scale to be used by the upper shell
     */
    protected static double ma(double... xs) {
        double ret = 0.0;
        for (double x : xs) {
            ret = Math.max(ret, Math.abs(x));
        }

        return ret;
    }

    /**
     * Uses the wheels classes to apply the vector driven holonomic formula to the four wheels of a
     * holonomic drivetrain
     *
     * @param direction angular vector in radians which gives the direction to move
     * @param velocity speed in which the robot should be moving
     * @param rotationVelocity speed which the robot should be rotating
     */
    protected void drive(double direction, double velocity, double rotationVelocity) {
        Wheels w = getWheels(direction, velocity, rotationVelocity);
        lfDrive.setPower(w.lf);
        rfDrive.setPower(w.rf);
        lbDrive.setPower(w.lr);
        rbDrive.setPower(w.rr);
    }

    /**
     * method to drive the robot using field-centric drive
     *
     * movement - left joystick
     * rotation - right joystick x-axis
     * slow-mode - right bumper
     */
    protected void FieldCentricDrive(){

        double turnRatio = 0.7;
        if (gamepad1.right_bumper){
            turnRatio = 0.3;
        }

        //pull coordinate values from x and y axis of joystick
        double x1 = gamepad1.left_stick_x, y1 = -gamepad1.left_stick_y;
        //create polar vector of given the joystick values
        double v = Math.sqrt(x1 * x1 + y1 * y1);
        double theta = Math.atan2(x1, y1);
        //get radian value of the robots angle
        double current = Math.toRadians(getGlobal() % 360);
        //apply values to vector-based holonomic drive
        drive(theta + current, v, gamepad1.right_stick_x * turnRatio);
    }

    /**
     * method which allows the rev integrated gyroscope to be used without the wrap around values
     * where it resets at 180 degrees
     *
     * @return double - unrestricted gyroscope value of the robot
     */
    protected double getGlobal(){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

}
