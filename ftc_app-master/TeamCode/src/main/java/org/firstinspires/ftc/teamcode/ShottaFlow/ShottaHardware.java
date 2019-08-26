package org.firstinspires.ftc.teamcode.ShottaFlow;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by isaac.blandin on 8/14/19.
 */

public abstract class ShottaHardware extends LinearOpMode {

    DcMotor flyRight;
    DcMotor flyLeft;
    DcMotor collector;

    DcMotor frontright;
    DcMotor frontleft;
    DcMotor backright;
    DcMotor backleft;

    Servo feeder;
    Servo agitator;

    public void initShotta(){

        flyRight = hardwareMap.dcMotor.get("flyr");
        flyLeft = hardwareMap.dcMotor.get("flyl");
        flyLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        collector = hardwareMap.dcMotor.get("coll");

        feeder = hardwareMap.servo.get("feed");
        feeder.setPosition(1);

        agitator = hardwareMap.servo.get("agit");
        agitator.setPosition(0.5);

        frontright = hardwareMap.dcMotor.get("rf");
        frontleft = hardwareMap.dcMotor.get("lf");
        backright = hardwareMap.dcMotor.get("rb");
        backleft = hardwareMap.dcMotor.get("lb");

        //reverse the right motors due to using AndyMark motors on direct drive
        frontright.setDirection(DcMotor.Direction.REVERSE);
        backright.setDirection(DcMotor.Direction.REVERSE);
    }
}
