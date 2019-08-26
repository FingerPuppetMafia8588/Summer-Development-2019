package org.firstinspires.ftc.teamcode.TestBots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by isaac.blandin on 7/16/19.
 */

@TeleOp (name = "painted nails")
public class TwoMotor extends LinearOpMode {

    DcMotor rightDrive;
    DcMotor leftDrive;

    public void runOpMode(){

        rightDrive = hardwareMap.dcMotor.get("rd");
        leftDrive = hardwareMap.dcMotor.get("ld");
        rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        while(opModeIsActive()){

            rightDrive.setPower(-gamepad1.right_stick_y);
            leftDrive.setPower(-gamepad1.left_stick_y);

        }
        stop();
    }

}
