package org.firstinspires.ftc.teamcode.AlCaDrone3;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by isaac.blandin on 6/25/19.
 */

@TeleOp(name = "Al 3.0")
public class AlTeleOp extends LinearOpMode {

    DcMotor rightDrive;
    DcMotor leftDrive;

    DcMotor collecter;
    DcMotor elevator;
    DcMotor flicker;

    public void runOpMode(){

        rightDrive = hardwareMap.dcMotor.get("rm");

        leftDrive = hardwareMap.dcMotor.get("lm");
        leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        collecter = hardwareMap.dcMotor.get("collector");
        elevator = hardwareMap.dcMotor.get("elevator");
        flicker = hardwareMap.dcMotor.get("flicker");

        waitForStart();
        while(opModeIsActive()){

            rightDrive.setPower(-gamepad1.right_stick_y);
            leftDrive.setPower(-gamepad1.left_stick_y);

            if (gamepad2.dpad_up){
                collecter.setPower(1);
            } else if (gamepad2.dpad_down){
                collecter.setPower(-1);
            } else {
                collecter.setPower(0);
            }

            if (gamepad2.right_bumper){
                elevator.setPower(-1);
            } else {
                elevator.setPower(0);
            }

            if (gamepad2.a){
                flicker.setPower(-1);
            } else {
                flicker.setPower(0);
            }

        }

        stop();

    }
}
