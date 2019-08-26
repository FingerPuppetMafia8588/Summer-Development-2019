package org.firstinspires.ftc.teamcode.ShottaFlow;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by isaac.blandin on 8/14/19.
 */

@TeleOp(name = "blue face baby")
public class BlueFaceBaby extends ShottaHardware {

    public void runOpMode(){

        initShotta();

        waitForStart();

        int count = 0;
        boolean toggle = false;

        while(opModeIsActive()){

            if(gamepad2.dpad_up){
                collector.setPower(1);
            } else if (gamepad2.dpad_down){
                collector.setPower(1);
            }else {
                collector.setPower(0);
            }

            if (gamepad2.y){
                count++;
            } else {
                count = 0;
            }

            if (count == 1){
                toggle = !toggle;
            }

            if (toggle){
                flyLeft.setPower(1);
                flyRight.setPower(1);
            } else {
                flyRight.setPower(0);
                flyLeft.setPower(0);
            }

            if (gamepad2.a){
                feeder.setPosition(0.86);
            } else {
                feeder.setPosition(1);
            }

            if (gamepad2.b||gamepad2.dpad_down){
                agitator.setPosition(0);
            } else if (gamepad2.dpad_up){
                agitator.setPosition(1);
            } else {
                agitator.setPosition(0.5);
            }

            frontright.setPower(-gamepad1.right_stick_y);
            backright.setPower(-gamepad1.right_stick_y);
            frontleft.setPower(-gamepad1.left_stick_y);
            backleft.setPower(-gamepad1.left_stick_y);

        }
        stop();
    }

}
