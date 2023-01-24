package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Test")
public class Test extends OpMode {
    public DcMotorEx arm = null;

    public Servo claw = null;

    public int armTarget = 0;
    public int armPos = 0;
    public int clawPressed = 0;

    @Override
    public void init() {
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        claw = hardwareMap.get(Servo.class, "claw");

        arm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        arm.setTargetPosition(0);
        arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        claw.setPosition(0);
    }

    @Override
    public void loop() {
        if (!(gamepad1.right_stick_y == 0)) {
            armTarget -= Math.round(gamepad1.right_stick_y * 5);
        } else if (gamepad1.left_trigger>0){
            armTarget = 250;
        }else if(gamepad1.right_trigger>0){
            armTarget = 0;
        }

        //limits
        if (armTarget > 250) {
            armTarget = 250;
        } else if (armTarget < 0) {
            armTarget = 0;
        }

        //power setting
        arm.setTargetPosition(armTarget);

        if (armTarget > armPos) {
            arm.setPower(0.3);
        } else if (armPos > armTarget) {
            arm.setPower(-0.3);
        } else if (armPos == armTarget) {
            arm.setPower(0);
        }

        armPos = arm.getCurrentPosition();
        telemetry.addData("error :", armTarget - armPos);
        telemetry.update();

        //intake
        {
            if ((gamepad1.left_bumper || gamepad1.right_bumper) && clawPressed == 0) {
                if (claw.getPosition() == 0.1) {
                    claw.setPosition(1);
                } else {
                    claw.setPosition(0.1);
                }
                clawPressed = 1;
            }
            if ((!gamepad1.left_bumper && !gamepad1.right_bumper) && clawPressed > 0) {
                clawPressed = 0;
            }
        }
    }
}
