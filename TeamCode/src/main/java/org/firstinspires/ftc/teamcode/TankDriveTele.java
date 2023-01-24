package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Tank Tele")
public class TankDriveTele extends OpMode {
    public DcMotorEx left = null;
    public DcMotorEx right = null;
    public DcMotorEx arm = null;
    public Servo claw = null;

    public double leftForward= 0.0;
    public double rightForward = 0.0;

    @Override
    public void init() {
        //wheel motor hardware control
        left = hardwareMap.get(DcMotorEx.class, "left");
        right = hardwareMap.get(DcMotorEx.class, "right");
        arm = hardwareMap.get(DcMotorEx.class, "arm");

        left.setDirection(DcMotorEx.Direction.FORWARD);
        right.setDirection(DcMotorEx.Direction.REVERSE);
        arm.setDirection(DcMotorEx.Direction.REVERSE);


        left.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        left.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        left.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        arm.setTargetPosition(0);
        arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        //tuning PIDs
        {
            left.setVelocityPIDFCoefficients(15, 0, 0, 0);
            right.setVelocityPIDFCoefficients(15, 0, 0, 0);
        }
    }

    @Override
    public void loop() {
        leftForward = gamepad1.left_stick_y;
        rightForward = gamepad1.right_stick_y;

        left.setVelocity(leftForward*3000);
        right.setVelocity(rightForward*3000);
    }
}
