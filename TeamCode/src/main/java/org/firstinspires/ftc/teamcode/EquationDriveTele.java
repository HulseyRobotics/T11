package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Equation Tele")
public class EquationDriveTele extends OpMode {
    public DcMotorEx left = null;
    public DcMotorEx right = null;

    public double forward = 0.0;
    public double turn = 0.0;

    public DcMotorEx arm = null;

    public Servo claw1 = null;
    public Servo claw2 = null;
    public int armTarget = 0;
    public int armPos = 0;

    @Override
    public void init() {
        //wheel motor hardware control
        left = hardwareMap.get(DcMotorEx.class, "left");
        right = hardwareMap.get(DcMotorEx.class, "right");
        arm = hardwareMap.get(DcMotorEx.class, "arm");

        claw1 = hardwareMap.get(Servo.class, "claw1");
        claw2 = hardwareMap.get(Servo.class, "claw2");

        left.setDirection(DcMotorEx.Direction.REVERSE);
        right.setDirection(DcMotorEx.Direction.FORWARD);
        arm.setDirection(DcMotorEx.Direction.FORWARD);


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

        arm = hardwareMap.get(DcMotorEx.class, "arm");

        arm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        arm.setTargetPosition(0);
        arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        arm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

    }

    @Override
    public void loop() {

        forward = -gamepad1.left_stick_y;
        turn = gamepad1.left_stick_x * 0.5;

        left.setVelocity((forward+turn)*3000);
        right.setVelocity((forward-turn)*3000);
        telemetry.addData("arm position",arm.getCurrentPosition());
        telemetry.update();
        //250 = arm max ticks



        if (!(gamepad1.right_stick_y == 0)) {
            armTarget -= Math.ceil(gamepad1.right_stick_y);
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
            arm.setPower(0.4);
        } else if (armPos > armTarget) {
            arm.setPower(-0.4);
        } else if (armPos == armTarget) {
            arm.setPower(0);
        }

        armPos = arm.getCurrentPosition();
        telemetry.addData("error :", armTarget - armPos);
        telemetry.update();

        //intake
        {
            if ((gamepad1.left_bumper || gamepad1.right_bumper)) {
                claw1.setPosition(1);
                claw2.setPosition(0);
            }else{
                claw1.setPosition(0);
                claw2.setPosition(1);
            }
        }

    }
}
