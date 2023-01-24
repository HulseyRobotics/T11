package org.firstinspires.ftc.teamcode.midterms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Course 4")

public class T11Course4 extends LinearOpMode {
    public DcMotor left = null;
    public DcMotor right = null;

    public double ticksPerDegree = 7.5;

    @Override
    public void runOpMode() throws InterruptedException {
        //wheel motor hardware control
        left = hardwareMap.get(DcMotor.class, "left");
        right = hardwareMap.get(DcMotor.class, "right");

        left.setDirection(DcMotor.Direction.FORWARD);
        right.setDirection(DcMotor.Direction.REVERSE);

        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //wait for start
        waitForStart();
        //init over
        straight(2800,7000);
        turn(-122, 7000);

        straight(4700,7000);
        turn(125, 7000);

        straight(3000,100000);

    }

    public void straight(int ticks, int wait) {
        left.setTargetPosition(ticks);
        right.setTargetPosition(ticks);

        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (ticks>0) {
            left.setPower(.4);
            right.setPower(.4);
        }else {
            left.setPower(-.4);
            right.setPower(-.4);
        }
        // END MCODE

        // CHANGE CODE
        // change value according to how long it takes robot to reach wanted position
        sleep(wait); // 5 seconds
        // END CHANGE CODE

        // MCODE
        left.setPower(0);
        right.setPower(0);
        // END MCODE

        // MOVE FORWARD 1.5 MAT LENGTH (END)

        // MCODE
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }


    public void turn(int degrees, int wait) {
        left.setTargetPosition((int) (-degrees * ticksPerDegree));
        right.setTargetPosition((int) (degrees * ticksPerDegree));

        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (degrees>0) {
            left.setPower(-.1);
            right.setPower(.1);
        }else {
            left.setPower(.1);
            right.setPower(-.1);
        }
        // END MCODE

        // CHANGE CODE
        // change value according to how long it takes robot to reach wanted position
        sleep(wait); // 5 seconds
        // END CHANGE CODE

        // MCODE
        left.setPower(0);
        right.setPower(0);
        // END MCODE

        // MOVE FORWARD 1.5 MAT LENGTH (END)

        // MCODE
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }
}

