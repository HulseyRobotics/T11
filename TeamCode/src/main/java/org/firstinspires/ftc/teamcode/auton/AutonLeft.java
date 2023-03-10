package org.firstinspires.ftc.teamcode.auton;

/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name="Auton left")
public class AutonLeft extends LinearOpMode
{
    // CHANGE CODE(change string according to what you named your motors)
    String frontLeftM = "left";
    String frontRightM = "right";
    // END OG CHANE CODE

    // MCODE
    DcMotorEx right = null;
    DcMotorEx left = null;

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    //    int ID_TAG_OF_INTEREST = 18; // Tag ID 18 from the 36h11 family
    int FORWARD = 2;
    int TURN = 1;

    AprilTagDetection tagOfInterest = null;

    public double ticksPerDegree = 8.3;

    @Override
    public void runOpMode()
    {

        //MCODE
        left = hardwareMap.get(DcMotorEx.class, frontLeftM);
        right = hardwareMap.get(DcMotorEx.class, frontRightM);
        // END MCODE

        // CHANGE CODE
        /*
        (sample mecanum equation: drive +/- strafe +/- turn)
        if I set power of 1 on drive variable it should drive forward
        if I set power of 1 on strafe variable it should strafe right
        if I set power of 1 on turn variable it should turn right
         */
        left.setDirection(DcMotorEx.Direction.FORWARD);
        right.setDirection(DcMotorEx.Direction.REVERSE);
        // END CHANGE CODE

        // MCODE
        left.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        left.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        left.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        // END MCODE

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(1280,720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == FORWARD || tag.id == TURN)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        if (tagOfInterest == null || tagOfInterest.id == FORWARD) {
            straight(2300,5000);
        }else {
            straight(2300,5000);
            turn(-90,5000);
            straight(1800,5000);
        }


        telemetry.update();
        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        while (opModeIsActive()) {sleep(20);}
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine("ID = " + detection.id);
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