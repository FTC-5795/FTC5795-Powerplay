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

package org.firstinspires.ftc.teamcode.otherCode.archive;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.teamcode.mainCode.functionClasses.vSlideMotorController;
import org.firstinspires.ftc.teamcode.OpenCV.cameraCode.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

//import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.otherCode.drive.SampleMecanumDrive;

import java.util.ArrayList;

@Disabled
public class Camera_Exa extends LinearOpMode {
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    int parkingSpace = 0;
//int getParkingSpace;

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

    //Tag ID of sleeve
    int Left = 17;
    int Middle = 18;
    int Right = 19;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode() {

        //    vSlideMotorController SlideLevel = new vSlideMotorController(hardwareMap);
//        gripServoController Grab = new gripServoController(hardwareMap);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 600, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
        //this bs makes the drive function work for...idk some reason, don't touch it
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        telemetry.setMsTransmissionInterval(50);

        //defines the sequences and loads them in before start

//        drive.setPoseEstimate(new Pose2d(0, 0, 0));

//        Grab.autoGrip(true);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */

        waitForStart();
        if (tagOfInterest != null) {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
        } else {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
        }

        while (opModeIsActive()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == Left || tag.id == Middle || tag.id == Right) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                // SlideLevel.autoVSlide(4);
                // SlideLevel.autoVSlide(1);
                // Grip.autoGrip(true);

                //Code for trajectory
                if (tagOfInterest == null || tagOfInterest.id == Left) {
                    int parkingSpace = 17;
                } else if (tagOfInterest.id == Middle) {
                    int parkingSpace = 18;
                } else if (tagOfInterest.id == Right) {
                    int parkingSpace = 19;
                }

                if (tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                } else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            } else {
                telemetry.addLine("Don't see tag of interest :(");

                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            sleep(20);
            telemetry.update();
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if (tagOfInterest != null) {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
        } else {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
        }


        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        while (opModeIsActive()) {
            sleep(20);
        }
    }

    void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }

}