package org.firstinspires.ftc.teamcode.OpenCV.cameraCode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.OpenCV.cameraCode.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.otherCode.drive.SampleMecanumDrive;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

public class cameraController {
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;
    double tagsize = 0.166; // UNITS ARE METERS

    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    int Left = 17; //aka 1
    int Middle = 18; //aka 2
    int Right = 19; //aka 3

    AprilTagDetection tagOfInterest = null;
    int parkingSpace = 0;


    public cameraController(HardwareMap hardwareMap) {
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
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
    }

    public int camera() {
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
        }

        if (tagOfInterest == null) {
            parkingSpace = 0;
        }
        else if (tagOfInterest.id == Left) {
            parkingSpace = 1; //17
        }
        else if (tagOfInterest.id == Middle) {
            parkingSpace = 2; //18
        }
        else if (tagOfInterest.id == Right) {
            parkingSpace = 3; //19
        }
        else {
            parkingSpace = 0;
        }

        return parkingSpace; //0-3 with 0 representing null
    }
}
