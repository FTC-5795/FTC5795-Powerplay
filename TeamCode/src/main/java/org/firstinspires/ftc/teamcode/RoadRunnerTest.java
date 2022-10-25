package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.LineSegment;
import com.acmerobotics.roadrunner.path.Path;
import com.acmerobotics.roadrunner.path.PathBuilder;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

/* Designed to practice road runner framework for future implementation with odometry
Notes: -Motion profiles are extensions of PID for better control
-Paths express where the robot needs to go
-Trajectories combine motion profiles and paths to form required velocities

*/

@Autonomous

public class RoadRunnerTest extends LinearOpMode {

    private double kP = 1;
    private double kI = 1;
    private double kD = 1;
    private double kV = 0;
    private double kA = 0;
    private double kStatic = 0;
    public ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        //PIDF Controller declarations
        PIDCoefficients coeffs = new PIDCoefficients(kP, kI, kD);
        PIDFController controller = new PIDFController(coeffs, kV, kA, kStatic); //use controller for PID calculations

        Pose2d pose = new Pose2d(0,0,0); //start position and orientation
        LineSegment line = new LineSegment(
                new Vector2d(0,0),
                new Vector2d(60,60)
        );

        while (opModeIsActive()) {
            controller.setTargetPosition(10);
            double correction = controller.update(0);
        }

    }

    public double PIDControl(double target, double state) {

        MotionProfile profile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(0, 0, 0),
                new MotionState(target, 0, 0),
                25,
                40,
                100
        ); //First motion state is start, second motion state is target (expressed in position, velocity, and acceleration)
        //maxVel, maxAccel, and maxJerk (derivative of Accel) limits MotionProfiles outputs

        MotionState profileState = profile.get(0);
        return (profileState.getX());
    }
}