package org.firstinspires.ftc.teamcode.OpenCV;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

public class GroundbreakerLeft extends LinearOpMode {

    private ElapsedTime autoTimer = new ElapsedTime();
    private ElapsedTime sleepTimer = new ElapsedTime();
    private int slideLevel; //1-12 slide height adjustment in trajectories
    private static double poleDepth = 2; //inches forward/back bot will move when scoring
    private int stackHeight = 9; //begins with stack of 5
    private int parkLocation = 0; //1,2,3 from left to right

    enum State {
        INITIAL,
        PRIMARY_CHANNEL,
        A_CHANNEL,
        B_CHANNEL
    }


    @Override
    public void runOpMode() throws InterruptedException {




    }
}
