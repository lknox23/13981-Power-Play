package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.drive.RRMecanumDrive;

/**
 * To understand how Road Runner works and setting it up: https://learnroadrunner.com/
 */


@Autonomous

public class RoadRunnerAuto extends LinearOpMode {

    public void runOpMode() {
        RRMecanumDrive drive = new RRMecanumDrive(hardwareMap);

        Pose2d startPos = new Pose2d(10, 10, Math.toRadians(90));
        drive.setPoseEstimate(startPos);

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        Trajectory traj1 = drive.trajectoryBuilder(startPos)
                //.splineTo(new Vector2d(20, 20), Math.toRadians(90))
                .forward(10)
                .build();
        drive.followTrajectory(traj1);

        // do something
        sleep(2000);
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .strafeLeft(10)
                .build();
        drive.followTrajectory(traj2);

        // do something else
        sleep(2000);
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .back(10)
                .build();
        drive.followTrajectory(traj3);

        // done

        return;
    }
}
