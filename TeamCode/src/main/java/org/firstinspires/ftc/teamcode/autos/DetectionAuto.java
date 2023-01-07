package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.commands.FindAprilTagCommand;
import org.firstinspires.ftc.teamcode.commands.MoveArmCommand;
import org.firstinspires.ftc.teamcode.roadrunner.drive.RRMecanumDrive;
import org.firstinspires.ftc.teamcode.shplib.commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.shplib.commands.ParallelCommandGroup;
import org.firstinspires.ftc.teamcode.shplib.commands.RunCommand;
import org.firstinspires.ftc.teamcode.shplib.commands.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.shplib.commands.WaitCommand;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.openftc.apriltag.AprilTagDetection;

@Autonomous
public class DetectionAuto extends BaseRobot {
    RRMecanumDrive drive;
    Trajectory t_check_signal, t_forward, t_back, t_left, t_right;
    @Override
    public void init() {
        super.init();
        drive = new RRMecanumDrive(hardwareMap);
        vision.setState(VisionSubsystem.State.ENABLED);

        Pose2d startPos = new Pose2d(0, 0, Math.toRadians(90));
        drive.setPoseEstimate(startPos);
        t_check_signal = drive.trajectoryBuilder(startPos)
                .splineToConstantHeading(new Vector2d(3, 12), 90)
                .build();
        t_forward = drive.trajectoryBuilder(t_check_signal.end())
                .forward(20)
                .build();
        t_back = drive.trajectoryBuilder(t_forward.end())
                .back(4)
                .build();
        t_left = drive.trajectoryBuilder(t_back.end())
                .strafeLeft(24)
                .build();
        t_right = drive.trajectoryBuilder(t_back.end())
                .strafeRight(26)
                .build();
    }
    @Override
    public void init_loop() {
        super.init_loop();
        vision.periodic(telemetry);
        telemetry.addData("tags detected: ", vision.getTags().size());
        if (vision.detectedTags()){
            for (AprilTagDetection tag : vision.getTags()) {
                telemetry.addData("ID: ", tag.id);
            }
            vision.setState(VisionSubsystem.State.DISABLED);
        }
    }



    @Override
    public void start() {
        super.start();
        CommandScheduler.getInstance().scheduleCommand(
            new SequentialCommandGroup(
                    /*
                new ParallelCommandGroup(
                    new FindAprilTagCommand(vision, t_check_signal.duration() + 1),
                    new RunCommand(()-> {
                        drive.followTrajectoryAsync(t_check_signal);
                    }).then(
                        new WaitCommand(t_check_signal.duration())
                    )
                ),

                     */
                new RunCommand(()-> {
                    drive.followTrajectoryAsync(t_check_signal);
                }),
                new WaitCommand(t_check_signal.duration()),
                new FindAprilTagCommand(vision),
                new RunCommand(()-> {
                    drive.followTrajectoryAsync(t_forward);
                }),
                new WaitCommand(t_forward.duration()),
                new RunCommand(()-> {
                    drive.followTrajectoryAsync(t_back);
                }),
                new WaitCommand(t_back.duration()),
                new RunCommand(()-> {
                    if (!vision.detectedTags())
                        return;
                    if(vision.getTags().get(0).id == 0)
                        drive.followTrajectoryAsync(t_left);
                    else if (vision.getTags().get(0).id == 2)
                        drive.followTrajectoryAsync(t_right);
                })
            )

        );
    }

    @Override
    public void loop() {
        super.loop();
        for (AprilTagDetection tag : vision.getTags()) {
            telemetry.addData("ID: ", tag.id);
        }
        drive.update();
        telemetry.addData("pose estimate (x): ", drive.getPoseEstimate().getX());
        telemetry.addData("pose estimate (y): ", drive.getPoseEstimate().getY());
        telemetry.addData("pose estimate (heading): ", drive.getPoseEstimate().getHeading());
    }

}
