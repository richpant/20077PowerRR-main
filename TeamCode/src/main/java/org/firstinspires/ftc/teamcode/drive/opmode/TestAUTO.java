package org.firstinspires.ftc.teamcode.drive.opmode;

import static com.qualcomm.robotcore.hardware.DcMotor.*;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "TestAuto")
public class TestAUTO extends LinearOpMode {
    private DcMotor lift;
    private Servo claw2;

    private Servo claw;


    @Override

    public void runOpMode() {
        lift = hardwareMap.get(DcMotor.class, "lift");
        claw = hardwareMap.get(Servo.class, "claw");
        claw2 = hardwareMap.get(Servo.class,"claw2");
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        lift.setMode(RunMode.STOP_AND_RESET_ENCODER);


        Pose2d startPose = new Pose2d(-34, -70, 0);
        drive.setPoseEstimate(startPose);

        //lift.getZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)

                .addTemporalMarker(() -> drive.close())
                .waitSeconds(.4)

                .addTemporalMarker(() -> drive.lift(4300))
                .strafeLeft(63.5)
                //.addTemporalMarker(() -> drive.lifthold(200))
                .waitSeconds(.4)
                .forward(6.7)
                .waitSeconds(.4)
               .addTemporalMarker(() -> drive.open())
                .waitSeconds(.6)
                .back(6.7)
                .strafeRight(10)
                //.lineToLinearHeading(new Pose2d(-25,0, Math.toRadians(0)))
                .addTemporalMarker(() -> drive.lift(500))
                .lineToLinearHeading(new Pose2d(-60,-19.5, Math.toRadians(-180) + 1e-6))
                .forward(4)
                //.splineTo(new Vector2d(-72, -20), Math.toRadians(-180) + 1e-6)
                .addTemporalMarker(()-> drive.close())
                .addTemporalMarker(() -> drive.lift(4300))
                .lineToLinearHeading(new Pose2d(-23,-19.5, Math.toRadians(90)))
                .forward(7.9)
                .waitSeconds(.4)
                .addTemporalMarker(() -> drive.open())
                .back(7.9)
                .addTemporalMarker(() -> drive.lift(300))
                .lineToLinearHeading(new Pose2d(-60,-19, Math.toRadians(-180)))
                .forward(4.5)
                .addTemporalMarker(()-> drive.close())
                .addTemporalMarker(() -> drive.lift(4400))
                .lineToLinearHeading(new Pose2d(-25,-18, Math.toRadians(90)))
                .forward(7.4)
                .waitSeconds(.4)
                .addTemporalMarker(() -> drive.open())
                .back(7.4)
                .addTemporalMarker(() -> drive.lift(200))
                .lineToLinearHeading(new Pose2d(-59,-19, Math.toRadians(-180)))
                .forward(4.5)
                .addTemporalMarker(()-> drive.close())
                .addTemporalMarker(() -> drive.lift(1400))
                .lineToLinearHeading(new Pose2d(-50,-19, Math.toRadians(-90)))
                .forward(7.4)
                .waitSeconds(.4)
                .addTemporalMarker(() -> drive.open())
                .back(7.4)


                //.lineToLinearHeading(new Pose2d(-37,-15, Math.toRadians(0)))


                .build();
        waitForStart();

        if (!isStopRequested())
            drive.followTrajectorySequence(trajSeq);
    }


}
