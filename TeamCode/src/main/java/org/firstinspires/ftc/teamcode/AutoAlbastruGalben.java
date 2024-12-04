package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.hardwarePapiu;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;
@Autonomous(name = "AutoAlbastruGalben", group = "A")
public final class AutoAlbastruGalben extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        hardwarePapiu robot = new hardwarePapiu(this);
        robot.init();
        Pose2d beginPose = new Pose2d(32, 63, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();

        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .strafeTo(new Vector2d(-12,-32))
                        .waitSeconds(0.5)
                        .strafeToSplineHeading(new Vector2d(-24,-50), Math.toRadians(90))
                        .strafeToSplineHeading(new Vector2d(-48,-38), Math.toRadians(90))
                        .waitSeconds(0.5)
                        .strafeToSplineHeading(new Vector2d(-55,-54), Math.toRadians(45))
                        .waitSeconds(0.5)
                        .strafeToSplineHeading(new Vector2d(-57,-38), Math.toRadians(90))
                        .waitSeconds(0.5)
                        .strafeToSplineHeading(new Vector2d(-55,-54), Math.toRadians(45))
                        .waitSeconds(0.5)
                        .strafeToSplineHeading(new Vector2d(-58,-37), Math.toRadians(135))
                        .waitSeconds(0.5)
                        .strafeToSplineHeading(new Vector2d(-55,-54), Math.toRadians(45))
                        .waitSeconds(0.5)
                        .strafeToSplineHeading(new Vector2d(-55,-10), Math.toRadians(0))
                        .strafeToSplineHeading(new Vector2d(-28,-10), Math.toRadians(0))
                        .waitSeconds(0.5)
                        .strafeToSplineHeading(new Vector2d(-55,-10), Math.toRadians(0))
                        .waitSeconds(0.5)
                        .strafeToSplineHeading(new Vector2d(-55,-54), Math.toRadians(45))
                        .build());

    }
}
