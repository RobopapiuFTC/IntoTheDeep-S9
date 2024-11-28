package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "AutoRosuGalben", group = "A")
public final class AutoRosuGalben extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(32, 63, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();

        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .strafeTo(new Vector2d(12,32))
                        .waitSeconds(0.5)
                        .strafeToSplineHeading(new Vector2d(24,50), Math.toRadians(270))
                        .strafeToSplineHeading(new Vector2d(48,38), Math.toRadians(270))
                        .waitSeconds(0.5)
                        .strafeToSplineHeading(new Vector2d(55,54), Math.toRadians(225))
                        .waitSeconds(0.5)
                        .strafeToSplineHeading(new Vector2d(57,38), Math.toRadians(270))
                        .waitSeconds(0.5)
                        .strafeToSplineHeading(new Vector2d(55,54), Math.toRadians(225))
                        .waitSeconds(0.5)
                        .strafeToSplineHeading(new Vector2d(58,37), Math.toRadians(315))
                        .waitSeconds(0.5)
                        .strafeToSplineHeading(new Vector2d(55,54), Math.toRadians(225))
                        .waitSeconds(0.5)
                        .strafeToSplineHeading(new Vector2d(55,10), Math.toRadians(180))
                        .strafeToSplineHeading(new Vector2d(28,10), Math.toRadians(180))
                        .waitSeconds(0.5)
                        .strafeToSplineHeading(new Vector2d(55,10), Math.toRadians(180))
                        .waitSeconds(0.5)
                        .strafeToSplineHeading(new Vector2d(55,54), Math.toRadians(225))
                        .build());

    }
}
