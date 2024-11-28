package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "AutoRosuColorat", group = "A")
public final class AutoRosuColorat extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(-12, 63, Math.toRadians(90));
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

            waitForStart();

            Actions.runBlocking(
                    drive.actionBuilder(beginPose)
                            .strafeTo(new Vector2d(-12,31))
                            .waitSeconds(0.5)
                            .strafeToSplineHeading(new Vector2d(-37,40), Math.toRadians(180))
                            .strafeToSplineHeading(new Vector2d(-37,5), Math.toRadians(180))
                            .strafeToSplineHeading(new Vector2d(-45,5), Math.toRadians(180))
                            .strafeToSplineHeading(new Vector2d(-45,55), Math.toRadians(180))
                            .strafeToSplineHeading(new Vector2d(-45,5), Math.toRadians(180))
                            .strafeToSplineHeading(new Vector2d(-54,5), Math.toRadians(180))
                            .strafeToSplineHeading(new Vector2d(-54,55), Math.toRadians(180))
                            .strafeToSplineHeading(new Vector2d(-37,55), Math.toRadians(270))
                            .strafeToSplineHeading(new Vector2d(-12,31), Math.toRadians(90))
                            .waitSeconds(0.5)
                            .strafeToSplineHeading(new Vector2d(-12,40), Math.toRadians(90))
                            .strafeToSplineHeading(new Vector2d(-54,30), Math.toRadians(180))
                            .strafeToSplineHeading(new Vector2d(-54,5), Math.toRadians(180))
                            .strafeToSplineHeading(new Vector2d(-60,5), Math.toRadians(180))
                            .strafeToSplineHeading(new Vector2d(-60,55), Math.toRadians(180))
                            .strafeToSplineHeading(new Vector2d(-37,55), Math.toRadians(270))
                            .strafeToSplineHeading(new Vector2d(-12,31), Math.toRadians(90))
                            .waitSeconds(0.5)
                            .strafeToSplineHeading(new Vector2d(-37,55), Math.toRadians(270))
                            .strafeToSplineHeading(new Vector2d(-12,31), Math.toRadians(90))
                            .waitSeconds(0.5)
                            .strafeToSplineHeading(new Vector2d(-54,55), Math.toRadians(90))
                            .build());

    }
}
