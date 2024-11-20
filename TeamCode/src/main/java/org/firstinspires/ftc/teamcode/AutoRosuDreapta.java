package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;
@Autonomous(name = "AutoRosuDreapta", group = "A")
public final class AutoRosuDreapta extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(-12, 63, Math.toRadians(270));
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

            waitForStart();

            Actions.runBlocking(
                    drive.actionBuilder(beginPose)
                            .strafeTo(new Vector2d(-47,40))
                            .waitSeconds(0.5)
                            .strafeToSplineHeading(new Vector2d(-50,50), Math.toRadians(135))
                            .waitSeconds(0.5)
                            .strafeToSplineHeading(new Vector2d(-58,40), Math.toRadians(270))
                            .waitSeconds(0.5)
                            .strafeToSplineHeading(new Vector2d(-50,50), Math.toRadians(135))
                            .waitSeconds(0.5)
                            .strafeToSplineHeading(new Vector2d(-58,40), Math.toRadians(235))
                            .waitSeconds(0.5)
                            .strafeToSplineHeading(new Vector2d(-50,50), Math.toRadians(135))
                            .waitSeconds(0.5)
                            .strafeToSplineHeading(new Vector2d(-30,0), Math.toRadians(0))
                            .strafeToSplineHeading(new Vector2d(-23,10), Math.toRadians(0))
                            .waitSeconds(1)
                            //.strafeToSplineHeading(new Vector2d(-30,0), Math.toRadians(0))
                            .setTangent(Math.toRadians(90))
                            .strafeToSplineHeading(new Vector2d(-45,12), Math.toRadians(0))
                            .splineToLinearHeading(new Pose2d(50,60,Math.toRadians(0)), Math.toRadians(10))
                            .strafeToSplineHeading(new Vector2d(55,54), Math.toRadians(45))
                            .waitSeconds(1)
                            .strafeToSplineHeading(new Vector2d(-50,58), Math.toRadians(0))
                            .build());

    }
}
