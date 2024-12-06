package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Hardware.hardwarePapiu;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;
@Autonomous(name = "AutoAlbastruGalben", group = "A")
public final class AutoAlbastruGalben extends LinearOpMode {
    public class Brat{
        private Servo Brat;
        public Brat(HardwareMap hardwareMap){
            Brat = hardwareMap.get(Servo.class, "brat");
        }
        public class BratFata implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                Brat.setPosition(0.03);

                return false;
            }
        }
        public Action bratfata(){
            return new BratFata();
        }
        public class BratJos implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                Brat.setPosition(1);

                return false;
            }
        }
        public Action bratjos(){
            return new BratJos();
        }
    }

    public class Cleste{
        private Servo cleste;
        public Cleste(HardwareMap hardwareMap){
            cleste = hardwareMap.get(Servo.class, "cleste");
        }
        public class ClesteStrans implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                cleste.setPosition(0.5);

                return false;
            }
        }
        public Action clestestrans(){
            return new ClesteStrans();
        }
        public class ClesteLasat implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                cleste.setPosition(0.3);

                return false;
            }
        }
        public Action clestelasat(){
            return new ClesteLasat();
        }
    }
    @Override
    public void runOpMode() throws InterruptedException {
        hardwarePapiu robot = new hardwarePapiu(this);
        robot.init();
        Pose2d beginPose = new Pose2d(32, -63, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        Brat brat = new Brat(hardwareMap);
        Cleste cleste = new Cleste(hardwareMap);
        robot.init();
        waitForStart();

        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .strafeTo(new Vector2d(12,-45))
                        .afterDisp(0, brat.bratfata())
                        .strafeTo(new Vector2d(12,-38.5))
                        .afterDisp(0, cleste.clestelasat())
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
