package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Hardware.VariableStorage;
import org.firstinspires.ftc.teamcode.Hardware.hardwarePapiu;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;
@Autonomous(name = "AutoAlbastruColorat", group = "A")
public final class AutoAlbastruColorat extends LinearOpMode {
    public class Brat{
        private Servo Brat;
        public Brat(HardwareMap hardwareMap){
            Brat = hardwareMap.get(Servo.class, "brat");
        }
        public class BratFata implements Action{
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
    public class Glisiera{
        private DcMotorEx glisiera;
        public Glisiera(HardwareMap hardwareMap){
            glisiera = hardwareMap.get(DcMotorEx.class, "glisiera");
            glisiera.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            glisiera.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        public class GlisieraSus implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                int ticks = (int)(75 * VariableStorage.TICKS_PER_CM_Z);
                glisiera.setTargetPosition(-ticks);
                glisiera.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                glisiera.setPower(0.6);

                return false;
            }
        }
        public Action GlisieraSus(){
            return new GlisieraSus();
        }
        public class GlisieraJos implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                int ticks = (int)(0 * VariableStorage.TICKS_PER_CM_Z);
                glisiera.setTargetPosition(-ticks);
                glisiera.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                glisiera.setPower(0.6);

                return false;
            }
        }
        public Action GlisieraJos(){
            return new GlisieraJos();
        }
    }
    @Override

    public void runOpMode() throws InterruptedException {

        Pose2d beginPose = new Pose2d(12, -60, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        hardwarePapiu robot = new hardwarePapiu(this);
        Brat brat = new Brat(hardwareMap);
        Cleste cleste = new Cleste(hardwareMap);
        robot.init();
        TrajectoryActionBuilder start = drive.actionBuilder(beginPose)
                .strafeTo(new Vector2d(12,-45))
                .waitSeconds(1);
        TrajectoryActionBuilder bara = start.endTrajectory().fresh()
                .strafeTo(new Vector2d(12,-38.5))
                .waitSeconds(1);
        TrajectoryActionBuilder specimen = bara.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(37,-40), Math.toRadians(0))
                .strafeToSplineHeading(new Vector2d(37,-5), Math.toRadians(0))
                .strafeToSplineHeading(new Vector2d(45,-5), Math.toRadians(0))
                .strafeToSplineHeading(new Vector2d(45,-55), Math.toRadians(0))
                .strafeToSplineHeading(new Vector2d(45,-5), Math.toRadians(0))
                .strafeToSplineHeading(new Vector2d(54,-5), Math.toRadians(0))
                .strafeToSplineHeading(new Vector2d(54,-55), Math.toRadians(0))
                .strafeToSplineHeading(new Vector2d(37,-55), Math.toRadians(90))
                .strafeToSplineHeading(new Vector2d(12,-40), Math.toRadians(270))
                .waitSeconds(0.5)
                .strafeToSplineHeading(new Vector2d(12,-42), Math.toRadians(270))
                .strafeToSplineHeading(new Vector2d(54,-30), Math.toRadians(0))
                .strafeToSplineHeading(new Vector2d(54,-5), Math.toRadians(0))
                .strafeToSplineHeading(new Vector2d(60,-5), Math.toRadians(0))
                .strafeToSplineHeading(new Vector2d(60,-55), Math.toRadians(0))
                .strafeToSplineHeading(new Vector2d(37,-55), Math.toRadians(90))
                .strafeToSplineHeading(new Vector2d(12,-40), Math.toRadians(270))
                .waitSeconds(0.5)
                .strafeToSplineHeading(new Vector2d(37,-55), Math.toRadians(90))
                .strafeToSplineHeading(new Vector2d(12,-40), Math.toRadians(270))
                .waitSeconds(0.5)
                .strafeToSplineHeading(new Vector2d(54,-55), Math.toRadians(270));
        Action starttraj=start.build();
        Action baratraj=bara.build();
        Action specimentraj=specimen.build();
        waitForStart();
        Actions.runBlocking(
                new SequentialAction(
                        starttraj,
                        brat.bratfata(),
                        baratraj,
                        cleste.clestelasat(),
                        specimentraj
                )
        );

    }
}
