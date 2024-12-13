package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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
import com.arcrobotics.ftclib.controller.PIDController;
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
    public DcMotorEx misumi;
    private PIDController controller;
    public static double p=0.03, i=0, d=0;
    public static double f=0;
    public static int target=0;
    public final double ticks_in_degree=700/180.0;
    public class Brat{
        private Servo Brat;
        public Brat(HardwareMap hardwareMap){
            Brat = hardwareMap.get(Servo.class, "brat");
        }
        public class BratFata implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                Brat.setPosition(0.1);

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
                cleste.setPosition(0.3);

                return false;
            }
        }
        public Action clestestrans(){
            return new ClesteStrans();
        }
        public class ClesteLasat implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                cleste.setPosition(0.5);

                return false;
            }
        }
        public Action clestelasat(){
            return new ClesteLasat();
        }
    }
    public class Intake{
        private Servo intake1;
        private Servo intake2;
        public Intake(HardwareMap hardwareMap){intake1 = hardwareMap.get(Servo.class, "intake1");intake2 = hardwareMap.get(Servo.class, "intake2");}
        public class IntakeJos implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                intake1.setPosition(0.3);
                intake2.setPosition(0.7);

                return false;
            }
        }
        public Action IntakeJos(){
            return new IntakeJos();
        }
        public class IntakeOut implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                intake1.setPosition(1);
                intake2.setPosition(0);

                return false;
            }
        }
        public Action IntakeOut(){
            return new IntakeOut();
        }
        public class IntakeSus implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                intake1.setPosition(0.7);
                intake2.setPosition(0.3);

                return false;
            }
        }
        public Action IntakeSus(){
            return new IntakeSus();
        }

    }
    public class Target{
        public class target200 implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                target=150;
                while(target>misumi.getCurrentPosition()-5) {
                    controller.setPID(p, i, d);
                    int pozitie = misumi.getCurrentPosition();
                    double pid = controller.calculate(pozitie, target);
                    double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;
                    double power = pid + ff;
                    misumi.setPower(power);
                    telemetry.addData("pos ", pozitie);
                    telemetry.addData("target ", target);
                }
                return false;
            }
        }

        public class target0 implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                target=0;
                while(target>misumi.getCurrentPosition()-5) {
                    controller.setPID(p, i, d);
                    int pozitie = misumi.getCurrentPosition();
                    double pid = controller.calculate(pozitie, target);
                    double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;
                    double power = pid + ff;
                    misumi.setPower(power);
                    telemetry.addData("pos ", pozitie);
                    telemetry.addData("target ", target);
                }
                return false;
            }
        }
        public Action target0(){return new target0();}
        public Action target200(){return new target200();}
    }
    public class ClesteI{

        private Servo Roata1;
        private Servo Roata2;
        public ClesteI(HardwareMap hardwareMap){
            Roata1 = hardwareMap.get(Servo.class, "roata1");
            Roata2 = hardwareMap.get(Servo.class, "roata2");
        }
        public class ClesteStrans implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                Roata1.setPosition(0.35);
                Roata2.setPosition(0.65);

                return false;
            }
        }
        public Action clestestrans(){
            return new ClesteStrans();
        }
        public class ClesteLasat implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                Roata1.setPosition(0.42);
                Roata2.setPosition(0.58);

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
                int ticks = (int)(60 * VariableStorage.TICKS_PER_CM_Z);
                glisiera.setTargetPosition(-ticks);
                glisiera.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                glisiera.setPower(0.6);

                return false;
            }
        }
        public Action GlisieraSus(){
            return new Glisiera.GlisieraSus();
        }
        public class GlisieraJos implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                int ticks = (int)(0.3 * VariableStorage.TICKS_PER_CM_Z);
                glisiera.setTargetPosition(-ticks);
                glisiera.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                glisiera.setPower(0.6);

                return false;
            }
        }
        public Action GlisieraJos(){
            return new Glisiera.GlisieraJos();
        }
        public class GlisieraBara implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                int ticks = (int)(30 * VariableStorage.TICKS_PER_CM_Z);
                glisiera.setTargetPosition(-ticks);
                glisiera.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                glisiera.setPower(0.6);

                return false;
            }
        }
        public Action GlisieraBara(){
            return new Glisiera.GlisieraBara();
        }

        public class GlisieraBaraJos implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                int ticks = (int)(12 * VariableStorage.TICKS_PER_CM_Z);
                glisiera.setTargetPosition(-ticks);
                glisiera.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                glisiera.setPower(0.3);

                return false;
            }
        }
        public Action GlisieraBaraJos(){
            return new Glisiera.GlisieraBaraJos();
        }
    }
    @Override

    public void runOpMode() throws InterruptedException {

        Pose2d beginPose = new Pose2d(12, -60, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        hardwarePapiu robot = new hardwarePapiu(this);
        Brat brat = new Brat(hardwareMap);
        Cleste cleste = new Cleste(hardwareMap);
        Intake intake=new Intake(hardwareMap);
        ClesteI clestei = new ClesteI(hardwareMap);
        Glisiera glisiera = new Glisiera(hardwareMap);
        misumi = hardwareMap.get(DcMotorEx.class , "misumi");
        controller = new PIDController(p,i,d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        target=0;
        misumi.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        misumi.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        misumi.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Target target1=new Target();
        robot.init();
        waitForStart();
        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .strafeTo(new Vector2d(7,-35))
                        .stopAndAdd(glisiera.GlisieraBara())
                        .waitSeconds(2)
                        .stopAndAdd(brat.bratfata())
                        .waitSeconds(0.7)
                        .stopAndAdd(glisiera.GlisieraBaraJos())
                        .waitSeconds(3)
                        .stopAndAdd(cleste.clestelasat())
                        .waitSeconds(0.5)
                        .stopAndAdd(brat.bratjos())
                        .stopAndAdd(glisiera.GlisieraJos())
                        .waitSeconds(0.5)
                        .strafeTo(new Vector2d(12,-50))
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
                        .strafeToSplineHeading(new Vector2d(50,-55), Math.toRadians(90))
                        .build());

    }
}
