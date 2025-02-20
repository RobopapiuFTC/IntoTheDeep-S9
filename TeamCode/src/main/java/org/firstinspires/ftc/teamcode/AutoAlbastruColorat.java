package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Hardware.VariableStorage;
import org.firstinspires.ftc.teamcode.Hardware.hardwarePapiu;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;

import java.util.Arrays;

@Autonomous(name = "AutoAlbastruColorat", group = "A")
public final class AutoAlbastruColorat extends LinearOpMode {
    public DcMotorEx misumi;
    private PIDController controller;
    public static double p=0.03, i=0, d=0;
    public static double f=0;
    public static int target=0;
    public final double ticks_in_degree=700/180.0;
    public class Brat{
        private Servo ServoBrat;
        private Servo ServoBrat1;
        public Brat(HardwareMap hardwareMap){
            ServoBrat = hardwareMap.get(Servo.class, "brat");
            ServoBrat1 = hardwareMap.get(Servo.class, "brat1");
        }
        public class BratFata implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                ServoBrat.setPosition(1);
                ServoBrat1.setPosition(1);

                return false;
            }
        }
        public Action bratfata(){
            return new Brat.BratFata();
        }
        public class BratAuto implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                ServoBrat.setPosition(0.4);
                ServoBrat1.setPosition(0.4);

                return false;
            }
        }
        public Action bratauto(){
            return new Brat.BratAuto();
        }

        public class BratJos implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                ServoBrat.setPosition(0.55);
                ServoBrat1.setPosition(0.55);

                return false;
            }
        }
        public Action bratjos(){
            return new Brat.BratJos();
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
                cleste.setPosition(0.23);

                return false;
            }
        }
        public Action clestestrans(){
            return new Cleste.ClesteStrans();
        }
        public class ClesteLasat implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                cleste.setPosition(0.65);

                return false;
            }
        }
        public Action clestelasat(){
            return new Cleste.ClesteLasat();
        }
    }
    public class Intake{
        private Servo intake1;
        private Servo intake2;
        public Intake(HardwareMap hardwareMap){intake1 = hardwareMap.get(Servo.class, "intake1");intake2 = hardwareMap.get(Servo.class, "intake2");}
        public class IntakeJos implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                intake1.setPosition(0.75);
                intake2.setPosition(0.25);

                return false;
            }
        }
        public Action IntakeJos(){
            return new Intake.IntakeJos();
        }
        public class IntakeJosJos implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                intake1.setPosition(0.7);
                intake2.setPosition(0.3);

                return false;
            }
        }
        public Action IntakeJosJos() {
            return new Intake.IntakeJosJos();
        }
        public class IntakeOut implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                intake1.setPosition(0.95);
                intake2.setPosition(0.05);

                return false;
            }
        }
        public Action IntakeOut(){
            return new Intake.IntakeOut();
        }

    }
    public class faras{
        private Servo faras;
        public faras(HardwareMap hardwareMap){faras = hardwareMap.get(Servo.class, "faras");}
        public class farasout implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                faras.setPosition(0.85);

                return false;
            }
        }
        public Action farasout(){
            return new faras.farasout();
        }
        public class farasin implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                faras.setPosition(0.5);

                return false;
            }
        }
        public Action farasin(){
            return new faras.farasin();
        }

    }
    public class Target{
        public class target150 implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
               /* target=150;
                while(target>misumi.getCurrentPosition()-5 || target<misumi.getCurrentPosition()-5){
                    controller.setPID(p, i, d);
                    int pozitie = misumi.getCurrentPosition();
                    double pid = controller.calculate(pozitie, target);
                    double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;
                    double power = pid + ff;
                    misumi.setPower(power);
                    telemetry.addData("pos ", pozitie);
                    telemetry.addData("target ", target);
                }
                misumi.setPower(0);*/
                int ticks = (int)(6 * VariableStorage.TICKS_PER_CM_Z);
                misumi.setTargetPosition(-ticks);
                misumi.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                misumi.setPower(0.5);
                return false;
            }
        }
        public class target300 implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                int ticks = (int)(13 * VariableStorage.TICKS_PER_CM_Z);
                misumi.setTargetPosition(-ticks);
                misumi.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                misumi.setPower(0.5);
                return false;
            }
        }
        public class target0 implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                int ticks = (int)(0 * VariableStorage.TICKS_PER_CM_Z);
                misumi.setTargetPosition(-ticks);
                misumi.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                misumi.setPower(0.5);
                return false;
            }
        }
        public Action target0(){return new Target.target0();}
        public Action target150(){return new Target.target150();}
        public Action target300(){return new Target.target300();}
    }
    public class Active{

        private DcMotorEx Active;
        public Active(HardwareMap hardwareMap){
            Active = hardwareMap.get(DcMotorEx.class, "active");
        }
        public class ActiveStop implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                Active.setPower(0);
                Active.setDirection(DcMotorSimple.Direction.REVERSE);

                return false;
            }
        }
        public Action activestop(){
            return new Active.ActiveStop();
        }
        public class ActiveIa implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                Active.setDirection(DcMotorSimple.Direction.REVERSE);
                Active.setPower(1);
                Active.setDirection(DcMotorSimple.Direction.REVERSE);

                return false;
            }
        }
        public Action activeia(){
            return new Active.ActiveIa();
        }

        public class activescoate implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                Active.setDirection(DcMotorSimple.Direction.FORWARD);
                Active.setPower(1);
                Active.setDirection(DcMotorSimple.Direction.FORWARD);

                return false;
            }
        }
        public Action activescoate(){
            return new Active.activescoate();
        }
    }
    public class Glisiera{
        private DcMotorEx glisiera;
        private DcMotorEx glisiera1;
        public Glisiera(HardwareMap hardwareMap){
            glisiera = hardwareMap.get(DcMotorEx.class, "glisiera");
            glisiera.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            glisiera.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            glisiera1 = hardwareMap.get(DcMotorEx.class, "glisiera1");
            glisiera1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            glisiera1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        public class GlisieraSus implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                int ticks = (int)(130 * VariableStorage.TICKS_PER_CM_Z);
                glisiera.setTargetPosition(-ticks);
                glisiera.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                glisiera.setPower(1);
                glisiera1.setTargetPosition(-ticks);
                glisiera1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                glisiera1.setPower(1);

                return false;
            }
        }
        public Action GlisieraSus(){
            return new Glisiera.GlisieraSus();
        }
        public class GlisieraJos implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                int ticks = (int)(0 * VariableStorage.TICKS_PER_CM_Z);
                glisiera.setTargetPosition(-ticks);
                glisiera.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                glisiera.setPower(1);
                glisiera1.setTargetPosition(-ticks);
                glisiera1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                glisiera1.setPower(1);

                return false;
            }
        }
        public Action GlisieraJos(){
            return new Glisiera.GlisieraJos();
        }
        public class GlisieraBara implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                int ticks = (int)(40 * VariableStorage.TICKS_PER_CM_Z);
                glisiera.setTargetPosition(-ticks);
                glisiera.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                glisiera.setPower(1);
                glisiera1.setTargetPosition(-ticks);
                glisiera1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                glisiera1.setPower(1);
                return false;
            }
        }
        public Action GlisieraBara(){
            return new Glisiera.GlisieraBara();
        }

        public class GlisieraBaraJos implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                int ticks = (int)(5 * VariableStorage.TICKS_PER_CM_Z);
                glisiera.setTargetPosition(-ticks);
                glisiera.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                glisiera.setPower(1);
                glisiera1.setTargetPosition(-ticks);
                glisiera1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                glisiera1.setPower(1);
                return false;
            }
        }
        public Action GlisieraBaraJos(){
            return new Glisiera.GlisieraBaraJos();
        }
    }
    @Override

    public void runOpMode() throws InterruptedException {

        Pose2d beginPose = new Pose2d(-2, -63.5, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        hardwarePapiu robot = new hardwarePapiu(this);
        Brat brat = new Brat(hardwareMap);
        Cleste cleste = new Cleste(hardwareMap);
        Intake intake=new Intake(hardwareMap);
        Active active = new Active(hardwareMap);
        Glisiera glisiera = new Glisiera(hardwareMap);
        faras faras = new faras(hardwareMap);
        VelConstraint baseVelConstraint= new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(50),
                new AngularVelConstraint(Math.PI/2)
        ));
        misumi = hardwareMap.get(DcMotorEx.class , "misumi");
       /* controller = new PIDController(p,i,d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        target=0; */
        misumi.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        misumi.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        misumi.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Target target=new Target();
        robot.init();
        robot.ServoBrat.setPosition(0.4);
        robot.ServoBrat1.setPosition(0.4);
        robot.Cleste.setPosition(0.23);
        waitForStart();
        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .stopAndAdd(brat.bratjos())
                        .strafeToSplineHeading(new Vector2d(-8,-25), Math.toRadians(90))//bara1
                        .stopAndAdd(glisiera.GlisieraBara())
                        .waitSeconds(0.9)
                        .stopAndAdd(cleste.clestelasat())
                        .strafeToSplineHeading(new Vector2d(-8,-40), Math.toRadians(90))
                        .stopAndAdd(brat.bratauto())
                        .stopAndAdd(glisiera.GlisieraJos())
                        .strafeToSplineHeading(new Vector2d(27,-50), Math.toRadians(90))
                        .strafeToSplineHeading(new Vector2d(32,-31), Math.toRadians(35))//cub1
                        .stopAndAdd(active.activeia())
                        .stopAndAdd(target.target300())
                        .waitSeconds(0.3)
                        .stopAndAdd(intake.IntakeJosJos())
                        .waitSeconds(0.7)
                        .stopAndAdd(intake.IntakeOut())
                        .strafeToSplineHeading(new Vector2d(35,-45), Math.toRadians(285))
                        .stopAndAdd(active.activescoate())
                        .waitSeconds(0.3)
                        .stopAndAdd(active.activeia())
                        .strafeToSplineHeading(new Vector2d(39.8,-33), Math.toRadians(25))//cub2
                        .stopAndAdd(intake.IntakeJosJos())
                        .waitSeconds(0.7)
                        .stopAndAdd(intake.IntakeOut())
                        .strafeToSplineHeading(new Vector2d(40,-45), Math.toRadians(285))
                        .stopAndAdd(active.activescoate())
                        .waitSeconds(0.3)
                        .stopAndAdd(active.activestop())
                        .stopAndAdd(target.target0())
                        .strafeToSplineHeading(new Vector2d(35,-50), Math.toRadians(90))//specimen1
                        .stopAndAdd(brat.bratfata())
                        .waitSeconds(0.1)
                        .strafeToSplineHeading(new Vector2d(35,-60), Math.toRadians(90))
                        .stopAndAdd(cleste.clestestrans())
                        .waitSeconds(0.3)
                        .stopAndAdd(glisiera.GlisieraBaraJos())
                        .stopAndAdd(brat.bratjos())
                        .stopAndAdd(glisiera.GlisieraJos())
                        .strafeToSplineHeading(new Vector2d(-12,-25), Math.toRadians(90))//bara2
                        .stopAndAdd(glisiera.GlisieraBara())
                        .waitSeconds(0.9)
                        .stopAndAdd(cleste.clestelasat())
                        .stopAndAdd(glisiera.GlisieraJos())
                        .strafeToSplineHeading(new Vector2d(-12,-40), Math.toRadians(90))
                        .strafeToSplineHeading(new Vector2d(35,-50), Math.toRadians(90))//specimen2
                        .stopAndAdd(brat.bratfata())
                        .waitSeconds(0.1)
                        .strafeToSplineHeading(new Vector2d(35,-60), Math.toRadians(90))
                        .stopAndAdd(cleste.clestestrans())
                        .waitSeconds(0.3)
                        .stopAndAdd(glisiera.GlisieraBaraJos())
                        .stopAndAdd(brat.bratjos())
                        .stopAndAdd(glisiera.GlisieraJos())
                        .strafeToSplineHeading(new Vector2d(-14,-26), Math.toRadians(90))//bara3
                        .stopAndAdd(glisiera.GlisieraBara())
                        .waitSeconds(0.9)
                        .stopAndAdd(cleste.clestelasat())
                        .stopAndAdd(glisiera.GlisieraJos())
                        .strafeToSplineHeading(new Vector2d(-14,-40), Math.toRadians(90))
                        .strafeToSplineHeading(new Vector2d(35,-50), Math.toRadians(90))//specimen3
                        .stopAndAdd(brat.bratfata())
                        .waitSeconds(0.1)
                        .strafeToSplineHeading(new Vector2d(35,-60), Math.toRadians(90))
                        .stopAndAdd(cleste.clestestrans())
                        .waitSeconds(0.2)
                        .stopAndAdd(glisiera.GlisieraBaraJos())
                        .stopAndAdd(brat.bratjos())
                        .stopAndAdd(glisiera.GlisieraJos())
                        .strafeToSplineHeading(new Vector2d(-16,-26), Math.toRadians(90))//bara4
                        .stopAndAdd(glisiera.GlisieraBara())
                        .waitSeconds(0.9)
                        .stopAndAdd(cleste.clestelasat())
                        .stopAndAdd(glisiera.GlisieraJos())
                        .strafeToSplineHeading(new Vector2d(-16,-40), Math.toRadians(90))
                        .strafeToSplineHeading(new Vector2d(40,-50), Math.toRadians(90))//specimen4
                        .stopAndAdd(brat.bratfata())
                        .waitSeconds(0.1)
                        .strafeToSplineHeading(new Vector2d(40,-60), Math.toRadians(90))
                        .stopAndAdd(cleste.clestestrans())
                        .waitSeconds(0.2)
                        .stopAndAdd(glisiera.GlisieraBaraJos())
                        .stopAndAdd(brat.bratjos())
                        .stopAndAdd(glisiera.GlisieraJos())
                        .strafeToSplineHeading(new Vector2d(-16,-25), Math.toRadians(90))//bara5
                        .stopAndAdd(glisiera.GlisieraBara())
                        .waitSeconds(0.9)
                        .stopAndAdd(cleste.clestelasat())
                        .stopAndAdd(glisiera.GlisieraJos())
                        .strafeToSplineHeading(new Vector2d(35,-50), Math.toRadians(90))
                        .stopAndAdd(brat.bratjos())
                        .strafeToSplineHeading(new Vector2d(35,-60), Math.toRadians(90))
                        .build());

    }
}
