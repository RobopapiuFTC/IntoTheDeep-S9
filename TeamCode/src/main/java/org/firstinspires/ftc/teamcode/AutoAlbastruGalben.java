package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;
import androidx.core.view.ActionProvider;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Hardware.VariableStorage;
import org.firstinspires.ftc.teamcode.Hardware.hardwarePapiu;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;
@Autonomous(name = "AutoAlbastruGalben", group = "A")
public final class AutoAlbastruGalben extends LinearOpMode {
    public DcMotorEx misumi;
    private PIDController controller;
    public static double p=0.03, i=0, d=0;
    public static double f=0;
    public static int target=0;
    public final double ticks_in_degree=700/180.0;
    public static Pose2d beginPose;
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
                ServoBrat.setPosition(0.8);
                ServoBrat1.setPosition(0.8);

                return false;
            }
        }
        public Action bratfata(){
            return new BratFata();
        }
        public class BratAuto implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                ServoBrat.setPosition(0.8);
                ServoBrat1.setPosition(0.8);

                return false;
            }
        }
        public Action bratauto(){
            return new BratAuto();
        }

        public class BratJos implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                ServoBrat.setPosition(0.19); //1
                ServoBrat1.setPosition(0.19);

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
                cleste.setPosition(0.25);

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
                intake1.setPosition(0.8);
                intake2.setPosition(0.2);

                return false;
            }
        }
        public Action IntakeJos(){
            return new IntakeJos();
        }
        public class IntakeJosJos implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                intake1.setPosition(0.74);
                intake2.setPosition(0.26);

                return false;
            }
        }
        public Action IntakeJosJos(){
            return new IntakeJosJos();
        }
        public class IntakeOut implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                intake1.setPosition(0.9);
                intake2.setPosition(0.1);

                return false;
            }
        }
        public Action IntakeOut(){
            return new IntakeOut();
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
                int ticks = (int)(16 * VariableStorage.TICKS_PER_CM_Z);
                misumi.setTargetPosition(-ticks);
                misumi.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                misumi.setPower(1);
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
        public Action target0(){return new target0();}
        public Action target150(){return new target150();}
        public Action target300(){return new target300();}
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
            return new ActiveStop();
        }
        public class ActiveIa implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                Active.setPower(1);
                Active.setDirection(DcMotorSimple.Direction.REVERSE);

                return false;
            }
        }
        public Action activeia(){
            return new ActiveIa();
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
                int ticks = (int)(155 * VariableStorage.TICKS_PER_CM_Z);
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
                int ticks = (int)(20 * VariableStorage.TICKS_PER_CM_Z);
                glisiera.setTargetPosition(-ticks);
                glisiera.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                glisiera.setPower(0.6);
                glisiera1.setTargetPosition(-ticks);
                glisiera1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                glisiera1.setPower(0.6);
                return false;
            }
        }
        public Action GlisieraBara(){
            return new Glisiera.GlisieraBara();
        }

        public class GlisieraBaraJos implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                int ticks = (int)(7 * VariableStorage.TICKS_PER_CM_Z);
                glisiera.setTargetPosition(-ticks);
                glisiera.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                glisiera.setPower(0.3);
                glisiera1.setTargetPosition(-ticks);
                glisiera1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                glisiera1.setPower(0.6);

                return false;
            }
        }
        public Action GlisieraBaraJos(){
            return new Glisiera.GlisieraBaraJos();
        }
    }
    @Override
    public void runOpMode() throws InterruptedException {
        hardwarePapiu robot = new hardwarePapiu(this);
        robot.init();
        beginPose = new Pose2d(-30, -60.5, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        Brat brat = new Brat(hardwareMap);
        Cleste cleste = new Cleste(hardwareMap);
        Intake intake=new Intake(hardwareMap);
        Active active = new Active(hardwareMap);
        Glisiera glisiera = new Glisiera(hardwareMap);
        misumi = hardwareMap.get(DcMotorEx.class , "misumi");
      /*  controller = new PIDController(p,i,d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        target=0; */
        misumi.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        misumi.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        misumi.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Target target=new Target();
        robot.init();
        robot.ServoBrat.setPosition(0.2);
        robot.ServoBrat1.setPosition(0.2);
        waitForStart();
        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .stopAndAdd(cleste.clestestrans())
                        .waitSeconds(0.1)
                        .stopAndAdd(glisiera.GlisieraSus())
                        .strafeToSplineHeading(new Vector2d(-32,-53), Math.toRadians(0))
                        .waitSeconds(1.4)
                        .strafeToSplineHeading(new Vector2d(-59,-52), Math.toRadians(45)) //cos1
                        .waitSeconds(0.1)
                        .stopAndAdd(brat.bratfata())
                        .waitSeconds(0.6)
                        .stopAndAdd(cleste.clestelasat())
                        .waitSeconds(0.3)
                        .stopAndAdd(brat.bratjos())
                        .waitSeconds(0.5)
                        .stopAndAdd(target.target300())
                        .stopAndAdd(glisiera.GlisieraJos())
                        .strafeToSplineHeading(new Vector2d(-49.5,-42), Math.toRadians(90)) //cub1
                        .stopAndAdd(active.activeia())
                        .stopAndAdd(intake.IntakeJos())
                        .strafeToSplineHeading(new Vector2d(-45,-38), Math.toRadians(90))
                        .stopAndAdd(intake.IntakeJosJos())
                        .waitSeconds(1.2)
                        .stopAndAdd(intake.IntakeOut())
                        .waitSeconds(0.2)
                        .stopAndAdd(target.target0())
                        .waitSeconds(0.7)
                        .stopAndAdd(cleste.clestestrans())
                        .waitSeconds(0.1)
                        .stopAndAdd(active.activestop())
                        .stopAndAdd(glisiera.GlisieraSus())
                        .waitSeconds(1.4)
                        .strafeToSplineHeading(new Vector2d(-55,-52), Math.toRadians(45)) //cos2
                        .waitSeconds(0.1)
                        .stopAndAdd(brat.bratfata())
                        .waitSeconds(0.6)
                        .stopAndAdd(cleste.clestelasat())
                        .waitSeconds(0.3)
                        .stopAndAdd(brat.bratjos())
                        .waitSeconds(0.5)
                        .stopAndAdd(target.target300())
                        .stopAndAdd(glisiera.GlisieraJos())
                        .strafeToSplineHeading(new Vector2d(-60,-43.5), Math.toRadians(87)) //cub2
                        .stopAndAdd(active.activeia())
                        .stopAndAdd(intake.IntakeJos())
                        .strafeToSplineHeading(new Vector2d(-55,-41), Math.toRadians(90))
                        .stopAndAdd(intake.IntakeJosJos())
                        .strafeToSplineHeading(new Vector2d(-58,-41), Math.toRadians(90))
                        .waitSeconds(1)
                        .stopAndAdd(intake.IntakeOut())
                        .stopAndAdd(target.target0())
                        .waitSeconds(0.6)
                        .stopAndAdd(cleste.clestestrans())
                        .waitSeconds(0.1)
                        .stopAndAdd(active.activestop())
                        .stopAndAdd(glisiera.GlisieraSus())
                        .waitSeconds(1.4)
                        .strafeToSplineHeading(new Vector2d(-53,-52), Math.toRadians(45)) //cos3
                        .waitSeconds(0.1)
                        .stopAndAdd(brat.bratfata())
                        .waitSeconds(0.6)
                        .stopAndAdd(cleste.clestelasat())
                        .waitSeconds(0.3)
                        .stopAndAdd(brat.bratjos())
                        .waitSeconds(0.5)
                        .stopAndAdd(target.target300())
                        .stopAndAdd(glisiera.GlisieraJos())
                        .strafeToSplineHeading(new Vector2d(-44.5,-18), Math.toRadians(180)) //cub3
                        .stopAndAdd(active.activeia())
                        .stopAndAdd(intake.IntakeJosJos())
                        .waitSeconds(1)
                        .stopAndAdd(intake.IntakeOut())
                        .stopAndAdd(target.target0())
                        .waitSeconds(0.6)
                        .stopAndAdd(cleste.clestestrans())
                        .waitSeconds(0.1)
                        .stopAndAdd(active.activestop())
                        .stopAndAdd(glisiera.GlisieraSus())
                        .waitSeconds(1.4)
                        .strafeToSplineHeading(new Vector2d(-49,-57), Math.toRadians(45)) //cos4
                        .waitSeconds(0.1)
                        .stopAndAdd(brat.bratfata())
                        .waitSeconds(0.6)
                        .stopAndAdd(cleste.clestelasat())
                        .waitSeconds(0.3)
                        .stopAndAdd(brat.bratjos())
                        .waitSeconds(0.5)
                        .build());

    }
}
