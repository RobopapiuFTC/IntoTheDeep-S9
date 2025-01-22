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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
        private Servo ServoBrat;
        private Servo ServoBrat1;
        public Brat(HardwareMap hardwareMap){
            ServoBrat = hardwareMap.get(Servo.class, "brat");
            ServoBrat1 = hardwareMap.get(Servo.class, "brat1");
        }
        public class BratFata implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                ServoBrat.setPosition(0.2);
                ServoBrat1.setPosition(0.8);

                return false;
            }
        }
        public Action bratfata(){
            return new Brat.BratFata();
        }
        public class BratAuto implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                ServoBrat.setPosition(0.2);
                ServoBrat1.setPosition(0.8);

                return false;
            }
        }
        public Action bratauto(){
            return new Brat.BratAuto();
        }

        public class BratJos implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                ServoBrat.setPosition(0.98); //1
                ServoBrat1.setPosition(0.02);

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
                cleste.setPosition(0.3);

                return false;
            }
        }
        public Action clestestrans(){
            return new Cleste.ClesteStrans();
        }
        public class ClesteLasat implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                cleste.setPosition(0.5);

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
                intake1.setPosition(0.67);
                intake2.setPosition(0.33);

                return false;
            }
        }
        public Action IntakeJos(){
            return new Intake.IntakeJos();
        }
        public class IntakeOut implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                intake1.setPosition(0.85);
                intake2.setPosition(0.15);

                return false;
            }
        }
        public Action IntakeOut(){
            return new Intake.IntakeOut();
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
                int ticks = (int)(35 * VariableStorage.TICKS_PER_CM_Z);
                glisiera.setTargetPosition(-ticks);
                glisiera.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                glisiera.setPower(0.6);
                glisiera1.setTargetPosition(-ticks);
                glisiera1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                glisiera1.setPower(0.6);

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
                glisiera.setPower(0.6);
                glisiera1.setTargetPosition(-ticks);
                glisiera1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                glisiera1.setPower(0.6);

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
                int ticks = (int)(12 * VariableStorage.TICKS_PER_CM_Z);
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
    public class Clestes{
        private Servo clestes;
        public Clestes(HardwareMap hardwareMap){
            clestes = hardwareMap.get(Servo.class, "clestes");
        }
        public class ClesteStrans implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                clestes.setPosition(0.3);

                return false;
            }
        }
        public Action clestestrans(){
            return new Clestes.ClesteStrans();
        }
        public class ClesteLasat implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                clestes.setPosition(0.5);

                return false;
            }
        }
        public Action clestelasat(){
            return new Clestes.ClesteLasat();
        }
    }
    @Override

    public void runOpMode() throws InterruptedException {

        Pose2d beginPose = new Pose2d(9, -61, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        hardwarePapiu robot = new hardwarePapiu(this);
        Brat brat = new Brat(hardwareMap);
        Cleste cleste = new Cleste(hardwareMap);
        Intake intake=new Intake(hardwareMap);
        Active active = new Active(hardwareMap);
        Glisiera glisiera = new Glisiera(hardwareMap);
        Clestes clestes = new Clestes(hardwareMap);
        misumi = hardwareMap.get(DcMotorEx.class , "misumi");
       /* controller = new PIDController(p,i,d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        target=0; */
        misumi.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        misumi.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        misumi.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Target target=new Target();
        robot.init();
        waitForStart();
        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .stopAndAdd(glisiera.GlisieraSus())
                        .waitSeconds(0.3)
                        .strafeTo(new Vector2d(7,-30))
                        .stopAndAdd(glisiera.GlisieraBara())
                        .waitSeconds(1)
                        .stopAndAdd(clestes.clestelasat())
                        .waitSeconds(0.2)
                        .stopAndAdd(glisiera.GlisieraJos())
                        .strafeToSplineHeading(new Vector2d(32,-50), Math.toRadians(30))
                        .strafeToSplineHeading(new Vector2d(32,-30), Math.toRadians(30))
                        .stopAndAdd(target.target150())
                        .stopAndAdd(intake.IntakeJos())
                        .stopAndAdd(active.activeia())
                        .waitSeconds(0.5)
                        .strafeToSplineHeading(new Vector2d(35,-45), Math.toRadians(300))
                        .stopAndAdd(active.activescoate())
                        .waitSeconds(0.5)
                        .stopAndAdd(active.activeia())
                        .strafeToSplineHeading(new Vector2d(43,-30), Math.toRadians(25))
                        .stopAndAdd(intake.IntakeJos())
                        .stopAndAdd(active.activeia())
                        .waitSeconds(0.5)
                        .strafeToSplineHeading(new Vector2d(45,-45), Math.toRadians(300))
                        .stopAndAdd(active.activescoate())
                        .waitSeconds(0.5)
                        .stopAndAdd(target.target0())
                        .stopAndAdd(intake.IntakeOut())
                        .stopAndAdd(active.activestop())
                        .strafeToSplineHeading(new Vector2d(35,-60), Math.toRadians(90))
                        .strafeToSplineHeading(new Vector2d(35,-65), Math.toRadians(90))
                        .stopAndAdd(clestes.clestestrans())
                        .waitSeconds(0.2)
                        .stopAndAdd(glisiera.GlisieraSus())
                        .waitSeconds(0.5)
                        .strafeToSplineHeading(new Vector2d(4,-31), Math.toRadians(270))
                        .stopAndAdd(glisiera.GlisieraBara())
                        .waitSeconds(1)
                        .stopAndAdd(clestes.clestelasat())
                        .waitSeconds(0.2)
                        .stopAndAdd(glisiera.GlisieraJos())
                        .strafeToSplineHeading(new Vector2d(35,-60), Math.toRadians(90))
                        .strafeToSplineHeading(new Vector2d(35,-65), Math.toRadians(90))
                        .stopAndAdd(clestes.clestestrans())
                        .waitSeconds(0.2)
                        .stopAndAdd(glisiera.GlisieraSus())
                        .waitSeconds(0.5)
                        .strafeToSplineHeading(new Vector2d(1,-31), Math.toRadians(270))
                        .stopAndAdd(glisiera.GlisieraBara())
                        .waitSeconds(1)
                        .stopAndAdd(clestes.clestelasat())
                        .waitSeconds(0.2)
                        .stopAndAdd(glisiera.GlisieraJos())
                        .strafeToSplineHeading(new Vector2d(35,-60), Math.toRadians(90))
                        .build());

    }
}
