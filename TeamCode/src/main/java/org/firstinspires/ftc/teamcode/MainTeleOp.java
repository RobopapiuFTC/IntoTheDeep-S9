package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Hardware.hardwarePapiu;

@TeleOp(name = "MainTeleOp", group = "A")
public class MainTeleOp extends OpMode {
    hardwarePapiu robot = new hardwarePapiu(this);
    private PIDController controller;
    public static double p=0.03, i=0, d=0;
    public static double f=0;
    public static int target=0;
    public final double ticks_in_degree=700/180.0;
    private DcMotorEx misumi;

    @Override
    public void init() {
        robot.init();
        controller = new PIDController(p,i,d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        misumi = hardwareMap.get(DcMotorEx.class, "misumi");
        misumi.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        misumi.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        misumi.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        robot.movement(gamepad1);
        robot.glisieragamepad(gamepad1, robot.Glisiera);
        robot.ridicaregamepad(gamepad2, robot.Ridicare1, robot.Ridicare2);
        if(gamepad1.x)robot.intake();
        if(gamepad1.left_bumper)robot.brat();
        if(gamepad1.y)robot.cleste();
        if(gamepad1.a)robot.rotireintake();
        if(gamepad2.dpad_down)target=5;
        if(gamepad2.dpad_left)target=200;
        if(gamepad2.dpad_right)target=600;
        if(gamepad2.dpad_up)target=1000;
        controller.setPID(p,i,d);
        int pozitie=misumi.getCurrentPosition();
        double pid = controller.calculate(pozitie, target);
        double ff = Math.cos(Math.toRadians(target/ticks_in_degree)) * f;
        double power = pid+ff;
        misumi.setPower(power);
        telemetry.addData("pos ", pozitie);
        telemetry.addData("target ", target);
    }
}