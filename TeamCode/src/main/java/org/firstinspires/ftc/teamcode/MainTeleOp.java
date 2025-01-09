package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Hardware.hardwarePapiu;

@TeleOp(name = "MainTeleOp", group = "A")
public class MainTeleOp extends OpMode {
    hardwarePapiu robot = new hardwarePapiu(this);
    private PIDController controller;
    public static double p=0.03, i=0, d=0;
    public static double f=0;
    public static int target=0;
    public final double ticks_in_degree=700/180.0;

    @Override
    public void init() {
        robot.init();
        controller = new PIDController(p,i,d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        robot.movement(gamepad1);
        robot.glisieragamepad(gamepad2, robot.Glisiera ,robot.Glisiera1);
        if(gamepad2.b)//robot.intake();
            robot.active();
        if(gamepad2.y)robot.rotireintakes();
        if(gamepad2.x)robot.cleste();
        if(gamepad2.left_bumper)robot.brat();
        robot.misumigamepad(gamepad1, robot.misumi);
        controller.setPID(p,i,d);
       /* int pozitie=misumi.getCurrentPosition();
        double pid = controller.calculate(pozitie, target);
        double ff = Math.cos(Math.toRadians(target/ticks_in_degree)) * f;
        double power = pid+ff;
        misumi.setPower(power);
        telemetry.addData("pos ", pozitie);
        telemetry.addData("target ", target); */
    }
}