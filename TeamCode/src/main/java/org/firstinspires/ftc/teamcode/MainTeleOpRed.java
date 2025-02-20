package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Hardware.hardwarePapiu;

@TeleOp(name = "MainTeleOpRed", group = "A")
public class MainTeleOpRed extends OpMode {
    hardwarePapiu robot = new hardwarePapiu(this);
    private PIDController controller;
    public static double p=0.01, i=0, d=0;
    public static double f=0;
    public static int target=0;
    public final double ticks_in_degree=700/180.0;
    public DcMotorEx misumi;
    public static double red,blue,green;
    public boolean trebe=true,ok=true;

    @Override
    public void init() {
        robot.init();
    }

    @Override
    public void loop() {
        robot.movement(gamepad1);
        robot.glisieragamepad(gamepad2, robot.Glisiera ,robot.Glisiera1);
        robot.misumigamepad(gamepad1,robot.misumi,robot.Glisiera,robot.Glisiera1);
        if(gamepad1.left_bumper)robot.farasr();
        if(gamepad2.b){
            robot.IntakeActive.setDirection(DcMotorSimple.Direction.FORWARD);
            robot.IntakeActive.setPower(1);
        }
        if(gamepad1.b){
            robot.ServoBrat.setPosition(0.45);
            robot.ServoBrat1.setPosition(0.45);
        }
        if(gamepad1.a)robot.bratspecimene();
        if(gamepad1.x)robot.clestespecimene();
        if(gamepad1.y)robot.rotireintakes();
        if(gamepad2.x)robot.cleste();
        if(gamepad2.left_bumper)robot.brat();
        if(gamepad1.dpad_up)robot.infata=true;
        red = robot.culoare.red();
        blue = robot.culoare.blue();
        green = robot.culoare.green();
        if(robot.infata=true) {
            if ((red >= 500 && blue <= 500 && green <= 500) || (red >= 500 && blue <= 500 && green >= 500) && trebe) {
                robot.Intake1.setPosition(0.95);
                robot.Intake2.setPosition(0.05);
                robot.miscaremisumi("down", robot.misumi);
                robot.isOpen = true;
                robot.infata=false;
            } else if (red <= 500 && blue >= 500 && green <= 500) {
                robot.IntakeActive.setDirection(DcMotorSimple.Direction.FORWARD);
                robot.IntakeActive.setPower(1);
            }
        }
        telemetry.addData("red",red);
        telemetry.addData("blue",blue);
        telemetry.addData("green",green);
        telemetry.addData("infata", robot.infata);
        telemetry.update();
    }
}