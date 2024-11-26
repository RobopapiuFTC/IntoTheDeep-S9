package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.hardwarePapiu;

@TeleOp(name = "MainTeleOp", group = "A")
public class MainTeleOp extends OpMode {
    hardwarePapiu robot = new hardwarePapiu(this);

    @Override
    public void init() {
        robot.init();
    }

    @Override
    public void loop() {
        robot.movement(gamepad1);
        robot.glisieragamepad(gamepad1, robot.Glisiera);
        robot.misumigamepad(gamepad2, robot.Misumi);
        robot.ridicaregamepad(gamepad2, robot.Ridicare1, robot.Ridicare2);
        if(gamepad1.x)robot.intake();
        if(gamepad1.left_bumper)robot.brat();
        if(gamepad1.y)robot.cleste();
        if(gamepad1.a)robot.rotireintake();
        if(gamepad1.b)robot.rotireintakescos();

    }
}