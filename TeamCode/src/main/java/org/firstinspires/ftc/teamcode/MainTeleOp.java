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
        robot.miscareservo(gamepad1, robot.ServoBrat);
        robot.glisieragamepad(gamepad1, robot.Glisiera);
    }
}