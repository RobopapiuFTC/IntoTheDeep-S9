package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp
public class PIDMisumi extends OpMode{
        private PIDController controller;
        public static double p=0, i=0, d=0;
        public static double f=0;
        public static int target=0;
        public final double ticks_in_degree=700/180.0;
        private DcMotorEx misumi;

    @Override
    public void init(){
        controller = new PIDController(p,i,d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        misumi = hardwareMap.get(DcMotorEx.class, "misumi");
    }
    @Override
    public void loop(){
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
