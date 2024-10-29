package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import java.util.Objects;
import java.util.concurrent.TimeUnit;

public class hardwarePapiu {

    public final OpMode myOpMode;

    public DcMotorEx leftFront;
    public DcMotorEx leftBack;
    public DcMotorEx rightFront;
    public DcMotorEx rightBack;
    public DcMotorEx Glisiera; //slot 2 pe Control hub 0-1-2-3 order
    public CRServo ServoBrat;
    public Servo ServoCleste;
    public Servo Cleste;

    public static double down=0,little=1,low=5,middle=15,up=25;
    boolean isOpen=false;


    public hardwarePapiu(OpMode opmode) {myOpMode = opmode;}



    public void init() {

        leftFront = myOpMode.hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = myOpMode.hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = myOpMode.hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = myOpMode.hardwareMap.get(DcMotorEx.class, "rightBack");
        Glisiera = myOpMode.hardwareMap.get(DcMotorEx.class, "glisiera");
        ServoBrat = myOpMode.hardwareMap.get(CRServo.class, "brat");
        ServoCleste = myOpMode.hardwareMap.get(Servo.class, "rotire");
        Cleste = myOpMode.hardwareMap.get(Servo.class, "cleste");

        //Configurari
        ServoBrat.setDirection(CRServo.Direction.FORWARD);
        ServoBrat.setPower(0);
        Glisiera.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Glisiera.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Cleste.setPosition(0);
        ServoCleste.setPosition(0);
    }
    public void movement(Gamepad gamepad1){

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x * 1.1;
        double rx = gamepad1.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y - x - rx) / denominator;
        double backLeftPower = (y + x - rx) / denominator;
        double frontRightPower = (-y - x - rx) / denominator;
        double backRightPower = (-y + x - rx) / denominator;


        leftFront.setPower(frontLeftPower);
        leftBack.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightBack.setPower(backRightPower);

    }
    public void miscareservo(Gamepad gamepad1){
        if(gamepad1.x){
            ServoBrat.setPower(1);
        }
        if(gamepad1.b){
            ServoBrat.setPower(-1);
        }
        if(!gamepad1.x && !gamepad1.b)ServoBrat.setPower(0);
    }
    public void miscareglisiera(String direction, DcMotorEx Glisiera){

        int a=0;

        if(Objects.equals(direction, "up")){
            int ticks = (int)(up * VariableStorage.TICKS_PER_CM_Z);
            Glisiera.setTargetPosition(-ticks+a);
            Glisiera.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            Glisiera.setPower(0.1);
        } else if(Objects.equals(direction, "down")){
            Glisiera.setTargetPosition((int)(down * VariableStorage.TICKS_PER_CM_Z)+a);
            Glisiera.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            Glisiera.setPower(0.1);
        } else if(Objects.equals(direction, "middle")){
            int ticks = (int)(middle * VariableStorage.TICKS_PER_CM_Z);
            Glisiera.setTargetPosition(-ticks+a);
            Glisiera.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            Glisiera.setPower(0.1);
        } else if(Objects.equals(direction, "low")){
            int ticks = (int)(low * VariableStorage.TICKS_PER_CM_Z);
            Glisiera.setTargetPosition(-ticks+a);
            Glisiera.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            Glisiera.setPower(0.1);
        } else if(Objects.equals(direction, "little")){
            Glisiera.setTargetPosition((int)(little * VariableStorage.TICKS_PER_CM_Z)+a);
            Glisiera.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            Glisiera.setPower(0.1);
        }

    }
    public void rotirecleste(){
        try {
            isOpen=!isOpen;
            if(isOpen){ //pt deschis
                ServoCleste.setPosition(0.5);
            }
            else{ //pt inchis
                ServoCleste.setPosition(0);
            }
            TimeUnit.MILLISECONDS.sleep(300);
        } catch (InterruptedException e){
            Thread.currentThread().interrupt();
        }

    }
    public void cleste(){
        try {
            isOpen=!isOpen;
            if(isOpen){ //pt inchis
                Cleste.setPosition(0.5);
            }
            else{ //pt deschis
                Cleste.setPosition(0);
            }
            TimeUnit.MILLISECONDS.sleep(300);
        } catch (InterruptedException e){
            Thread.currentThread().interrupt();
        }

    }
    public void glisieragamepad(Gamepad gamepad1,DcMotorEx Glisiera){

        if(gamepad1.dpad_left){
            miscareglisiera("middle",Glisiera);
        }
        if(gamepad1.dpad_up){
            miscareglisiera("up",Glisiera);
        }
        if(gamepad1.dpad_down){
            miscareglisiera("down",Glisiera);
        }
        if(gamepad1.dpad_right){
            miscareglisiera("low",Glisiera);
        }

    }
    /** public void glisieragamepad(Gamepad gamepad1,DcMotorEx Glisiera){
        String[] inaltime = {"down", "little", "low", "middle", "high"};
        int i=0;
        while(i<=inaltime.length){
                if(gamepad1.dpad_left) {
                    if(i<inaltime.length-1)i++;
                    if(i<inaltime.length)miscareglisiera(inaltime[i], Glisiera);

                }
                if(gamepad1.dpad_right) {
                    if(i>0)i--;
                    if(i<inaltime.length)miscareglisiera(inaltime[i], Glisiera);

                }
                if(gamepad1.dpad_down){
                    i=0;
                    miscareglisiera(inaltime[i], Glisiera);
                }
        }
    }
    */
}