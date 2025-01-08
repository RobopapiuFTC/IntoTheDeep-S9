package org.firstinspires.ftc.teamcode.Hardware;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
    public DcMotorEx Ridicare1;
    public DcMotorEx Ridicare2;

    public Servo ServoBrat;
    public Servo Roata1;
    public Servo Roata2;
    public Servo Intake1;
    public Servo Intake2;
    public CRServo IntakeActive;

    public Servo ServoCleste;
    public Servo Cleste;

    public static double down=0,little=1,low=5,middle=32,up=53;
    public static double upr=60,middler=350;
    public static double downm=0,littlem=1,lowm=5,middlem=10,upm=15;
    boolean isOpenR=false, isOpen=true, isOpenI=true, isOpenRI=true,isOpenRR=false,isOpenA=false;
    int i=0;


    public hardwarePapiu(OpMode opmode) {myOpMode = opmode;}



    public void init() {

        leftFront = myOpMode.hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = myOpMode.hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = myOpMode.hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = myOpMode.hardwareMap.get(DcMotorEx.class, "rightBack");
        Glisiera = myOpMode.hardwareMap.get(DcMotorEx.class, "glisiera");
        Ridicare1 = myOpMode.hardwareMap.get(DcMotorEx.class, "ridicare2");
        IntakeActive = myOpMode.hardwareMap.get(CRServo.class, "active");
        /*ServoBrat = myOpMode.hardwareMap.get(Servo.class, "brat");
        Roata1 = myOpMode.hardwareMap.get(Servo.class, "roata1");
        Roata2 = myOpMode.hardwareMap.get(Servo.class, "roata2");
        Intake1 = myOpMode.hardwareMap.get(Servo.class, "intake1");
        Intake2 = myOpMode.hardwareMap.get(Servo.class, "intake2");
        Cleste = myOpMode.hardwareMap.get(Servo.class, "cleste"); */

        //Configurari
       /* Roata1.setPosition(0.33);
        Roata2.setPosition(0.67);
        Glisiera.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Glisiera.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Ridicare1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Ridicare1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Ridicare1.setDirection(DcMotorSimple.Direction.REVERSE);
        //Ridicare2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //Ridicare2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Intake1.setPosition(0.8);
        Intake2.setPosition(0.2);
        ServoBrat.setPosition(1);
        Cleste.setPosition(0.3); */
    }
    public void movement(Gamepad gamepad1){

        double y = gamepad1.left_stick_y;
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
   /* public void miscareservo(Gamepad gamepad1){
        if(gamepad1.x){
            ServoBrat.setPower(1);
        }
        if(gamepad1.b){
            ServoBrat.setPower(-1);
        }
        if(!gamepad1.x && !gamepad1.b)ServoBrat.setPower(0);
    } */
    public void miscaremisumi(String direction, DcMotorEx Glisiera){

        int a=0;

        if(Objects.equals(direction, "up")){
            int ticks = (int)(upm * VariableStorage.TICKS_PER_CM_Z);
            Glisiera.setTargetPosition(-ticks+a);
            Glisiera.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            Glisiera.setPower(0.1);
        } else if(Objects.equals(direction, "down")){
            Glisiera.setTargetPosition((int)(downm * VariableStorage.TICKS_PER_CM_Z)+a);
            Glisiera.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            Glisiera.setPower(0.4);
        } else if(Objects.equals(direction, "middle")){
            int ticks = (int)(middlem * VariableStorage.TICKS_PER_CM_Z);
            Glisiera.setTargetPosition(-ticks+a);
            Glisiera.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            Glisiera.setPower(0.1);
        } else if(Objects.equals(direction, "low")){
            int ticks = (int)(lowm * VariableStorage.TICKS_PER_CM_Z);
            Glisiera.setTargetPosition(-ticks+a);
            Glisiera.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            Glisiera.setPower(0.4);
        } else if(Objects.equals(direction, "little")){
            Glisiera.setTargetPosition((int)(littlem * VariableStorage.TICKS_PER_CM_Z)+a);
            Glisiera.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            Glisiera.setPower(0.1);
        }

    }
    public void miscareridicare(String direction, DcMotorEx Glisiera) {

        int a=0;

        if(Objects.equals(direction, "up")){
            int ticks = (int)(upr * VariableStorage.TICKS_PER_CM_Z);
            Glisiera.setTargetPosition(-ticks+a);
            Glisiera.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            Glisiera.setPower(0.1);
        } else if(Objects.equals(direction, "down")){
            Glisiera.setTargetPosition((int)(downm * VariableStorage.TICKS_PER_CM_Z)+a);
            Glisiera.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            Glisiera.setPower(0.7);
        } else if(Objects.equals(direction, "middle")){
            int ticks = (int)(middler * VariableStorage.TICKS_PER_CM_Z);
            Glisiera.setTargetPosition(-ticks+a);
            Glisiera.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            Glisiera.setPower(0.7);
        } else if(Objects.equals(direction, "low")){
            int ticks = (int)(lowm * VariableStorage.TICKS_PER_CM_Z);
            Glisiera.setTargetPosition(-ticks+a);
            Glisiera.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            Glisiera.setPower(0.1);
        } else if(Objects.equals(direction, "little")){
            Glisiera.setTargetPosition((int)(littlem * VariableStorage.TICKS_PER_CM_Z)+a);
            Glisiera.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            Glisiera.setPower(0.1);
        }

    }
    public void miscareglisiera(String direction, DcMotorEx Glisiera){


        int a=0;
        if(Objects.equals(direction, "up")){
            int ticks = (int)(up * VariableStorage.TICKS_PER_CM_Z);
            Glisiera.setTargetPosition(-ticks+a);
            Glisiera.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            Glisiera.setPower(0.3);
        } else if(Objects.equals(direction, "down")){
            Glisiera.setTargetPosition((int)(down * VariableStorage.TICKS_PER_CM_Z)+a);
            Glisiera.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            Glisiera.setPower(0.3);
        } else if(Objects.equals(direction, "middle")){
            int ticks = (int)(middle * VariableStorage.TICKS_PER_CM_Z);
            Glisiera.setTargetPosition(-ticks+a);
            Glisiera.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            Glisiera.setPower(0.6);
        } else if(Objects.equals(direction, "low")){
            int ticks = (int)(10 * VariableStorage.TICKS_PER_CM_Z);
            Glisiera.setTargetPosition(-ticks+a);
            Glisiera.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            Glisiera.setPower(0.6);
        } else if(Objects.equals(direction, "little")){
            Glisiera.setTargetPosition((int)(little * VariableStorage.TICKS_PER_CM_Z)+a);
            Glisiera.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            Glisiera.setPower(0.1);
        }

    }
    public void rotireintake(){
        try {
            isOpenRI=!isOpenRI;
            if(isOpenRI){ //pt deschis
                Roata1.setPosition(0.33);
                Roata2.setPosition(0.67);
            }
            else{ //pt inchis
                Roata1.setPosition(0.45);
                Roata2.setPosition(0.55);

            }
            TimeUnit.MILLISECONDS.sleep(150);
        } catch (InterruptedException e){
            Thread.currentThread().interrupt();
        }
    }
    public void rotireintakes(){
        try {
                Intake1.setPosition(0.7);
                Intake2.setPosition(0.3);
                isOpenI = true;
            TimeUnit.MILLISECONDS.sleep(150);
        } catch (InterruptedException e){
            Thread.currentThread().interrupt();
        }
    }
    public void brat(){
        try {
            isOpenR=!isOpenR;
            if(isOpenR){ //pt deschis
                ServoBrat.setPosition(0.1);
            }
            else{ //pt inchis
                ServoBrat.setPosition(1);
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
                Cleste.setPosition(0.3);
            }
            TimeUnit.MILLISECONDS.sleep(300);
        } catch (InterruptedException e){
            Thread.currentThread().interrupt();
        }

    }
    public void intake(){
        try {
            isOpenI=!isOpenI;
            if(isOpenI){ //pt inchis
                Intake1.setPosition(1);
                Intake2.setPosition(0);
                Cleste.setPosition(0.5);
                isOpen=true;
            }
            else{ //pt deschis
                Intake1.setPosition(0.36);
                Intake2.setPosition(0.64);
            }
            TimeUnit.MILLISECONDS.sleep(300);
        } catch (InterruptedException e){
            Thread.currentThread().interrupt();
        }

    }
    public void active(){
        try {
            isOpenA=!isOpenA;
            if(isOpenA){ //pt inchis
                IntakeActive.setPower(1);
                IntakeActive.setDirection(DcMotorSimple.Direction.REVERSE);
            }
            else{ //pt deschis
                IntakeActive.setPower(1);
                IntakeActive.setDirection(DcMotorSimple.Direction.FORWARD);
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
    public void ridicaregamepad(Gamepad gamepad1,DcMotorEx Ridicare1, DcMotorEx Ridicare2){

        if(gamepad1.x){
            miscareridicare("middle",Ridicare1);
        }
        if(gamepad1.b){
            miscareridicare("down",Ridicare1);

        }

    }
    public void misumigamepad(Gamepad gamepad1,DcMotorEx Glisiera){

        if(gamepad1.dpad_left){
            miscaremisumi("middle",Glisiera); //pidMisumi.target = 500;
        }
        if(gamepad1.dpad_up){
            miscaremisumi("up",Glisiera);
        }
        if(gamepad1.dpad_down){
            miscaremisumi("down",Glisiera);
        }
        if(gamepad1.dpad_right){
            miscaremisumi("low",Glisiera);
        }

    }
  /**  public void glisieragamepad(Gamepad gamepad1,DcMotorEx Glisiera){
        String[] inaltime = {"down", "little", "low", "middle", "high"};
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
    } **/

}