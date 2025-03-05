package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import java.util.Objects;
import java.util.concurrent.TimeUnit;

public class hardwarePapiu {

    public final OpMode myOpMode;

    public DcMotorEx leftFront;
    public DcMotorEx leftBack;
    public DcMotorEx rightFront;
    public DcMotorEx rightBack;
    public DcMotorEx Glisiera; //slot 2 pe Control hub 0-1-2-3 order
    public DcMotorEx Glisiera1;
    public DcMotorEx misumi;
    public DcMotorEx Ridicare2;

    public Servo ServoBrat;
    public Servo ServoBrat1;
    public Servo ClesteS;
    public Servo Intake1;
    public Servo Intake2;
    public Servo faras;
    public Servo catarare;
    public DcMotorEx IntakeActive;
    public ColorSensor culoare;

    public Servo ServoCleste;
    public Servo Cleste;

    public static double down=0,little=7,low=40,middle=75,up=160;
    public static double upr=60,middler=350;
    public static double downm=0,littlem=1,lowm=5,middlem=10,upm=21;
    public boolean isOpenR=true, isOpen=true, isOpenI=false, isOpenRI=false,isOpenRR=false,isOpenA=false,isOpenC=false,isOpenF=false;
    public static double red,blue,green,alpha;
    public boolean infata=false, ok=true;
    int i=0;


    public hardwarePapiu(OpMode opmode) {myOpMode = opmode;}



    public void init() {

        leftFront = myOpMode.hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = myOpMode.hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = myOpMode.hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = myOpMode.hardwareMap.get(DcMotorEx.class, "rightBack");
        Glisiera = myOpMode.hardwareMap.get(DcMotorEx.class, "glisiera");
        Glisiera1 = myOpMode.hardwareMap.get(DcMotorEx.class, "glisiera1");
        misumi = myOpMode.hardwareMap.get(DcMotorEx.class, "misumi");
       // ClesteS=myOpMode.hardwareMap.get(Servo.class, "clestes");
        IntakeActive = myOpMode.hardwareMap.get(DcMotorEx.class, "active");
        ServoBrat = myOpMode.hardwareMap.get(Servo.class, "brat");
        ServoBrat1 = myOpMode.hardwareMap.get(Servo.class, "brat1");
        Intake1 = myOpMode.hardwareMap.get(Servo.class, "intake1");
        Intake2 = myOpMode.hardwareMap.get(Servo.class, "intake2");
       Cleste = myOpMode.hardwareMap.get(Servo.class, "cleste");
       faras = myOpMode.hardwareMap.get(Servo.class, "faras");
      // catarare = myOpMode.hardwareMap.get(Servo.class, "catarare");
       culoare = myOpMode.hardwareMap.get(ColorSensor.class, "culoare");
        //Configurari
        Glisiera.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        Glisiera1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        culoare.enableLed(true);
     //   catarare.setPosition(0.5);
        faras.setPosition(0.5);
       Intake1.setPosition(0.95);
        Intake2.setPosition(0.05);
        Cleste.setPosition(0.45);
      //  ClesteS.setPosition(0.26);
        Glisiera1.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        IntakeActive.setDirection(DcMotorSimple.Direction.REVERSE);
        misumi.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        misumi.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void movement(Gamepad gamepad1){

        double drive = -gamepad1.left_stick_y; // Remember, Y stick is reversed!
        double strafe = gamepad1.left_stick_x; // Counteract imperfect strafing
        double turn = gamepad1.right_stick_x;

        double frontLeftPower = (drive + strafe + turn);
        double backLeftPower = (drive - strafe + turn + 0.05);
        double frontRightPower = (drive - strafe - turn);
        double backRightPower = (drive + strafe - turn + 0.15);
        if(gamepad1.right_trigger>0.3){
            leftFront.setPower(frontLeftPower/3);
            leftBack.setPower(backLeftPower/3);
            rightFront.setPower(frontRightPower/3);
            rightBack.setPower(backRightPower/3);
        }else {
            leftFront.setPower(frontLeftPower);
            leftBack.setPower(backLeftPower);
            rightFront.setPower(frontRightPower);
            rightBack.setPower(backRightPower);
        }

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
            Glisiera.setPower(0.5);
            infata=true;
            ok=true;
        } else if(Objects.equals(direction, "down")){
            Glisiera.setTargetPosition((int)(downm * VariableStorage.TICKS_PER_CM_Z)+a);
            Glisiera.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            Glisiera.setPower(0.5);
            infata=false;
        } else if(Objects.equals(direction, "middle")){
            int ticks = (int)(middlem * VariableStorage.TICKS_PER_CM_Z);
            Glisiera.setTargetPosition(-ticks+a);
            Glisiera.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            Glisiera.setPower(0.5);
            ok=true;
        } else if(Objects.equals(direction, "low")){
            int ticks = (int)(lowm * VariableStorage.TICKS_PER_CM_Z);
            Glisiera.setTargetPosition(-ticks+a);
            Glisiera.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            Glisiera.setPower(0.5);
            ok=true;
        } else if(Objects.equals(direction, "little")){
            Glisiera.setTargetPosition((int)(littlem * VariableStorage.TICKS_PER_CM_Z)+a);
            Glisiera.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            Glisiera.setPower(0.1);
        }

    }
    public void miscareridicare(String direction, DcMotorEx Glisiera) {

        int a=0;

        if(Objects.equals(direction, "up")){
            int ticks = (int)(up * VariableStorage.TICKS_PER_CM_Z);
            Glisiera.setTargetPosition(-ticks+a);
            Glisiera.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            Glisiera.setPower(0.1);
        } else if(Objects.equals(direction, "down")){
            Glisiera.setTargetPosition((int)(down * VariableStorage.TICKS_PER_CM_Z)+a);
            Glisiera.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            Glisiera.setPower(0.7);
        } else if(Objects.equals(direction, "middle")){
            int ticks = (int)(middle * VariableStorage.TICKS_PER_CM_Z);
            Glisiera.setTargetPosition(-ticks+a);
            Glisiera.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            Glisiera.setPower(0.7);
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
    public void miscareglisiera(String direction, DcMotorEx Glisiera, DcMotorEx Glisiera1){


        int a=0;
        if(Objects.equals(direction, "up")){
            int ticks = (int)(up * VariableStorage.TICKS_PER_CM_Z);
            Glisiera.setTargetPosition(-ticks+a);
            Glisiera.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            Glisiera.setPower(1);
            Glisiera1.setTargetPosition(-ticks+a);
            Glisiera1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            Glisiera1.setPower(1);
            activestop();
        } else if(Objects.equals(direction, "down")){
            Glisiera.setTargetPosition((int)(down * VariableStorage.TICKS_PER_CM_Z)+a);
            Glisiera.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            Glisiera.setPower(1);
            Glisiera1.setTargetPosition((int)(down * VariableStorage.TICKS_PER_CM_Z)+a);
            Glisiera1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            Glisiera1.setPower(1);
        } else if(Objects.equals(direction, "middle")){
            int ticks = (int)(middle * VariableStorage.TICKS_PER_CM_Z);
            Glisiera.setTargetPosition(-ticks+a);
            Glisiera.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            Glisiera.setPower(1);
            Glisiera1.setTargetPosition(-ticks+a);
            Glisiera1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            Glisiera1.setPower(1);
            activestop();
        } else if(Objects.equals(direction, "low")){
            int ticks = (int)(low * VariableStorage.TICKS_PER_CM_Z);
            Glisiera.setTargetPosition(-ticks+a);
            Glisiera.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            Glisiera.setPower(0.6);
            Glisiera1.setTargetPosition(-ticks+a);
            Glisiera1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            Glisiera1.setPower(0.6);
            activestop();
        } else if(Objects.equals(direction, "little")){
            Glisiera.setTargetPosition((int)(-little * VariableStorage.TICKS_PER_CM_Z)+a);
            Glisiera.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            Glisiera.setPower(0.4);
            Glisiera1.setTargetPosition((int)(-little * VariableStorage.TICKS_PER_CM_Z)+a);
            Glisiera1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            Glisiera1.setPower(0.4);
        }

    }
    public void rotireintakes(){
        try {
            isOpenI=!isOpenI;
            if(isOpenI){ //pt deschis
                Intake1.setPosition(0.81);
                Intake2.setPosition(0.19);
            }
            else{ //pt inchis
                Intake1.setPosition(0.95);
                Intake2.setPosition(0.05);
            }
            TimeUnit.MILLISECONDS.sleep(150);
        } catch (InterruptedException e){
            Thread.currentThread().interrupt();
        }
    }
    public void farasr(){
        try {
            isOpenF=!isOpenF;
            if(isOpenF){ //pt deschis
                faras.setPosition(0.93);
            }
            else{ //pt inchis
                faras.setPosition(0.5);
            }
            TimeUnit.MILLISECONDS.sleep(150);
        } catch (InterruptedException e){
            Thread.currentThread().interrupt();
        }
    }
    public void brat(){
        try {
            isOpenR=!isOpenR;
            if(isOpenR){ //pt deschis
                ServoBrat.setPosition(0.85);
                ServoBrat1.setPosition(0.85);
            }
            else{ //pt inchis
                ServoBrat.setPosition(0.19);
                ServoBrat1.setPosition(0.19);
            }
            TimeUnit.MILLISECONDS.sleep(300);
        } catch (InterruptedException e){
            Thread.currentThread().interrupt();
        }

    }
    public void clestespecimene(){
        try {
            isOpenR=!isOpenR;
            if(isOpenR){ //pt deschis
                Cleste.setPosition(0.65);
            }
            else{ //pt inchis
                Cleste.setPosition(0.23);
            }
            TimeUnit.MILLISECONDS.sleep(300);
        } catch (InterruptedException e){
            Thread.currentThread().interrupt();
        }

    }
    public void bratspecimene(){
        try {
            isOpenR=!isOpenR;
            if(isOpenR){ //pt deschis
                ServoBrat.setPosition(0.99);
                ServoBrat1.setPosition(0.99);
            }
            else{ //pt inchis
                ServoBrat.setPosition(0.55);
                ServoBrat1.setPosition(0.55);
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
                Cleste.setPosition(0.45);
            }
            else{ //pt deschis
                Cleste.setPosition(0.23);
            }
            TimeUnit.MILLISECONDS.sleep(300);
        } catch (InterruptedException e){
            Thread.currentThread().interrupt();
        }

    }
    public void clestes(){
        try {
            isOpenC=!isOpenC;
            if(isOpenC){ //pt inchis
                ClesteS.setPosition(0.5);
            }
            else{ //pt deschis
                ClesteS.setPosition(0.26);
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
    public void activestop(){
        try {//pt inchis
                IntakeActive.setPower(0);
                isOpenA=false;
            IntakeActive.setDirection(DcMotorSimple.Direction.REVERSE);
            TimeUnit.MILLISECONDS.sleep(300);
        } catch (InterruptedException e){
            Thread.currentThread().interrupt();
        }

    }

    public void glisieragamepad(Gamepad gamepad1,DcMotorEx Glisiera, DcMotorEx Glisiera1){

        if(gamepad1.dpad_left){
            miscareglisiera("middle",Glisiera,Glisiera1);
        }
        if(gamepad1.dpad_up){
            miscareglisiera("up",Glisiera,Glisiera1);
        }
        if(gamepad1.dpad_down){
            miscareglisiera("down",Glisiera,Glisiera1);
        }
        if(gamepad1.dpad_right){
            miscareglisiera("low",Glisiera,Glisiera1);
        }

    }
    public void ridicaregamepad(Gamepad gamepad1,DcMotorEx Ridicare1){

        if(gamepad1.dpad_left){
            miscareridicare("middle",Ridicare1);
        }
        if(gamepad1.dpad_up){
            miscareridicare("up",Ridicare1);
        }
        if(gamepad1.dpad_down){
            miscareridicare("down",Ridicare1);
        }
        if(gamepad1.dpad_right){
            miscareridicare("low",Ridicare1);
        }
    }
    public void misumigamepad(Gamepad gamepad1,DcMotorEx Glisiera,DcMotorEx Glisiera1,DcMotorEx Glisiera2){

        if(gamepad1.dpad_left){
            miscaremisumi("middle",Glisiera); //pidMisumi.target = 500;
        }
        if(gamepad1.dpad_up){
            miscaremisumi("up",Glisiera);
            IntakeActive.setDirection(DcMotorSimple.Direction.REVERSE);
            IntakeActive.setPower(1);
        }
        if(gamepad1.dpad_down){
            miscaremisumi("down",Glisiera);
           // miscareglisiera("little",Glisiera1,Glisiera2);
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