package org.firstinspires.ftc.teamcode.Hardware;

import com.acmerobotics.dashboard.config.Config;

@Config
public class VariableStorage {

    /** Varibile pentru facut automatizare la teleop */

    public static double PI = 3.1415;
    public double GEAR_MOTOR_40_TICKS = 1120;
    public double GEAR_MOTOR_ORBITAL20_TICKS = 537.6;
    public static double GEAR_MOTOR_GOBILDA5202_TICKS = 537.7;
    public static double WHEEL_DIAMETER_CM = 4;
    //double TICKS_PER_CM_Z = GEAR_MOTOR_40_TICKS / (WHEEL_DIAMETER_CM * PI);
    public static double TICKS_PER_CM_Z = GEAR_MOTOR_GOBILDA5202_TICKS / (WHEEL_DIAMETER_CM * PI);

    public static double cOpen1=1,cOpen2=0,cClosed1=0.42,cClosed2=0.58;
    public static double sOpen1=0,sOpen2=0,sClosed1=0.3,sClosed2=0.6;
    public static double down=0,auto=2,low=5,middle=15,up=25;
    public static double Open=0.1, Closed=0.55;
    public static double OpenA=0.5, ClosedA=0;

    public static class ClesteConfig {
        public double cOpen1;
        public double cOpen2;
        public double cClosed1;
        public double cClosed2;
        public ClesteConfig(double cOpen1, double cOpen2, double cClosed1, double cClosed2){
            this.cOpen1 = cOpen1;
            this.cOpen2 = cOpen2;
            this.cClosed1 = cClosed1;
            this.cClosed2 = cClosed2;
        }
    }
    public static class CutieConfig {
        public double sOpen1;
        public double sOpen2;
        public double sClosed1;
        public double sClosed2;
        public CutieConfig(double sOpen1, double sOpen2, double sClosed1, double sClosed2){
            this.sOpen1 = sOpen1;
            this.sOpen2 = sOpen2;
            this.sClosed1 = sClosed1;
            this.sClosed2 = sClosed2;
        }
    }
    public static class TavaConfig {
        public double Open;
        public double Closed;
        public TavaConfig(double Open, double Closed){
            this.Open = Open;
            this.Closed = Closed;
        }
    }
    public static class AvionConfig {
        public double OpenA;
        public double ClosedA;
        public AvionConfig(double OpenA, double ClosedA){
            this.OpenA = OpenA;
            this.ClosedA = ClosedA;
        }
    }
    public static class SlideConfig {
        public double down;
        public double low;
        public double middle;
        public double up;
        public double auto;
        public SlideConfig(double down, double low, double middle, double up,double auto){
            this.down = down;
            this.low = low;
            this.middle = middle;
            this.up = up;
            this.auto = auto;
        }
    }

    public VariableStorage(){}
}
