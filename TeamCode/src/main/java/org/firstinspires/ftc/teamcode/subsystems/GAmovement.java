//package org.firstinspires.ftc.teamcode.movement;
//
//import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
//
//import com.qualcomm.hardware.bosch.BNO055IMU;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.IMU;
//import com.qualcomm.robotcore.util.ReadWriteFile;
//
//import org.checkerframework.checker.units.qual.A;
//import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
//import org.json.JSONObject;
//
//import java.io.File;
//import java.io.IOException;
//import java.util.ArrayList;
//import java.util.List;
//
//public abstract class GAmovement {
//    String telem = "";
//    final int inch = 1892;
//    HardwareMap map;
//    DcMotorEx fl;
//    DcMotorEx fr;
//    DcMotorEx bl;
//    DcMotorEx br;
//    IMU Imu;
//    DcMotorEx Xodo;
//    DcMotorEx Yodo;
//   //`   GAstate s = new GAstate();
//   // GAlocalization localization;
//    int substate = 0;
//    int substateGoal = 0;
//    int ticksThisState = 0;
//    int state = 0;
//    int prevTickSpeed = 0;
//    int speedF = 0;
//    int speedS = 0;
//    int startDistF = 0;
//    int startDistS = 0;
//    int targetDistF;
//    int targetDistS;
//    int[] distanceToDecelF = {0, 210, 430, 770, 790, 1200, 1300, 1760, 1800, 2000};
//    int distanceToDecelS;
//
//    final double A = 0.2;
//    final double N = 1.7;
//    double Ef = 0;
//    double Es = 0;
//    //!!USE ADDITION!!//
//    int[] D = {0, 100, 2000, 4900, 7400, 9800, 12800, 15700, 20900, 26600};
//    double Cf = 0;
//    double Cs = 0;
//    boolean deceleratingf = false;
//    boolean deceleratings = false;
//    int fnegative = 1;
//    int snegative = 1;
//    int stuckcountX = 0;
//    int stuckdisttrackerX = 0;
//    int stuckcountY = 0;
//    int stuckdisttrackerY = 0;
//
//    protected GAmovement() throws IOException {
//    }
//
//    public final void GAtestInit (DcMotorEx front_left, DcMotorEx front_right, DcMotorEx back_left, DcMotorEx back_right, IMU imu, DcMotorEx X_axis_odometer, DcMotorEx Y_axis_odometer) {
//        fl = front_left;
//        fr = front_right;
//        bl = back_left;
//        br = back_right;
//        Imu = imu;
//        Xodo = X_axis_odometer;
//        Yodo = Y_axis_odometer;
// //       localization = new GAlocalization(imu, X_axis_odometer, Y_axis_odometer);
//    }
//    public boolean autoConstructor (final int t) {
//        return true;
//    }
//
//    public final void complete () {
//        substate ++;
//    }
//    public final boolean synchronize () {
//        if (ticksThisState == 0) {
//            substateGoal ++;
//            return true;
//        }
//        return false;
//    }
//
//    public final void update () {
//        if (substate == substateGoal && ticksThisState != 0) {
//            state ++;
//            ticksThisState = 0;
//        }
//        autoConstructor(state);
//        ticksThisState ++;
//    }
//    public final int getState () {
//        return state;
//    }
//    public final int getTicks () {
//        return ticksThisState;
//    }
//    public final void Map (HardwareMap r) {
//        map = r;
//        fl = r.get(DcMotorEx .class, "front left");
//        fr = r.get(DcMotorEx.class, "front right");
//        bl = r.get(DcMotorEx.class, "back left");
//        br = r.get(DcMotorEx.class, "back right");
//
//        fl.setDirection(DcMotorSimple.Direction.REVERSE);
//        bl.setDirection(DcMotorSimple.Direction.REVERSE);
//        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        Xodo = bl;
//        Yodo = fl;
//    }
//    public final void ADMove (int forward, int strafe, double targetSpeed) {
//        double Sf = 0;
//        double Ss = 0;
//        int Y = -Yodo.getCurrentPosition();
//        int X = -Xodo.getCurrentPosition();
//        if (forward < 0) {
//            fnegative = -1;
//        }
//        if (strafe < 0) {
//            snegative = -1;
//        }
//        if (strafe == 0) {
//            Ss = 0;
//        } else {
//            Ss = 0.1*snegative;
//        }
//        if (forward == 0) {
//            Sf = 0;
//        } else {
//            Sf = 0.1*fnegative;
//        }
//        if (synchronize()) {
//            Ef = Y + forward;
//            Es = X + strafe;
//            deceleratingf = false;
//            deceleratings = false;
//        } else {
//            //forward math
//            if (!deceleratingf) {
//                Sf = Math.min(Math.pow(Math.abs(Cf), Math.abs(A)), Math.abs(targetSpeed))*fnegative;
//                if (Math.abs(Ef - Y) <= D[(int) (Sf*10)]) {
//                    deceleratingf = true;
//                }
//            } else {
//                Sf = Math.max(Math.pow(Math.abs(Cf), Math.abs(N)), 0)*fnegative;
//            }
//            //strafe math
//            if (!deceleratings) {
//                Ss = Math.min(Math.pow(Math.abs(Cs), Math.abs(A)), Math.abs(targetSpeed))*snegative;
//                if (Math.abs(Es - X) <= D[(int) (Ss*10)]) {
//                    deceleratings = true;
//                }
//            } else {
//                Ss = Math.max(Math.pow(Math.abs(Cs), Math.abs(N)), 0)*snegative;
//            }
//        }
//        if (Ss >= 1) {
//            Ss = 0.99;
//        }
//        if (Ss <= -1) {
//            Ss = -0.99;
//        }
//        if (Sf >= 1) {
//            Sf = 0.99;
//        }
//        if (Sf <= -1) {
//            Sf = -0.99;
//        }
//        Cs = Ss;
//        Cf = Sf;
//        fl.setPower(Cf + Cs);
//        bl.setPower(Cf - Cs);
//        fr.setPower(Cf - Cs);
//        br.setPower(Cf + Cs);
//        if (Ss == 0 && Sf == 0) {
//            complete();
//        }
//        telem =
//                "Forward position: " + Y +
//                "\nStrafe Position: " + X +
//                "\nForward speed: " + Cf +
//                "\nStrafe speed: " + Cs +
//                "\nForward decel: " + deceleratingf +
//                "\nStrafe decel: " + deceleratings +
//                "\nForward end: " + Ef +
//                "\nStrafe end: " + Es
//        ;
//        DistDataManager(Cf, Y);
//
//        //obstacle failsafe:
//        if (Math.abs(X) - stuckdisttrackerX < 50 && !(forward > 0.1)) {
//            stuckcountX ++;
//        } else {
//            stuckcountX = 0;
//        }
//        if (stuckcountX >= 15) {
//            deceleratingf = true;
//            telem = "Stuck (forward)!";
//        }
//
//        if (Math.abs(Y) - stuckdisttrackerY < 50 && !(strafe > 0.1)) {
//            stuckcountY ++;
//        } else {
//            stuckcountY = 0;
//        }
//        if (stuckcountY >= 15) {
//            deceleratings = true;
//            telem = "Stuck (strafe)!";
//        }
//
//        if (Math.abs(Y) >= Ef) {
//            deceleratingf = true;
//        }
//        if (Math.abs(X) >= Es) {
//            deceleratings = true;
//        }
//    }
//    public final String telemetryData () {
//        return telem;
//    }
//    double prevSpeed;
//    ArrayList<Integer> dists = new ArrayList<>();
//    AvgSpeed average;
//    private double calculateAverage(List<Integer> ds) {
//        return ds.stream()
//                .mapToDouble(d -> d)
//                .average()
//                .orElse(0.0);
//    }
//    int prevDist;
//    private void DistDataManager (double speed, int distance) {
//        if (prevSpeed == speed) {
//            dists.add(distance - prevDist);
//            prevDist = distance;
//        } else {
//            dists.clear();
//            prevDist = 0;
//        }
//        prevSpeed = speed;
//        average = new AvgSpeed(dists.size(), speed, calculateAverage(dists));
//    }
//    public final String DistData () {
//        return "Speed: " + average.currentSpeed + "\nLoop: " + average.loopsThisSpeed + "\nAverage distance for this speed: " + average.avgDistancePerLoop;
//    }
//}
//class AvgSpeed {
//    public int loopsThisSpeed;
//    public double currentSpeed;
//    public double avgDistancePerLoop;
//    public AvgSpeed (int Loop, double CurrentSpeed, double Average) {
//        loopsThisSpeed = Loop;
//        currentSpeed = CurrentSpeed;
//        avgDistancePerLoop = Average;
//    }
//    public int LoopsThisSpeed() {
//        return loopsThisSpeed;
//    }
//    public double CurrentSpeed () {
//        return currentSpeed;
//    }
//    public double AverageDistancePerLoop () {
//        return avgDistancePerLoop;
//    }
//}