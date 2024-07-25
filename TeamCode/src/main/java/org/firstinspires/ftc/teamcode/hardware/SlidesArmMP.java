package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.usefuls.Math.M;
import org.firstinspires.ftc.teamcode.usefuls.Motor.DcMotorBetter;
import org.firstinspires.ftc.teamcode.usefuls.Motor.PID;


@Config
public class SlidesArmMP {
    public enum SlideState {
        RETRACTED,
        INTAKE,
        DEPOSIT
    }
    private static final double S_LOWER_BOUND = 0; //number in ticks
    private static final double S_UPPER_BOUND = -1100; //80.25 inches
    private static final double INCHES_TO_TICKS = -34.97131; //number in ticks

    private static final double A_LOWER_BOUND = 0;
    private static final double A_UPPER_BOUND = 800; //180 degrees
    public static final double TICKS_DEGREE_CONSTANT = 4.44444444444; //.225

    public static double kp=5;
    private double error;
    private DcMotorBetter s;
    private DcMotorBetter a;
    private DcMotorBetter bl;
    private DcMotorBetter br;


    private PID linSlideController;
    // private PID armController;

    public static double targetLinSlidePosition = 0;
    public static double targetArmPosition = 0;

    public static double sKp = 7, sKi = 3, sKd = .5;
    public static double aKp = 5, aKi = 0, aKd = 0;
    private boolean lowering = false;
    private double targetAngle=0;

    PIDFController armController = new PIDFController(new PIDCoefficients(aKp, aKi, aKd));

    public SlidesArmMP(HardwareMap hardwareMap){
        s = new DcMotorBetter(hardwareMap.get(DcMotorEx.class, "s"));
        a = new DcMotorBetter(hardwareMap.get(DcMotorEx.class, "a"));
        a.setDirection(DcMotorSimple.Direction.REVERSE);
        a.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        a.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        a.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);


        bl = new DcMotorBetter(hardwareMap.get(DcMotorEx.class, "bl"));
        bl.setLowerBound(S_LOWER_BOUND);
        bl.setUpperBound(S_UPPER_BOUND);

        br = new DcMotorBetter(hardwareMap.get(DcMotorEx.class, "br"));
        br.setLowerBound(A_LOWER_BOUND);
        br.setUpperBound(A_UPPER_BOUND);

        this.linSlideController = new PID(new PID.Coefficients(sKp, sKi, sKd),
                () -> (this.bl.getCurrentPosition()) - this.targetLinSlidePosition,
                factor -> {
                    this.s.setPower(M.clamp(factor, .8, -1)); //b is extension
                });

    }
    public SlidesArmMP stopAndResetEncoder() {
        this.br.stopAndResetEncoder();
        this.bl.stopAndResetEncoder();
        return this;
    }
    public double getError(){
        return error;
    }
    public double getCurrentSlidesPosition() {
        return this.bl.getCurrentPosition();
    }
    public double getCurrentArmPosition() {
        return this.br.getCurrentPosition();
    }

    public void setDegrees(double degrees){
        targetAngle=degrees;
        targetArmPosition = (degrees * TICKS_DEGREE_CONSTANT) / A_UPPER_BOUND;
    }
    public double getCurrentDegrees(){
        return (getCurrentArmPosition()/TICKS_DEGREE_CONSTANT) * A_UPPER_BOUND;
    }



    public void setInches(double inches) {
        targetLinSlidePosition = (inches * INCHES_TO_TICKS)  / S_UPPER_BOUND; //untested
    }
    public double getCurrentInches() {
        return  (getCurrentSlidesPosition()/INCHES_TO_TICKS) * S_UPPER_BOUND;
    }
    public double getTargetAngle(){
        return targetAngle;
    }

    double errorFunction(double start, double end, double maxv, double maxa, double maxj, double timeMS) {
        MotionProfile profile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(start, 0, 0),
                new MotionState(end, 0, 0),
                maxv,
                maxa,
                maxj
        );
        MotionState state = profile.get(timeMS/1000);

        armController.setTargetPosition(state.getX());
        armController.setTargetVelocity(state.getV());
        armController.setTargetAcceleration(state.getA());
        return armController.update(a.getCurrentPosition()); // erro
    }

    public void updateMP(double start, double end, double startTime, double currTime){
        setDegrees(end);
        linSlideController.update();
        if(getTargetAngle()==0)
        {
            lowering=true;
        }else{
            lowering=false;
        }
//        error = -1 * getCurrentArmPosition()-targetArmPosition ;
        error = errorFunction(start, targetArmPosition, 20, 10, 10, currTime - startTime);

        double armPower = kp*error;
        if(getCurrentDegrees() < 85 && armPower > .2&&lowering){
            armPower = 0;
        }


        a.setPower(armPower);

        a.update();
        s.update();
    }
}
