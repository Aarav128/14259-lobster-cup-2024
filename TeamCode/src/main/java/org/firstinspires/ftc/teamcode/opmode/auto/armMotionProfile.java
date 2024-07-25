package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.posePID2.DT;
import org.firstinspires.ftc.teamcode.hardware.SlidesArmMP;

@TeleOp
@Config
public class armMotionProfile extends LinearOpMode{
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        motor = hardwareMap.get(DcMotorEx.class, "motor1");
//        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//        FtcDashboard dashboard = FtcDashboard.getInstance();
//        Telemetry telemetry = dashboard.getTelemetry();
//        waitForStart();
//        telemetry.addData("velocity", 0);
//        telemetry.addData("power", 0);
//        telemetry.update();
//        sleep(5000);
//        while (opModeIsActive()) {
//            moveMotionProfile(0, 100, 10, 2, 4, 100000);
//        }
//
//    }
public static double degreeTarget = 15;
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        DT drive = new DT(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        SlidesArmMP slidesArm = new SlidesArmMP(hardwareMap);
        slidesArm.stopAndResetEncoder();

        waitForStart();
        double start = slidesArm.getCurrentArmPosition();
        double elapsedTime = System.currentTimeMillis();
        while (opModeIsActive()){
            drive.setPowers(0, 0, 0);
            slidesArm.setDegrees(degreeTarget);
            slidesArm.updateMP(start, degreeTarget, elapsedTime, System.currentTimeMillis());
            telemetry.addData("angle", slidesArm.getCurrentDegrees());
            telemetry.addData("error", slidesArm.getError());
            telemetry.update();
        }
    }
}
