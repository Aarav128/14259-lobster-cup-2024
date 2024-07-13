
package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.posePID2.DT;



@Autonomous
public class auto extends LinearOpMode {

    enum State {
        PROP,
        AFTERPROP,
        DEPOSIT,
        PARK,
        FINISH
    }
    DT drive;
    boolean timeToggle = true;
    double TimeStamp = 0;
    ElapsedTime timer = new ElapsedTime();
    State currentState;
    Pose2d startPose = new Pose2d(12,-60, Math.toRadians(90));

    Pose2d prop = new Pose2d(12, -50,Math.toRadians(90));
    Pose2d afterProp = new Pose2d(12, -55,Math.toRadians(90));
    Pose2d deposit = new Pose2d(45, -44, Math.toRadians(0));
    Pose2d park = new Pose2d(45,-60, Math.toRadians(0));
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new DT(hardwareMap, new Pose2d(startPose.getX(), startPose.getY(), startPose.getHeading()));
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        currentState = State.PROP;
        waitForStart();

        while(opModeIsActive()){
            switch(currentState){
                case PROP:
                    drive.lineTo(prop.getX(),prop.getY(), prop.getHeading());
                    if(drive.isAtTarget()){
                        if(timeToggle){//timeToggle starts at true by default
                            TimeStamp = timer.milliseconds();
                            timeToggle = false;
                        }
                        if(timer.milliseconds() > TimeStamp + 2000){
                            timeToggle = true;
                            currentState = State.AFTERPROP;

                        }
                    }

                    break;
                case AFTERPROP:
                    drive.lineTo(afterProp.getX(),afterProp.getY(), afterProp.getHeading());
                    if(drive.isAtTarget()){
                        if(timeToggle){//timeToggle starts at true by default
                            TimeStamp = timer.milliseconds();
                            timeToggle = false;
                        }
                        if(timer.milliseconds() > TimeStamp + 2000){
                            timeToggle = true;
                            currentState = State.DEPOSIT;

                        }
                    }

                    break;
                case DEPOSIT:
                    drive.lineTo(deposit.getX(), deposit.getY(), deposit.getHeading());
                    if(drive.isAtTarget()){
                        currentState = State.PARK;
                    }
                    break;
                case PARK:
                    drive.lineTo(park.getX(), park.getY(), park.getHeading());
                    if(drive.isAtTarget()){
                        currentState = State.FINISH;
                    }
                    break;
                case FINISH:
                    break;
            }
            telemetry.addData("at target",drive.isAtTarget());
            telemetry.addData("x", drive.getX());
            telemetry.addData("y", drive.getY());
            telemetry.addData("r", drive.getR());
            telemetry.update();
            drive.update();
        }
    }
}
