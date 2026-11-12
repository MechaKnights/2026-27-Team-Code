package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;


@Config
public class DriveCodeCommon extends LinearOpMode{



    int GREEN_GREEN_MIN = 80;
    int GREEN_RED_MAX = 70;
    int GREEN_BLUE_MAX = 70;

    double speed = 1.0;


    @Override
    public void runOpMode() throws InterruptedException {
    }

    public void drives(MecanumDrive drive){
        if (gamepad1.right_bumper) {
            speed = 0.5;
        } else {
            speed = 1.0;
        }
        drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                        -gamepad1.left_stick_y*speed,
                        -gamepad1.left_stick_x*speed
                ),
                -gamepad1.right_stick_x*speed
        ));
    }

    public void intake(MecanumDrive drive){
        if(gamepad2.a){
            drive.intakeOne.setPower(1.0);
            drive.intakeTwo.setPower(1.0);
        }
        else {
            drive.intakeOne.setPower(0);
            drive.intakeTwo.setPower(0);
        }
    }
    public void shooter(MecanumDrive drive){
        if(gamepad2.right_trigger > 0.5){
            drive.launcherOne.setPower(-0.5);
        }else{
            drive.launcherOne.setPower(0.0);
        }
        if(gamepad2.dpad_right){
            drive.turret.setPower(0.25);
        } else if (gamepad2.dpad_left) {
            drive.turret.setPower(-0.25);
        }
        else {
            drive.turret.setPower(0);
        }
        if (gamepad2.left_bumper){
            drive.hood.setPosition(drive.hood.getPosition() - .1);
        } else if (gamepad2.right_bumper) {
            drive.hood.setPosition(drive.hood.getPosition()+1);
        }
    }
}