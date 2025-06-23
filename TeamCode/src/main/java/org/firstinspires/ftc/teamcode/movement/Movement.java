package org.firstinspires.ftc.teamcode.movement;

import android.util.Log;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.Point;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;


import java.util.Arrays;
import java.util.List;


public class Movement {
        private DcMotor front_right;
        private DcMotor front_left;
        private DcMotor back_right;
        private DcMotor back_left;

        public Movement(HardwareMap hardwaremap) {
            front_right = hardwaremap.get(DcMotor.class, "front_right");
            front_left = hardwaremap.get(DcMotor.class, "front_left");
            back_left = hardwaremap.get(DcMotor.class, "back_left");
            back_right = hardwaremap.get(DcMotor.class, "back_right");
            front_left.setDirection(DcMotorSimple.Direction.REVERSE);
            back_left.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        public void control_movement(Gamepad gamepad1){
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            double frontLeftPower = (y + x + rx);
            double backLeftPower = (y - x + rx) ;
            double frontRightPower = (y - x - rx);
            double backRightPower = (y + x - rx) ;

            front_left.setPower(frontLeftPower);
            front_right.setPower(frontRightPower);
            back_left.setPower(backLeftPower);
            back_right.setPower(backRightPower);

        }

}