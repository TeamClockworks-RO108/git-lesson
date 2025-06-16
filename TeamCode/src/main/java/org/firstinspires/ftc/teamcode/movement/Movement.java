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

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.utils.EdgeDetector;

import java.util.Arrays;
import java.util.List;


public class Movement {
    private double maxSpeed = 1;
    private double currentDirection = 0;
    private double currentSpeed = 0;
    private double local_angular_speed;
    private double local_manual_manual_speed;

    private final Follower follower;

    private final EdgeDetector followerEdge = new EdgeDetector(false);

    public EdgeDetector getFollowerEdge() {
        return followerEdge;
    }

    private final Telemetry telemetry;
    private final Pose takePose = new Pose(21.6, 30, Math.toRadians(180)),
            placePose = new Pose(29, 63.813, Math.toRadians(0));

    PathBuilder place;

    public Movement(Follower follower, Telemetry telemetry) {
        this.follower = follower;
        this.telemetry = telemetry;
        follower.setTeleOpMovementVectors(0, 0, 0);
        follower.startTeleopDrive();
        followerEdge.onRelease(follower::startTeleopDrive);

        place = getFollower().pathBuilder()
                .addPath(new BezierLine(
                        new Point(takePose),
                        new Point(placePose)
                ))
                .setLinearHeadingInterpolation(takePose.getHeading(), placePose.getHeading());

    }

    public Movement setMaximumSpeed(double speed) {
        maxSpeed = Range.clip(speed, 0, 1000);
        maxSpeed /= 1000;
        updateMotors();
        return this;
    }

    /**
     * Gets the maximum speed
     *
     * @return the maximum speed
     */
    public double getMaximumSpeed() {
        return maxSpeed * 1000;
    }

    /**
     * Drive the motors
     *
     * @param speed     The speed to move at. Must be in range [0, 1000] Affected by maximum speed
     * @param direction The direction, in angles, relative to current position.
     *                  0 moves forward
     *                  (+) positive angles steer left
     *                  (-) negative angles steer right
     * @return self reference (this). Useful in chaining methods
     */
    public void drive(double speed, double direction, double angular_speed, double manual_angular_speed) {
        currentDirection = direction;
        currentSpeed = Range.clip(speed, 0, 1);
        local_angular_speed = angular_speed;
        local_manual_manual_speed = manual_angular_speed;
        updateMotors();
    }

    private void updateMotors() {

        boolean manual = Math.abs(currentSpeed) > 0.01 || Math.abs(local_manual_manual_speed) > 0.01;
        double direction = currentDirection * Math.PI / 180;

        double rotate = -(local_angular_speed + local_manual_manual_speed); // add - if rotating in reverse
        double x = -Math.sin(direction) * maxSpeed * currentSpeed;
        double y = -Math.cos(direction) * maxSpeed * currentSpeed;

        double max = Math.abs(Math.max(Math.abs(x), Math.abs(y)));
        if (max > 1) {
            x /= max;
            y /= max;
        }


        // if following and manual intervention, break trajectory
        // if not following just do basic teleop task
        boolean busy = follower.isBusy();
        followerEdge.update(busy);
        if (follower.isBusy()) {
            if (manual) {
                follower.startTeleopDrive();
                follower.setTeleOpMovementVectors(y, x, rotate, true);
            }
        } else follower.setTeleOpMovementVectors(y, x, rotate, true);
        telemetry.addData("manual", manual);
        telemetry.addData("rotate", rotate);
        telemetry.addData("x", x);
        telemetry.addData("y", y);
        telemetry.addData("max", max);
        telemetry.addData("busy", busy);

        follower.update();
    }


    public Follower getFollower() {
        return this.follower;
    }

    public void moveToHighChamber() {
        follower.followPath(place.build());
    }


}

