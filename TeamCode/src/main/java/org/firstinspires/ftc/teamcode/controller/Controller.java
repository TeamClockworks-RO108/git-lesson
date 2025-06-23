package org.firstinspires.ftc.teamcode.controller;

import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.movement.Movement;
import org.firstinspires.ftc.teamcode.outtake.Gripper;

import pedroPathing.constants.LConstants;


@SuppressWarnings("unused")
@TeleOp(name = "KEBAB")
public class Controller extends OpMode {
    public final Gamepad prevGamepad1 = new Gamepad();
    private final Gamepad prevGamepad2 = new Gamepad();

    private Movement movement;
    private Gripper gripper;

    @Override
    public void init() {
        movement = new Movement(hardwareMap);
        gripper  = new Gripper(hardwareMap);
    }

    // Initialize components after start to prevent movement in init
    @Override
    public void start() {

    }


    @Override
    public void loop() {

        movement.control_movement(gamepad1);

        if(getGamepad1().a && !prevGamepad1.a){
            gripper.move_servo();
        }

        prevGamepad1.copy(getGamepad1());

    }

    protected Gamepad getGamepad1() {
        return gamepad1;
    }
    protected Gamepad getGamepad2() {
        return gamepad2;
    }
    protected Gamepad getConfigGamepad() {
        return new Gamepad();
    }

}
