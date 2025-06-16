package org.firstinspires.ftc.teamcode.controller;

import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import pedroPathing.constants.LConstants;


@SuppressWarnings("unused")
@TeleOp(name = "FTC S9")
public class Controller extends OpMode {
    private final Gamepad prevGamepad1 = new Gamepad();
    private final Gamepad prevGamepad2 = new Gamepad();




    @Override
    public void init() {
    }

    // Initialize components after start to prevent movement in init
    @Override
    public void start() {

    }


    @Override
    public void loop() {


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
