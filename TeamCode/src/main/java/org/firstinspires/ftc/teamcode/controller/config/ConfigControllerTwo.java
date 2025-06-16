package org.firstinspires.ftc.teamcode.controller.config;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.controller.Controller;

public class ConfigControllerTwo extends Controller {
    @Override
    protected Gamepad getGamepad1() {
        return super.getGamepad1();
    }

    @Override
    protected Gamepad getGamepad2() {
        return new Gamepad();
    }

    @Override
    protected Gamepad getConfigGamepad() {
        return gamepad2;
    }
}
