package org.firstinspires.ftc.teamcode.controller.config;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.controller.Controller;

@TeleOp(name = "Config Controller 1")
public class ConfigControllerOne extends Controller {
    @Override
    protected Gamepad getGamepad1() {
        return new Gamepad();
    }

    @Override
    protected Gamepad getGamepad2() {
        return super.getGamepad2();
    }

    @Override
    protected Gamepad getConfigGamepad() {
        return gamepad1;
    }
}
