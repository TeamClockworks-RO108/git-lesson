package org.firstinspires.ftc.teamcode.outtake;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;



public class Gripper {
    /* Servo Positions */
    private Servo gripper;

    private double openPos = 0.0;
    private double closedPos = 0.9;

    private boolean isOpen = true;
    public Gripper(HardwareMap hardwareMap){
        gripper = hardwareMap.get(Servo.class, "claw_servo");
        gripper.setPosition(openPos);

    }

    public void move_servo(){
        if(isOpen){
            gripper.setPosition(closedPos);
            isOpen =false;
        } else {
            gripper.setPosition(openPos);
            isOpen = true;
        }
    }


}