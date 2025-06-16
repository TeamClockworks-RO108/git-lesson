package org.firstinspires.ftc.teamcode.outtake;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.AngleMapper;

public class Gripper {
    /* Servo Positions */
    private double posGripperClose = 0.7;
    private double posGripperOpen = 0.5;
    private double posJointBasket = AngleMapper.map(0, 255,190),
            posJointTake = AngleMapper.map(0, 255, 110),
            posJointHang = AngleMapper.map(0, 255, 110),
            posJointPark = 0.1754,
            posJointParkLift = 0.686;
    private double posWristBasket = 0.53,
            posWristTake = 0.64,
            posWristHang = 0.605,
            posWristPark = 0.6;

    /* Components */
    private Servo gripperServo;
    private final Servo[] rotServos = new Servo[2];
    private ButtonSensor buttonSensor;

    /* Other */
    private boolean isIn = false;

    public Gripper(HardwareMap hardwareMap) {
        gripperServo = hardwareMap.get(Servo.class, "Out Gripper Servo");
        rotServos[0] = hardwareMap.get(Servo.class, "Out Rot Servo Joint");
        rotServos[1] = hardwareMap.get(Servo.class, "Out Rot Servo Wrist");
        buttonSensor = new ButtonSensor();
        buttonSensor.init(hardwareMap);

        rotServos[1].setDirection(Servo.Direction.REVERSE);
        gripperServo.setPosition(posGripperClose);

        rotServos[0].setPosition(posJointHang);
        rotServos[1].setPosition(posWristHang);
    }

    /* Release & Hold */
    public void release(){
        gripperServo.setPosition(posGripperOpen);
    }
    public void hold(){
        gripperServo.setPosition(posGripperClose);
    }

    /* Position Commands */
    public void goPark(){
        rotServos[0].setPosition(posJointPark);
        rotServos[1].setPosition(posWristPark);
        isIn = true;
    }
    public void goTakeSpecimen(){
        rotServos[0].setPosition(posJointTake);
        rotServos[1].setPosition(posWristTake);
        isIn = false;
    }
    public void goPlaceBasket(){
        rotServos[0].setPosition(posJointBasket);
        rotServos[1].setPosition(posWristBasket);
        isIn= false;
    }
    public void goParkLift(){
        rotServos[0].setPosition(posJointParkLift);
        rotServos[1].setPosition(posWristBasket);
        isIn= false;
    }
    public void goPlaceSpecimen(){
        rotServos[0].setPosition(posJointHang);
        rotServos[1].setPosition(posWristHang);
        isIn= false;
    }

    /* Getters & Setters */
    public boolean isIn(){
        return isIn;
    }
    public double posGripperClose() {
        return posGripperClose;
    }

    public void posGripperClose(double posGripperClose) {
        this.posGripperClose = posGripperClose;
    }

    public double posGripperOpen() {
        return posGripperOpen;
    }

    public void posGripperOpen(double posGripperOpen) {
        this.posGripperOpen = posGripperOpen;
    }

    public double posJointBasket() {
        return posJointBasket;
    }

    public void posJointBasket(double posJointBasket) {
        this.posJointBasket = posJointBasket;
    }

    public double posJointTake() {
        return posJointTake;
    }

    public void posJointTake(double posJointTake) {
        this.posJointTake = posJointTake;
    }

    public double posJointHang() {
        return posJointHang;
    }

    public void posJointHang(double posJointHang) {
        this.posJointHang = posJointHang;
    }

    public double posJointPark() {
        return posJointPark;
    }

    public void posJointPark(double posJointPark) {
        this.posJointPark = posJointPark;
    }

    public double posWristBasket() {
        return posWristBasket;
    }

    public void posWristBasket(double posWristBasket) {
        this.posWristBasket = posWristBasket;
    }

    public double posWristTake() {
        return posWristTake;
    }

    public void posWristTake(double posWristTake) {
        this.posWristTake = posWristTake;
    }

    public double posWristHang() {
        return posWristHang;
    }

    public void posWristHang(double posWristHang) {
        this.posWristHang = posWristHang;
    }

    public double posWristPark() {
        return posWristPark;
    }

    public void posWristPark(double posWristPark) {
        this.posWristPark = posWristPark;
    }
    public boolean buttonIsPressed(){
        return buttonSensor.getState();
    }


    public static class ButtonSensor{
        private DigitalChannel digitalTouch;

        public void init(HardwareMap hardwareMap){
            this.digitalTouch = hardwareMap.get(DigitalChannel.class, "Gripper Button Sensor");
        }

        public boolean getState() {
          return digitalTouch.getState();

        }
    }

}