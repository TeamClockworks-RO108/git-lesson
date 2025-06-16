package org.firstinspires.ftc.teamcode.controller;

import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.algo.PIDFLinear;
import org.firstinspires.ftc.teamcode.intake.Intake;
import org.firstinspires.ftc.teamcode.movement.AngleHelper;
import org.firstinspires.ftc.teamcode.movement.GamepadMovementInput;
import org.firstinspires.ftc.teamcode.movement.IMUSensor;
import org.firstinspires.ftc.teamcode.movement.Movement;
import org.firstinspires.ftc.teamcode.algo.PID;
import org.firstinspires.ftc.teamcode.movement.RotationController;
import org.firstinspires.ftc.teamcode.outtake.Gearbox;
import org.firstinspires.ftc.teamcode.outtake.Lift;
import org.firstinspires.ftc.teamcode.outtake.Gripper;
import org.firstinspires.ftc.teamcode.outtake.Outtake;
import org.firstinspires.ftc.teamcode.utils.EdgeDetector;
import org.firstinspires.ftc.teamcode.utils.ParameterChanger;
import org.firstinspires.ftc.teamcode.utils.ServoPositionTuner;
import org.firstinspires.ftc.teamcode.utils.Signals;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;


@SuppressWarnings("unused")
@TeleOp(name = "FTC S9")
public class Controller extends OpMode {
    private final Gamepad prevGamepad1 = new Gamepad();
    private final Gamepad prevGamepad2 = new Gamepad();

    private ParameterChanger parameterChanger = null;

    private RotationController rotationController = null;

    protected boolean blueAlliance;

    private final GamepadMovementInput gamepadInput1 = new GamepadMovementInput();
    private final GamepadMovementInput gamepadInput2 = new GamepadMovementInput();
    private final EdgeDetector rotateLeftButton = new EdgeDetector(false);
    private final EdgeDetector rotateRightButton = new EdgeDetector(false);
    private final EdgeDetector rotateBackButton = new EdgeDetector(false);
    private final EdgeDetector rotateFwdButton = new EdgeDetector(false);

    private final EdgeDetector ejectButton = new EdgeDetector(false);
    private final EdgeDetector gatherButton = new EdgeDetector(false);
    private final EdgeDetector returnButtonIntake = new EdgeDetector(false);

    private final EdgeDetector driverEjectButton = new EdgeDetector(false);
    private final EdgeDetector driverGatherButton = new EdgeDetector(false);
    private final EdgeDetector driverReturnButtonIntake = new EdgeDetector(false);
    private final EdgeDetector driverReturnButton2Intake = new EdgeDetector(false);
    private final EdgeDetector goToBasket = new EdgeDetector(false);
    private final EdgeDetector command1 = new EdgeDetector(false);
    private final EdgeDetector returnButtonOuttake = new EdgeDetector(false);

    private final EdgeDetector basketButton = new EdgeDetector(false);

    private final EdgeDetector transferButton = new EdgeDetector(false);

    private final EdgeDetector resetEncodersButton = new EdgeDetector(false);

    private final EdgeDetector resetHeadingButton = new EdgeDetector(false);

    private final EdgeDetector parkLift = new EdgeDetector(false);

    private final EdgeDetector changeGear = new EdgeDetector(false);

    private final EdgeDetector liftToHang = new EdgeDetector(false);

    private final EdgeDetector goToHang = new EdgeDetector(false);

    private final EdgeDetector moveToHighChamber = new EdgeDetector(false);

    private final EdgeDetector extendSmallJoint = new EdgeDetector(false);

    private final EdgeDetector scoreInLowBasket = new EdgeDetector(false);

    private ServoPositionTuner servoPositionTuner = null;


    private Movement movement = null;
    private IMUSensor imuSensor = null;

    private Lift lift = null;
    private Gripper gripper;
    private Gearbox gearbox;
    private Intake intake;
    private Outtake outtake;
    private Signals signals;


    @Override
    public void init() {
        Constants.setConstants(FConstants.class, LConstants.class);

        // Create the parameter changer
        parameterChanger = new ParameterChanger(getConfigGamepad(), telemetry);

        // Initialize sensors
        gamepadInput1.init(getGamepad1());
        gamepadInput2.init(getGamepad2());
        imuSensor = new IMUSensor(telemetry, hardwareMap, "imu", null);

        signals = new Signals(telemetry);

        // Initialize movement associated buttons
        movement = new Movement(new com.pedropathing.follower.Follower(hardwareMap), telemetry);

        rotationController = new RotationController(movement, imuSensor, telemetry);

        // Initialize intake
        intake = new Intake(hardwareMap, signals, telemetry, blueAlliance);

        setupRotationButtons();
        setupLiftButtons();
        setupIntakeButtons();

        prevGamepad1.copy(getGamepad1());
        prevGamepad2.copy(getGamepad2());
        imuSensor.readDevice();
        zeroHeading = imuSensor.getHeading();

        setupParameterChanger();
        parameterChanger.enable(true);


        servoPositionTuner = new ServoPositionTuner(hardwareMap, telemetry);
    }


    // Initialize components after start to prevent movement in init
    @Override
    public void start() {
        gripper = new Gripper(hardwareMap);
        gearbox = new Gearbox(hardwareMap);
        lift = new Lift(hardwareMap, gearbox);
        outtake = new Outtake(lift, gripper, signals, telemetry, gearbox);
    }

    double zeroHeading;

    @Override
    public void loop() {


        parameterChanger.update();

        servoPositionTuner.servoTunerFsmUpdate();
        extendSmallJoint.update(getGamepad1().cross);

        resetHeadingButton.update(getGamepad1().dpad_up);

        driverReturnButtonIntake.update(getGamepad1().right_trigger > 0.01);
        driverReturnButton2Intake.update(getGamepad1().left_trigger > 0.01); // CE I ASTA?
        driverGatherButton.update(getGamepad1().right_bumper);
        driverEjectButton.update(getGamepad1().left_bumper);
        resetEncodersButton.update(getGamepad2().left_trigger > 0.01);
        gatherButton.update(getGamepad2().square);
        returnButtonIntake.update(getGamepad2().cross);

        command1.update(getGamepad2().circle);
        scoreInLowBasket.update(getGamepad2().dpad_left);
        returnButtonOuttake.update(getGamepad2().dpad_down);
        transferButton.update(getGamepad2().dpad_right);
        basketButton.update(getGamepad2().dpad_up);
        parkLift.update(getGamepad2().left_bumper);
        changeGear.update(getGamepad2().triangle);
        goToHang.update(getGamepad2().right_bumper);
        liftToHang.update(getGamepad2().right_trigger > 0.01);

        intake.update();
        outtake.update();
        signals.update();
        gearbox.update();

        controlMovement(getGamepad1(), gamepad2);
        prevGamepad1.copy(getGamepad1());
        prevGamepad2.copy(getGamepad2());
    }

    private void controlMovement(Gamepad gamepad1, Gamepad gamepad2) {
        resetHeadingButton.onPress(() -> zeroHeading = imuSensor.getHeading());

        imuSensor.readDevice();
        gamepadInput1.readDevice();
        gamepadInput2.readDevice();
        double fineMovementPower = 0.2;
        if (gamepadInput1.getAmplitudeRight() > 0.05 || gamepadInput2.getAmplitudeRight() > 0.05) {
            rotationController.blindRotate(getGamepad1().right_stick_x + (getGamepad2().right_stick_x) * fineMovementPower);
        } else {
            if (rotationController.isBlindRotating()) {
                //      rotationController.setHeading(imuSensor.getHeading());
                rotationController.blindRotate(0);
            }
        }

        // TODO: Reverse turbo button logic
        movement.setMaximumSpeed(1000.0);
        // TODO: Calculate walkAngle and walkSpeed using AngleHelper.addPolarMagnitude and AngleHelper.addPolarAngle
        // TODO: Clamp walkSpeed to [0, 1]
        // TODO: Turbo only for chassis driver maybe?
        double d1p = gamepadInput1.getAmplitudeLeft();
        double d2p = gamepadInput2.getAmplitudeLeft() * fineMovementPower;
        double d1a = gamepadInput1.getAngleLeft() * Math.PI / 180;
        double d2a = gamepadInput2.getAngleLeft() * Math.PI / 180;
        double walkSpeed = (AngleHelper.addPolarMagnitude(d1p, d2p, d1a, d2a));
        double walkAngle = AngleHelper.norm(zeroHeading + AngleHelper.addPolarAngle(d1p, d2p, d1a, d2a) * 180 / Math.PI);
        rotationController.setDirection(walkAngle, walkSpeed);
        rotationController.update();

        telemetry.update();
    }

    private void setupRotationButtons() {
        rotateFwdButton.onPress(() -> rotationController.setHeading(zeroHeading));
        rotateBackButton.onPress(() -> rotationController.setHeading(zeroHeading + 180));
        rotateLeftButton.onPress(() -> rotationController.setHeading(zeroHeading + 90));
        rotateRightButton.onPress(() -> rotationController.setHeading(zeroHeading - 90));
        movement.getFollowerEdge().onPress(() -> {
            rotationController.setHeading(imuSensor.getHeading());
        });

        moveToHighChamber.onPress(() -> {
            movement.moveToHighChamber();
        });

    }

    private void setupIntakeButtons() {
        gatherButton.onPress(() -> intake.command(Intake.Command.GATHER));
        ejectButton.onPress(() -> intake.command(Intake.Command.EJECT));
        returnButtonIntake.onPress(() -> intake.command(Intake.Command.RETURN));

        driverGatherButton.onPress(() -> intake.command(Intake.Command.GATHER));
        driverEjectButton.onPress(() -> intake.command(Intake.Command.EJECT));
        driverReturnButtonIntake.onPress(() -> intake.command(Intake.Command.RETURN));
        driverReturnButton2Intake.onPress(() -> outtake.command(Outtake.Command.RETURN));

        extendSmallJoint.onPress(() -> intake.command(Intake.Command.GATHER_SMALL));
    }

    private void setupLiftButtons() {
        transferButton.onPress(() -> {
            intake.command(Intake.Command.TRANSFER);
        });
        basketButton.onPress(() -> outtake.command(Outtake.Command.BASKET));
        returnButtonOuttake.onPress(() -> outtake.command(Outtake.Command.RETURN));
        command1.onPress(() -> outtake.command(Outtake.Command.SPECIMEN));

        scoreInLowBasket.onPress(() -> outtake.command(Outtake.Command.LOW));

        resetEncodersButton.onHold(() -> {
            lift.manualOverride(-0.5, true);
        });
        resetEncodersButton.onRelease(() -> {
            lift.setManualOverrideActive(false);
            lift.resetEncoder();
            lift.goTakeSpecimen();
        });

        liftToHang.onHold(() -> {
            lift.manualOverride(-1);
        });

        liftToHang.onRelease(() -> {
            lift.setManualOverrideActive(false);
            lift.resetEncoder();
            lift.goHome();
        });

        goToHang.onPress(() -> {
            gearbox.command(Gearbox.Command.LIFT_UP);
        });


        parkLift.onPress(() -> outtake.command(Outtake.Command.PARK));

        changeGear.onPress(() -> gearbox.command(Gearbox.Command.CHANGE_GEAR));


    }

    //TODO: Register created params from Intake here
    private void setupParameterChanger() {
        PID rotPid = rotationController.pid();
        parameterChanger.register("Heading KP", rotPid::kp, rotPid::kp);
        parameterChanger.register("Heading KD", rotPid::kd, rotPid::kd);
        parameterChanger.register("Heading KI", rotPid::ki, rotPid::ki);


        PIDFLinear liftPid = lift.controller();
        parameterChanger.register("Lift KP", liftPid::kp, liftPid::kp);
        parameterChanger.register("Lift KD", liftPid::kd, liftPid::kd);
        parameterChanger.register("Lift KI", liftPid::ki, liftPid::ki);
        parameterChanger.register("Lift KA", liftPid::ka, liftPid::ka);
        parameterChanger.register("Lift KB", liftPid::kb, liftPid::kb);
        parameterChanger.register("Lift K0", liftPid::k0, liftPid::k0);
        parameterChanger.register("Lift height Home", lift::posHome, lift::posHome);
        parameterChanger.register("Lift height Parking", lift::posParking, lift::posParking);
        parameterChanger.register("Lift height HangSpecimen", lift::posHangSpecimen, lift::posHangSpecimen);
        parameterChanger.register("Lift height HangedSpecimen", lift::posHangedSpecimen, lift::posHangedSpecimen);
        parameterChanger.register("Lift height HighBasket", lift::posHighBasket, lift::posHighBasket);
        parameterChanger.register("Lift height LowBasket", lift::posLowBasket, lift::posLowBasket);
        parameterChanger.register("Lift tolerance ForcePower", lift::toleranceForcePower, lift::toleranceForcePower);
        parameterChanger.register("Lift tolerance ReachedDestination", lift::toleranceReachedDestination, lift::toleranceReachedDestination);
        parameterChanger.register("Lift power Force Raise", lift::forceRaisePower, lift::forceRaisePower);
        parameterChanger.register("Lift power Force Lower", lift::forceLowerPower, lift::forceLowerPower);

        parameterChanger.register("Gripper Open", gripper::posGripperOpen, gripper::posGripperOpen);
        parameterChanger.register("Gripper Close", gripper::posGripperClose, gripper::posGripperClose);
        parameterChanger.register("Arm Joint Park", gripper::posJointPark, gripper::posJointPark);
        parameterChanger.register("Arm Wrist Park", gripper::posWristPark, gripper::posWristPark);
        parameterChanger.register("Arm Joint Take", gripper::posJointTake, gripper::posJointTake);
        parameterChanger.register("Arm Wrist Take", gripper::posWristTake, gripper::posWristTake);
        parameterChanger.register("Arm Joint Basket", gripper::posJointBasket, gripper::posJointBasket);
        parameterChanger.register("Arm Wrist Basket", gripper::posWristBasket, gripper::posWristBasket);
        parameterChanger.register("Arm Joint Hang", gripper::posJointHang, gripper::posJointHang);
        parameterChanger.register("Arm Wrist Hang", gripper::posWristHang, gripper::posWristHang);
        parameterChanger.register("Arm Gripper Closing Time", outtake::getTimeToCloseGripper, outtake::setTimeToCloseGripper);
        parameterChanger.register("Arm Gripper Rotate Time", outtake::getTimeToRotateGripper, outtake::setTimeToRotateGripper);

        parameterChanger.register("Intake Joint Up", intake::posJointUp, intake::posJointUp);
        parameterChanger.register("Intake Wrist Up", intake::posWristUp, intake::posWristUp);
        parameterChanger.register("Intake Joint Down", intake::posJointDown, intake::posJointDown);
        parameterChanger.register("Intake Wrist Down", intake::posWristDown, intake::posWristDown);
        parameterChanger.register("Intake Power", intake::intakePower, intake::intakePower);
        parameterChanger.register("Intake Transfer Time", intake::transferringTime, intake::transferringTime);


        parameterChanger.register("Intake Extend Scissor Right", intake::extendedAngleRight, intake::extendedAngleRight);
        parameterChanger.register("Intake Extend Scissor Left", intake::extendedAngleLeft, intake::extendedAngleLeft);
        parameterChanger.register("Intake Retract Scissor Right", intake::retractedAngleRight, intake::retractedAngleRight);
        parameterChanger.register("Intake Retract Scissor Left", intake::retractedAngleLeft, intake::retractedAngleLeft);
        parameterChanger.register("Intake Extend Time", intake::extendingScissorTime, intake::extendingScissorTime);
        parameterChanger.register("Intake Rotate Joint Time", intake::extendingJointTime, intake::extendingJointTime);

        parameterChanger.register("Gearbox Speed Pos", gearbox::speedPos, gearbox::speedPos);
        parameterChanger.register("Gearbox Power Pos", gearbox::powerPos, gearbox::powerPos);
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
