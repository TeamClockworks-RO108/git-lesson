package pedroPathing.constants;

import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class LConstants {
    static {
        ThreeWheelIMUConstants.forwardTicksToInches =  .002;
        ThreeWheelIMUConstants.strafeTicksToInches = .002;
        ThreeWheelIMUConstants.turnTicksToInches = .002;
        ThreeWheelIMUConstants.leftY = 7.275;
        ThreeWheelIMUConstants.rightY = -7.632;
        ThreeWheelIMUConstants.strafeX = 10.537;
        ThreeWheelIMUConstants.leftEncoder_HardwareMapName = "Motor Front Left";
        ThreeWheelIMUConstants.rightEncoder_HardwareMapName = "Motor Front Right";
        ThreeWheelIMUConstants.strafeEncoder_HardwareMapName = "Motor Rear Right";
        ThreeWheelIMUConstants.leftEncoderDirection = Encoder.REVERSE;
        ThreeWheelIMUConstants.rightEncoderDirection = Encoder.REVERSE;
        ThreeWheelIMUConstants.strafeEncoderDirection = Encoder.FORWARD;
        ThreeWheelIMUConstants.IMU_HardwareMapName = "imu";
        ThreeWheelIMUConstants.IMU_Orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        );
    }
}




