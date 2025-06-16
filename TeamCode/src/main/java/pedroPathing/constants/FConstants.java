package pedroPathing.constants;

import com.pedropathing.localization.Localizers;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.util.CustomFilteredPIDFCoefficients;
import com.pedropathing.util.CustomPIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class FConstants {
    static {
        FollowerConstants.localizers = Localizers.THREE_WHEEL_IMU;
        FollowerConstants.leftFrontMotorName = "Motor Front Left";
        FollowerConstants.leftRearMotorName = "Motor Rear Left";
        FollowerConstants.rightFrontMotorName = "Motor Front Right";
        FollowerConstants.rightRearMotorName = "Motor Rear Right";


        FollowerConstants.leftFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.leftRearMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
        FollowerConstants.rightRearMotorDirection = DcMotorSimple.Direction.FORWARD;

        FollowerConstants.useBrakeModeInTeleOp = true;

        FollowerConstants.mass = 13;

        FollowerConstants.xMovement = 69;
        FollowerConstants.yMovement = 50.5;

        FollowerConstants.forwardZeroPowerAcceleration = -44.5;
        FollowerConstants.lateralZeroPowerAcceleration = -90;

        FollowerConstants.translationalPIDFCoefficients.setCoefficients(0.12/1.45 ,0.025/1.45,0.007/1.45,0);
        // 0.12, 0.025, 0.007, 0
        FollowerConstants.useSecondaryTranslationalPID = false ;
        FollowerConstants.secondaryTranslationalPIDFCoefficients.setCoefficients(0.1,0,0.01,0); // Not being used, @see useSecondaryTranslationalPID

        FollowerConstants.headingPIDFCoefficients.setCoefficients(1,0,0.1,0);
        // 1, 0, 0.1, 0
        FollowerConstants.useSecondaryHeadingPID = false;
        FollowerConstants.secondaryHeadingPIDFCoefficients.setCoefficients(0.5,0.005,0.1 ,0); // Not being used, @see useSecondaryHeadingPID

        FollowerConstants.drivePIDFCoefficients.setCoefficients(0.008,0,0.00002,0.6,0.1);
        //0.008, 0, 0.00002, 0.6, 0.1
        FollowerConstants.useSecondaryDrivePID = false;
        FollowerConstants.secondaryDrivePIDFCoefficients.setCoefficients(0,0,0,0.6,0); // Not being used, @see useSecondaryDrivePID

        FollowerConstants.zeroPowerAccelerationMultiplier = 3;
        FollowerConstants.centripetalScaling = 0.0005;

        FollowerConstants.pathEndTimeoutConstraint = 350;
        FollowerConstants.pathEndTValueConstraint = 0.995;
        FollowerConstants.pathEndVelocityConstraint = 0.1;
        FollowerConstants.pathEndTranslationalConstraint = 0.1;
        FollowerConstants.pathEndHeadingConstraint = 0.007;
    }
}
