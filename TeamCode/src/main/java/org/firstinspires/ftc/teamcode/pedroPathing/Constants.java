package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.TwoWheelConstants;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.pedropathing.control.PIDFCoefficients;

public class Constants {
    public static final Pose blueStartPose = new Pose(64, 8, Math.toRadians(180));
    public static final Pose redStartPose = new Pose(120, 83, Math.toRadians(0));
    public static final Pose farRedShoot = new Pose(86, 16, 2.78);
    public static final Pose farBlueShoot = new Pose(58, 18, -2.7);
    public static final Pose redCloseShoot = new Pose(76,76,Math.toRadians(135));
    public static final Pose blueCloseShoot = new Pose(68,72, Math.toRadians(-135));
    public static final Pose redPark = new Pose(35,30, 0);
    public static final Pose bluePark = new Pose(100,30, 0);
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(10.35)
            .lateralZeroPowerAcceleration(-78.459973019259)
            .forwardZeroPowerAcceleration(-42.43979431)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.07, 0, 0.08, 0.03))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.015, 0.01))
            .headingPIDFCoefficients(new com.pedropathing.control.PIDFCoefficients(0.3, 0, 0.07, 0.03))
            .secondaryHeadingPIDFCoefficients(new com.pedropathing.control.PIDFCoefficients(1, 0, 0.03, 0.01))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.02, 0, 0.0005, 0.6, 0.08))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.01, 0, 0.0005, 0.6, 0.2))
            .useSecondaryTranslationalPIDF(true)
            .useSecondaryHeadingPIDF(true)
            .useSecondaryDrivePIDF(true);



    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1.2, 1);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("drv1")
            .xVelocity(56.18647859202622)
            .yVelocity(48.56820300535601)
            .rightRearMotorName("drv4")
            .leftRearMotorName("drv3")
            .leftFrontMotorName("drv2")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);

    public static TwoWheelConstants localizerConstants = new TwoWheelConstants()
            .forwardEncoder_HardwareMapName("drv2")
            .strafeEncoder_HardwareMapName("drv4")
            .strafeEncoderDirection(Encoder.FORWARD)
            .forwardEncoderDirection(Encoder.FORWARD)
            .strafePodX(-0.614173228)
            .forwardPodY(7.08661417)
            .forwardTicksToInches(0.00196293080)
            .strafeTicksToInches(-0.0020667523)
            .IMU_HardwareMapName("imu")
            .IMU_Orientation(
                    new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                            RevHubOrientationOnRobot.UsbFacingDirection.UP
                    )
            );

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .twoWheelLocalizer(localizerConstants)
                .build();
    }
}
