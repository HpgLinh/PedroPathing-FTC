package org.firstinspires.ftc.teamcode.tests;

import static java.lang.Math.atan2;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

import java.util.function.Supplier;

@Configurable
@TeleOp(name="Shooter")
public class shootertest extends OpMode {
    public DcMotorEx shooter = null, shootstraff = null;
    public Turret turret;
    public DcMotor intakeDrive = null;
    public Servo srvLoad = null, srvOval = null, srvIntake = null, srvTurret1 = null, srvTurret2 = null;
    public Follower follower;
    public static com.pedropathing.control.PIDFController headingPid;
    public static double shooterTarget;
    public boolean runIntake, goToHeading;
    public Supplier<PathChain> toGoal;


    double P = 50;
    public IMU imu;
    double I = 0.6;
    double D = 0.001;
    double F = 17;

    int stateIntake = 0;

    int power =1200;



    // Logic type
    boolean toggle_shooting = true;
    boolean prevX=false, prevB=false, prevLB=false, prevRB=false;
    boolean prevOpt=false, prevShare = false;
    boolean prevDpadUp=false, prevDpadDown=false, prevDpadLeft=false, prevDpadRight=false;
    boolean prevRT=false, prevLT=false;
    boolean prevA=false, prevY=false;
    boolean intakeDeployed = false; // false -> stow
    boolean loadOpen = false;
    IMU.Parameters parameters = new IMU.Parameters(
            new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                    RevHubOrientationOnRobot.UsbFacingDirection.UP
            )
    );

    @Override
    public void init() {
        imu = hardwareMap.get(IMU.class, "imu");

        turret = new Turret(hardwareMap);
        headingPid = new PIDFController(Constants.followerConstants.getCoefficientsHeadingPIDF());
        shooter = hardwareMap.get(DcMotorEx.class, "drvShoot");
        shootstraff = hardwareMap.get(DcMotorEx.class, "drvShootstraff");
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(Constants.redStartPose);
        runIntake = false;
        goToHeading = false;


        intakeDrive = hardwareMap.get(DcMotor.class, "drvintake");
        srvIntake = hardwareMap.get(Servo.class, "srvIntake");
        srvLoad = hardwareMap.get(Servo.class, "srvLoad");
        srvOval = hardwareMap.get(Servo.class, "srvOval0");


        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shootstraff.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shootstraff.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter.setVelocityPIDFCoefficients(P, I, D, F);
        shootstraff.setVelocityPIDFCoefficients(P, I, D, F);

    }
    private double clip01(double v) {
        return Range.clip(v, 0.0, 1.0);
    }
    @Override
    public void start() {
        srvIntake.setPosition(0.25);
        srvLoad.setPosition(0.0);
        srvOval.setPosition(0);
        follower.startTeleopDrive(false);
        imu.initialize(parameters);
        imu.resetYaw();
    }
    public int targetX = 138, targetY = 140;
    @Override
    public void loop() {
        double current_heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        shooter .setVelocity(toggle_shooting ? power : 0);
        shootstraff.setVelocity(toggle_shooting ? power : 0);
//        if (gamepad1.dpadUpWasPressed()) {
//            stateIntake = !stateIntake;
//        }
//        intakeDrive.setPower(stateIntake ? 0.7 : 0);


        follower.update();
        if (!goToHeading) {
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    false
            );
        }
        else {
            headingPid.updatePosition(follower.getHeading());
            double angle_unit = getRedTargetHeading(targetX-follower.getPose().getX(), targetY-follower.getPose().getY()) - current_heading;
            while (angle_unit > Math.PI) angle_unit -= 2 * Math.PI;
            while (angle_unit <= -Math.PI) angle_unit += 2 * Math.PI;
            turret.setPosition(-Math.toDegrees(angle_unit));
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    false
            );
        }
        if (gamepad1.leftBumperWasPressed()) {
            goToHeading = !goToHeading;

            if (!goToHeading) {
                follower.breakFollowing();
                follower.startTeleopDrive(false);
            }
        }
//        if(gamepad1.dpadUpWasPressed()) targetX += 10;
//        if(gamepad1.dpadDownWasPressed()) targetX -= 10;
//        if(gamepad1.dpadRightWasPressed()) targetY += 10;
//        if(gamepad1.dpadLeftWasPressed()) targetY -= 10;

        boolean X =  gamepad1.x;
        if (X && !prevX) {
            stateIntake = (stateIntake != 0) ? 0 : 1;
        }
        prevX = X;

        boolean B = gamepad1.b;
        if (B && !prevB) {
            stateIntake = (stateIntake != 0) ? 0 : -1;
        }
        prevB = B;

        boolean rtPress = (gamepad1.right_trigger > 0.9);
        if (rtPress && !prevRT) power += 50;
        prevRT = rtPress;

        boolean ltPress = (gamepad1.left_trigger > 0.9);
        if (ltPress && !prevLT) power -= 50;
        prevLT = ltPress;

//        boolean lb = gamepad1.left_bumper;
//        if(lb && !prevLB) srvIntake.setPosition(0);
//        if(!lb && prevLB) srvIntake.setPosition(0.25);
//        prevLB = lb;

        boolean rb = gamepad1.right_bumper;
        if (rb && !prevRB) {          // vừa nhấn
            srvLoad.setPosition(0.35);
            loadOpen = true;
        }
        if (!rb && prevRB) {          // vừa thả
            srvLoad.setPosition(0.0);
            loadOpen = false;
        }
        prevRB = rb;

        if (stateIntake == 0) intakeDrive.setPower(0);
        else if (stateIntake == -1) intakeDrive.setPower(-1);
        else if (stateIntake == 1) intakeDrive.setPower(loadOpen ? 0.6 : 1);

        if(stateIntake != 0) toggle_shooting = true;
        else toggle_shooting = false;

        if(gamepad1.dpadRightWasPressed()) {srvTurret1.setPosition(srvTurret1.getPosition() + 0.02); srvTurret2.setPosition(srvTurret2.getPosition() + 0.02);}
        if(gamepad1.dpadLeftWasPressed()) {srvTurret1.setPosition(srvTurret1.getPosition() - 0.02); srvTurret2.setPosition(srvTurret2.getPosition() - 0.02);}

        boolean dUp = gamepad1.dpad_up;
        if (dUp && !prevDpadUp) srvOval.setPosition(clip01(Math.min(0.4, srvOval.getPosition() + 0.04)));
        prevDpadUp = dUp;

        boolean dDown = gamepad1.dpad_down;
        if (dDown && !prevDpadDown) srvOval.setPosition(clip01(Math.max(0.0, srvOval.getPosition() - 0.04)));
        prevDpadDown = dDown;

        //if(gamepad1.options) toggle_shooting ^= true;



        if(gamepad1.a) {
            power = 1200;
            srvOval.setPosition(0.16);
//            srvTurret2.setPosition(-0.251 + 0.5);
//            srvTurret1.setPosition(-0.251 + 0.5);
        }

        if(gamepad1.y) {
            power = 850;
            srvOval.setPosition(0.0);
//            srvTurret2.setPosition(-0.15 + 0.5);
//            srvTurret1.setPosition(-0.15 + 0.5);
        }


        // -----------------------------
        telemetry.addData("Intake_servo", srvIntake.getPosition());
        telemetry.addData("Oval", srvOval.getPosition());
        telemetry.addData("turret", turret.currentDegree);
        telemetry.addData("power", power);
        telemetry.addData("State", stateIntake);
        telemetry.addLine("========== Status =========");
        telemetry.addData("bot X", follower.getPose().getX());
        telemetry.addData("bot Y: ", follower.getPose().getY());
        telemetry.addData("bot Heading: ", Math.toDegrees(current_heading));
        telemetry.addData("arctan2:", Math.toDegrees(atan2(130 - follower.getPose().getY(), 0 - follower.getPose().getX())));
        telemetry.addData("Distance: ",
                getDistance(targetX-follower.getPose().getX(), targetY-follower.getPose().getY()));
        telemetry.addLine("========== Target ==========");
        telemetry.addData("X", targetX);
        telemetry.addData("Y", targetY);
        telemetry.update();
    }

    public static double getRegressionVelocity (double distance) {
        return -0.000724792 * Math.pow(distance, 2) + 1.10181 * distance + 100.38172;
    }
    public static double getDistance(double x, double y) {
        return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
    }
    public static double getRedTargetHeading(double x, double y) {
        return atan2(y,x);
    }
}