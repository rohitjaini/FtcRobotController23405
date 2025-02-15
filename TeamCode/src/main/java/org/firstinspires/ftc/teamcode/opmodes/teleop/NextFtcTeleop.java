package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.utility.InstantCommand;
import com.rowanmcalpin.nextftc.ftc.NextFTCOpMode;
import com.rowanmcalpin.nextftc.ftc.driving.MecanumDriverControlled;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.DepositBucket;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.IntakeClaw;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Slides;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.SpecClaw;

@TeleOp
public class NextFtcTeleop extends NextFTCOpMode {

    public NextFtcTeleop() {
        super(DepositBucket.INSTANCE, IntakeClaw.INSTANCE, Slides.INSTANCE, SpecClaw.INSTANCE);
    }

    public String frontLeftName = "frontLeftMotor";
    public String backLeftName = "backLeftMotor";
    public String frontRightName = "frontRightMotor";
    public String backRightName = "backRightMotor";

    public MotorEx frontLeftMotor;
    public MotorEx frontRightMotor;
    public MotorEx backLeftMotor;
    public MotorEx backRightMotor;

    public MotorEx[] motors;

    public IMU imu;

    public Command driverControlled;

    @Override
    public void onInit() {
        frontLeftMotor = new MotorEx(frontLeftName);
        backLeftMotor = new MotorEx(backLeftName);
        backRightMotor = new MotorEx(backRightName);
        frontRightMotor = new MotorEx(frontRightName);

// Change your motor directions to suit your robot.
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        motors = new MotorEx[] { frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor };

// Only include this if you are using field centric. If using, change your control hub orientation to suit your robot.
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD)));
        imu.resetYaw();
    }

    @Override
    public void onStartButtonPressed() {

        driverControlled = new MecanumDriverControlled(motors, gamepadManager.getGamepad1(), false, imu);
        driverControlled.invoke();

        gamepadManager.getGamepad1().getRightBumper().setPressedCommand(IntakeClaw.INSTANCE::full_close); //close intake claw to grab

        gamepadManager.getGamepad1().getLeftBumper().setPressedCommand(IntakeClaw.INSTANCE::transfer_intake_open); //open intake claw to release

        gamepadManager.getGamepad2().getB().setPressedCommand(SpecClaw.INSTANCE::open); //open spec claw

        gamepadManager.getGamepad2().getX().setPressedCommand(SpecClaw.INSTANCE::close); //close spec claw

        gamepadManager.getGamepad1().getBack().setPressedCommand(() -> {
            return new InstantCommand(() -> {
                imu.resetYaw();
                gamepad1.rumble(100);
                return null;
            });});
    }
   }