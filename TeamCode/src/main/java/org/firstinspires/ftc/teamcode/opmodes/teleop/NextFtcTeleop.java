package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static org.firstinspires.ftc.teamcode.tuning.Globals.ARM_GRAB_POSITION;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
import com.rowanmcalpin.nextftc.core.command.utility.InstantCommand;
import com.rowanmcalpin.nextftc.ftc.NextFTCOpMode;
import com.rowanmcalpin.nextftc.ftc.driving.MecanumDriverControlled;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx;
import org.firstinspires.ftc.teamcode.tuning.Globals.*;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.DepositBucket;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.IntakeArm;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.IntakeArmWrist;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.IntakeClaw;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Slides;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.SpecClaw;

import kotlin.jvm.functions.Function0;

@TeleOp
public class NextFtcTeleop extends NextFTCOpMode {

    public NextFtcTeleop() {
        super(DepositBucket.INSTANCE, IntakeClaw.INSTANCE, Slides.INSTANCE, SpecClaw.INSTANCE, IntakeArm.INSTANCE, IntakeArmWrist.INSTANCE);
    }

    public String backLeftName = "backLeftMotor";
    public String frontRightName = "frontRightMotor";
    public String backRightName = "backRightMotor";
    public String frontLeftName = "frontLeftMotor";

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

        backLeftMotor.getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

        gamepadManager.getGamepad2().getX().setPressedCommand(SpecClaw.INSTANCE::open); //open spec claw

        gamepadManager.getGamepad2().getB().setPressedCommand(SpecClaw.INSTANCE::close); //close spec claw

        gamepadManager.getGamepad2().getY().setPressedCommand(DepositBucket.INSTANCE::ToDeposit); //bring bucket to deposit to deposit sample

        gamepadManager.getGamepad2().getA().setPressedCommand(DepositBucket.INSTANCE::ToTransfer); //bring bucket to transfer to get ready for transfer

        gamepadManager.getGamepad2().getDpadUp().setPressedCommand(Slides.INSTANCE::toDeposit); //bring slides fully up for sample deposit

        gamepadManager.getGamepad2().getDpadDown().setPressedCommand(Slides.INSTANCE::toLowest); //bring slides all the way down

        gamepadManager.getGamepad2().getDpadLeft().setPressedCommand(Slides.INSTANCE::toSpecBar);

        gamepadManager.getGamepad2().getDpadRight().setPressedCommand(Slides.INSTANCE::toSpecClip);

        gamepadManager.getGamepad2().getBack().setPressedCommand(Slides.INSTANCE::resetSlidesEncoder); //zero slides position in case of belt slip



        // intake arm and wrist control
        gamepadManager.getGamepad1().getA().setPressedCommand(
                () -> new SequentialGroup(
                        IntakeArm.INSTANCE.toGrab(),
                        IntakeArmWrist.INSTANCE.grab(),
                        IntakeClaw.INSTANCE.full_open()
                )

        );

        gamepadManager.getGamepad1().getY().setPressedCommand(
                () -> new SequentialGroup(
                        IntakeArm.INSTANCE.toTransfer(),
                        IntakeArmWrist.INSTANCE.transfer(),
                        IntakeClaw.INSTANCE.transfer_intake_open()
                )

        );

        gamepadManager.getGamepad1().getX().setPressedCommand(
                () -> new SequentialGroup(
                        IntakeArm.INSTANCE.toSubHold(),
                        IntakeArmWrist.INSTANCE.grab()
                )

        );

        gamepadManager.getGamepad1().getB().setPressedCommand(
                () -> new SequentialGroup(
                        IntakeArm.INSTANCE.toHold(),
                        IntakeArmWrist.INSTANCE.hold()
                )

        );

        gamepadManager.getGamepad1().getDpadDown().setPressedCommand( () -> {
            return new InstantCommand(() -> {
                ARM_GRAB_POSITION += 15;
            });
                }
        );

        gamepadManager.getGamepad1().getDpadUp().setPressedCommand( () -> {
                    return new InstantCommand(() -> {
                        ARM_GRAB_POSITION -= 15;
                    });
                }
        );


        gamepadManager.getGamepad1().getBack().setPressedCommand(() -> {
            return new InstantCommand(() -> {
                imu.resetYaw();
                gamepad1.rumble(100);
            });});
    }

    @Override
    public void onUpdate() {
        telemetry.addData("Intake arm target position", IntakeArm.INSTANCE.intakeArmController.getTarget());
        telemetry.addData("Intake arm current position", IntakeArm.INSTANCE.intakeArmMotor.getCurrentPosition());
        telemetry.addData("Slides target position", Slides.INSTANCE.slidesController.getTarget());
        telemetry.addData("Slides current position", Slides.INSTANCE.rightSlideMotor.getCurrentPosition());
        telemetry.addData("IMU yaw angle", Math.toDegrees(imu.getRobotYawPitchRollAngles().getYaw()));
        telemetry.update();
    }
}