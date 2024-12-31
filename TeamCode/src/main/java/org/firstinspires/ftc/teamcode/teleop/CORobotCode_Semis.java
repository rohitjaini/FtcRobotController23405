package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriverRR;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * TeleOp mode for controlling the robot. Integrates driving, arm, slide, and intake systems.
 * Uses PIDF controllers to manage motor movements for precise positioning.
 */
@Config
@TeleOp
public class CORobotCode_Semis extends LinearOpMode {

    public static double MAX_ARM_POWER = 0.7;
    public static int ARM_INITIAL_ANGLE = 90; //deg
    public static double MAX_SLIDE_POWER_UP = 1;
    public static double MAX_SLIDE_POWER_DOWN = 0.8;
    public static int SLIDE_DEPOSIT_POSITION = 2600;
    public static int SLIDE_SPEC_BAR_POSITION = 1400;
    public static int SLIDE_SPEC_CLIP_POSITION = 950;
    public static int SLIDE_SPEC_GRAB_POSITION = 0;
    public static int ARM_GRAB_POSITION = 900;
    public static int ARM_GRAB_LOWER_POSITION = 930;
    public static int ARM_HOLD_POSITION = 160;
    public static int ARM_TRANSFER_POSITION = 560;
    public static int ARM_SUB_HOLD = 850;
    public static double WRIST_TRANSFER_POSITION = 0.20;
    public static double WRIST_GRAB_POSITION = 0.62;
    public static double ARM_CLAW_FULL_OPEN = 0.45;
    public static double ARM_CLAW_FULL_CLOSE = 0.65;
    public static double ARM_CLAW_TRANSFER_OPEN = 0.5;
    public static double SPEC_CLAW_OPEN = 0.9;
    public static double SPEC_CLAW_CLOSE = 0.3;
    public static double BUCKET_DEPOSIT_POSITION = 0.78;
    public static double BUCKET_TRANSFER_POSITION = 0.1;
    private PIDFMotorController armController;
    private PIDFMotorControllerSlides slideController;

    // Define hardware components
    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    private Servo rightWristServo, specServo, bucketServo;
    private Servo clawIntake;
    private GoBildaPinpointDriverRR pinpointdriver;
    private IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize hardware
        initializeHardware();

        // Reset IMU orientation
        IMU.Parameters imuParameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                )
        );
        imu.initialize(imuParameters);
        pinpointdriver.initialize();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        // Wait for start
        telemetry.addLine("Initialized. Ready to start.");
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            handleDriving();
            slideControl();
            intakeControl();
            bucketControl();
            specClawControl();
            resetSlidePID();

            runPIDIterations();
            telemetry.update();
        }
    }

    /**
     * Initializes the hardware components and PIDF controllers.
     */
    private void initializeHardware() {
        // Initialize motors and servos
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        DcMotorEx intakeArmMotor = hardwareMap.get(DcMotorEx.class, "intakeArmMotor");
        DcMotorEx rightSlideMotor = hardwareMap.get(DcMotorEx.class, "rightSlideMotor");

        rightWristServo = hardwareMap.get(Servo.class, "rightWristServo");
        specServo = hardwareMap.get(Servo.class, "specServo");
        bucketServo = hardwareMap.get(Servo.class, "bucketServo");
        clawIntake = hardwareMap.get(Servo.class, "clawIntake");

        double armTicksInDegrees = 537.7 / 360.0;
        double slideTicksInDegrees = 384.5 / 360.0;

        // Initialize PIDF controllers for the arm and slide
        armController = new PIDFMotorController(intakeArmMotor, 0.008, 0.32, 0.0005, 0.4, armTicksInDegrees, MAX_ARM_POWER, ARM_INITIAL_ANGLE);
        slideController = new PIDFMotorControllerSlides(rightSlideMotor, 0.01, 0.6, 0.001, 0, slideTicksInDegrees, MAX_SLIDE_POWER_UP);

        // Set directions for drivetrain motors
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize the IMU
        imu = hardwareMap.get(IMU.class, "imu");
        pinpointdriver = hardwareMap.get(GoBildaPinpointDriverRR.class,"pinpoint");
    }

    /**
     * Manages the drivetrain controls using mecanum drive with field-oriented control.
     */
    private void handleDriving() {
        double y = -gamepad1.left_stick_y; // Forward/backward
        double x = gamepad1.left_stick_x;  // Left/right
        double rx = gamepad1.right_stick_x; // Rotation

        if (gamepad1.back) {
            imu.resetYaw();
            pinpointdriver.resetYaw();
            gamepad1.rumble(100);
        }

        double botHeading = pinpointdriver.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Field-oriented adjustments
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        // Set motor powers
        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
    }

    private void slideControl(){
        double inputPosition = gamepad2.left_trigger + gamepad2.right_trigger;
        int slidePosition = (int) (inputPosition / 2 * SLIDE_DEPOSIT_POSITION);
        if (slidePosition != 0) {
            moveSlidesToPosition(slidePosition);
        } else {
            if (gamepad2.dpad_up) {
                moveSlidesToPosition(SLIDE_DEPOSIT_POSITION);
            } else if (gamepad2.dpad_down) {
                moveSlidesToPosition(SLIDE_SPEC_GRAB_POSITION);
            } else if (gamepad2.dpad_left) {
                moveSlidesToPosition(SLIDE_SPEC_BAR_POSITION);
            } else if (gamepad2.dpad_right) {
                moveSlidesToPosition(SLIDE_SPEC_CLIP_POSITION);
            }
        }
    }

    private void moveSlidesToPosition(int position){
        if(slideController.getCurrentPosition() < position){
            slideController.setMaxSpeed(MAX_SLIDE_POWER_UP);
        } else {
            slideController.setMaxSpeed(MAX_SLIDE_POWER_DOWN);
        }

        slideController.setTargetPosition(position);
    }

    private void intakeControl(){
        if (gamepad1.a){
            armController.setTargetPosition(ARM_GRAB_POSITION);
            rightWristServo.setPosition(WRIST_GRAB_POSITION);
            clawIntake.setPosition(ARM_CLAW_FULL_OPEN);
        } else if (gamepad1.b){
            armController.setTargetPosition(ARM_HOLD_POSITION);
            rightWristServo.setPosition(WRIST_GRAB_POSITION);
        } else if (gamepad1.y){
            armController.setTargetPosition(ARM_TRANSFER_POSITION);
            rightWristServo.setPosition(WRIST_TRANSFER_POSITION);
        } else if (gamepad1.x){
            armController.setTargetPosition(ARM_SUB_HOLD);
            rightWristServo.setPosition(WRIST_GRAB_POSITION);
        } else if (gamepad1.right_bumper){
            clawIntake.setPosition(ARM_CLAW_FULL_CLOSE);
        } else if (gamepad1.left_bumper){
            clawIntake.setPosition(ARM_CLAW_TRANSFER_OPEN);
        }
        else if (gamepad1.dpad_down){
            armController.setTargetPosition(ARM_GRAB_LOWER_POSITION);
            rightWristServo.setPosition(WRIST_GRAB_POSITION);
        }
    }

    /**
     * Controls the bucket position based on gamepad2 inputs.
     */
    private void bucketControl() {
        if (gamepad2.a) {
            bucketServo.setPosition(BUCKET_TRANSFER_POSITION); // Default position
        } else if (gamepad2.y) {
            bucketServo.setPosition(BUCKET_DEPOSIT_POSITION); // Release position
        }
    }
    private void specClawControl(){
        if (gamepad2.b) {
            specServo.setPosition(SPEC_CLAW_CLOSE); //close spec claw
        }
        if (gamepad2.x) {
            specServo.setPosition(SPEC_CLAW_OPEN); //open spec claw
        }
    }
    private void resetSlidePID(){
        if (gamepad2.back) {
            slideController.resetMotorEncoder();
            gamepad2.rumble(100);
        }
    }
    private void runPIDIterations() {
        PIDFMotorController.MotorData armMotorData = armController.runIteration();
        PIDFMotorControllerSlides.MotorData slideMotorData = slideController.runIteration();
        telemetry.addData("Arm Position", armMotorData.CurrentPosition);
        telemetry.addData("Arm Target", armMotorData.TargetPosition);
        telemetry.addData("Arm Power", armMotorData.SetPower);
        telemetry.addData("Slides Position", slideMotorData.CurrentPosition);
        telemetry.addData("Slides Target", slideMotorData.TargetPosition);
    }
}