package org.firstinspires.ftc.teamcode.autonomous;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.teleop.MecanumDrive;
import org.firstinspires.ftc.teamcode.teleop.PIDFMotorController;

@Config
@Autonomous
public class IL_SpecAuto extends LinearOpMode {
    public static boolean USER_INPUT_FLAG = false;
    public static int SLIDES_ABOVE_BAR = 1900;
    public static int SLIDES_BELOW_BAR = 1270;
    public static int SLIDES_SPEC_PICKUP = 0;
    public static double SPEC_CLAW_CLOSE = 0.3;
    public static double SPEC_CLAW_OPEN = 0.9;
    public static int INTAKE_ARM_UP = 10;
    public static int INTAKE_ARM_DOWN = 250;
    public static double SLIDE_MAX_SPEED = 0.9;
    public static double ARM_MAX_SPEED = 0.5;
    public static double WRIST_SERVO_DOWN = 0.05;
    public static int ARM_INITIAL_ANGLE = 50; //deg

    public static class SpecClaw {
        private final Servo specServo;

        public SpecClaw(HardwareMap hardwareMap) {
            specServo = hardwareMap.get(Servo.class, "specServo");
        }

        public class CloseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                specServo.setPosition(SPEC_CLAW_CLOSE);
                return false;
            }
        }

        public Action closeClaw() {
            return new CloseClaw();
        }

        public class OpenClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                specServo.setPosition(SPEC_CLAW_OPEN);
                return false;
            }
        }

        public Action openClaw() {
            return new OpenClaw();
        }
    }
    public static class WristServo {
        private final Servo rightWristServo;

        public WristServo(HardwareMap hardwareMap) {
            rightWristServo = hardwareMap.get(Servo.class, "rightWristServo");
        }

        public class WristServoIn implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                rightWristServo.setPosition(WRIST_SERVO_DOWN);
                return false;
            }
        }
        public Action wristServoIn() {
            return new WristServoIn();
        }
    }
    public static class Slides {
        private final PIDFMotorController slideController;

        public Slides(HardwareMap hardwareMap) {
            DcMotorEx rightSlideMotor = hardwareMap.get(DcMotorEx.class, "rightSlideMotor");
            rightSlideMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            double slideTicksInDegrees = 537.7 / 360.0;
            slideController = new PIDFMotorController(rightSlideMotor, 0.01, 0.25, 0.001, 0, slideTicksInDegrees, SLIDE_MAX_SPEED);
            slideController.resetMotorEncoder();
        }

        public class SlidesPIDIteration implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                PIDFMotorController.MotorData data = slideController.runIteration();
                packet.put("slides power", data.SetPower);
                packet.put("slides position", data.CurrentPosition);
                packet.put("slides target", data.TargetPosition);
                return true;
            }
        }

        public Action slidesPIDIteration() {
            return new SlidesPIDIteration();
        }

        public class MoveSlidesAction implements Action {
            private final int targetPosition;

            public MoveSlidesAction(int targetPosition) {
                this.targetPosition = targetPosition;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                slideController.setTargetPosition(targetPosition);
                packet.put("slides target position", targetPosition);
                return false;
            }
        }

        public Action slidesToAboveBar() {
            return new MoveSlidesAction(SLIDES_ABOVE_BAR);
        }

        public Action slidesToBelowBar() {
            return new MoveSlidesAction(SLIDES_BELOW_BAR);
        }

        public Action slidesToSpecPickup() {
            return new MoveSlidesAction(SLIDES_SPEC_PICKUP);
        }
    }

    public static class IntakeArm {
        private final PIDFMotorController armController;

        public IntakeArm(HardwareMap hardwareMap) {
            DcMotorEx intakeArmMotor = hardwareMap.get(DcMotorEx.class, "intakeArmMotor");
            intakeArmMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            double armTicksInDegrees = 1425.1 / 360.0;
            armController = new PIDFMotorController(intakeArmMotor, 0.008, 0.13, 0.001, 0.4, armTicksInDegrees, ARM_MAX_SPEED, ARM_INITIAL_ANGLE);
            armController.resetMotorEncoder();
        }

        public class ArmPIDIteration implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                PIDFMotorController.MotorData data = armController.runIteration();
                packet.put("arm power", data.SetPower);
                packet.put("arm position", data.CurrentPosition);
                packet.put("arm target", data.TargetPosition);
                return true;
            }
        }

        public Action armPIDIteration() {
            return new ArmPIDIteration();
        }

        public class moveArmAction implements Action {
            private final int targetPosition;

            public moveArmAction(int targetPosition) {
                this.targetPosition = targetPosition;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                armController.setTargetPosition(targetPosition);
                packet.put("intake arm target position", targetPosition);
                return false;
            }
        }

        public Action intakeArmUp() {
            return new moveArmAction(INTAKE_ARM_UP);
        }

        public Action intakeArmDown() {
            return new moveArmAction(INTAKE_ARM_DOWN);
        }
    }

    public static class WaitForUser implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if(USER_INPUT_FLAG){
                USER_INPUT_FLAG = false;
                packet.put("Waiting For User", false);
                return false;
            }
            packet.put("Waiting For User", true);
            USER_INPUT_FLAG = false;
            return false;
        }
    }

    public Action waitForUser() {
        return new WaitForUser();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(13, -61.5, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        SpecClaw specClaw = new SpecClaw(hardwareMap);
        Slides slides = new Slides(hardwareMap);
        IntakeArm intakeArm = new IntakeArm(hardwareMap);
        WristServo wristServo = new WristServo(hardwareMap);


        TrajectoryActionBuilder moveAwayFromBarrier = drive.actionBuilder(beginPose)
                .strafeTo(new Vector2d(13, -50))
                .waitSeconds(0.001);
        TrajectoryActionBuilder moveIntoSpec1Position = moveAwayFromBarrier.fresh()
                .waitSeconds(1.5)
                .strafeTo(new Vector2d(0, -25))
                .waitSeconds(0.001);
        TrajectoryActionBuilder driveBack = moveIntoSpec1Position.fresh()
                .strafeTo(new Vector2d(0, -37))
                .waitSeconds(0.001);
        TrajectoryActionBuilder push2SamplesGrabSpec = driveBack.fresh()
                .strafeTo(new Vector2d(43, -35)) // go to the right
                .strafeTo(new Vector2d(43, -10))
                .splineTo(new Vector2d(53, -10), Math.toRadians(270))
                .strafeTo(new Vector2d(53, -57))
                .strafeTo(new Vector2d(53, -3))
                .waitSeconds(0.5)
                .strafeTo(new Vector2d(60,-3))
                .strafeTo(new Vector2d(60,-58))
                .strafeTo(new Vector2d(58,-63.5))
                .waitSeconds(0.001);
        TrajectoryActionBuilder goToSubSecondSpec = push2SamplesGrabSpec.fresh()
                .waitSeconds(0.5)
                .strafeTo(new Vector2d(53, -45)) //strafe up field
                .strafeToLinearHeading(new Vector2d(4,-45), Math.toRadians(270)) //change heading
                .strafeTo(new Vector2d(4, -25))
                .waitSeconds(0.001);
        TrajectoryActionBuilder goToZoneThirdSpec = goToSubSecondSpec.fresh()
                .waitSeconds(0.001)
                .strafeTo(new Vector2d(4,-35))
                .strafeToLinearHeading(new Vector2d(40,-53), Math.toRadians(90))
                .strafeTo(new Vector2d(58,-63.5))
                .waitSeconds(0.001);
        TrajectoryActionBuilder goToSubThirdSpec = goToZoneThirdSpec.fresh()
                .waitSeconds(0.5)
                .strafeTo(new Vector2d(58, -45)) //strafe up field
                .strafeToLinearHeading(new Vector2d(-4,-45), Math.toRadians(270)) //change heading
                .strafeTo(new Vector2d(-4, -25))
                .waitSeconds(0.001);
        TrajectoryActionBuilder goBackAndPark = goToSubThirdSpec.fresh()
                .waitSeconds(1)
                .strafeTo(new Vector2d(-4, -45))
                .strafeToLinearHeading(new Vector2d(52,-59), Math.toRadians(90))
                .waitSeconds(0.001);

        Action moveAwayFromBarrierAction = moveAwayFromBarrier.build();
        Action moveIntoSpec1PositionAction = moveIntoSpec1Position.build();
        Action driveBackAction = driveBack.build();
        Action push2SamplesGrabSpecAction = push2SamplesGrabSpec.build();
        Action goToSubSecondSpecAction = goToSubSecondSpec.build();
        Action goToZoneThirdSpecAction = goToZoneThirdSpec.build();
        Action goToSubThirdSpecAction = goToSubThirdSpec.build();
        Action goBackAndParkAction = goBackAndPark.build();

        Action autoSequence = new SequentialAction(
                waitForUser(),
                moveAwayFromBarrierAction, // Move away from the barrier
                waitForUser(),
                waitForUser(),
                waitForUser(),
                moveIntoSpec1PositionAction, // Move into position to place the first spec
                waitForUser(),
                waitForUser(),
                waitForUser(),
                driveBackAction, //drive back to put slides fully down
                waitForUser(),
                waitForUser(),
                push2SamplesGrabSpecAction, //pushes sample into player person zone, then grabs spec
                waitForUser(),
                waitForUser(),
                waitForUser(),
                goToSubSecondSpecAction,
                goToZoneThirdSpecAction,
                goToSubThirdSpecAction,
                waitForUser(),
                waitForUser(),
                waitForUser(),
                goBackAndParkAction, //park in player person zone
                waitForUser(),
                waitForUser(),
                new SleepAction(1), //wait for slides to come down
                new SleepAction(0.5) //wait for servo to go in
        );
        Action pidControlLoops = new ParallelAction(
                slides.slidesPIDIteration(), //cycling through slides to hold PID
                intakeArm.armPIDIteration() //cycling through intakeArm to hold PID
        );

        waitForStart();
        Actions.runBlocking(new ParallelAction(autoSequence, pidControlLoops));
    }
}