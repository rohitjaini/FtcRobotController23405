/*package org.firstinspires.ftc.teamcode.autonomous;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.teleop.PIDFMotorController;

@Config
@Autonomous
public class SampleAuto3Samples extends LinearOpMode {
    public static boolean USER_INPUT_FLAG = false;
    public static int SLIDES_ABOVE_BAR = 1400;
    public static int SLIDES_BELOW_BAR = 950;
    public static int SLIDES_SPEC_PICKUP = 0;
    public static int SLIDES_DEPOSIT = 2600;
    public static double SPEC_CLAW_CLOSE = 0.3;
    public static double SPEC_CLAW_OPEN = 0.9;
    public static int INTAKE_ARM_TRANSFER = 560;
    public static int INTAKE_ARM_HOLD = 300;
    public static int INTAKE_ARM_INTAKE = 910;
    public static double SLIDE_MAX_SPEED = 0.9;
    public static double ARM_MAX_SPEED = 0.5;
    public static double WRIST_SERVO_TRANSFER = 0.20;
    public static double WRIST_SERVO_GRAB_AND_HOLD = 0.62;
    public static double BUCKET_TRANSFER_POSITION = 0.1;
    public static double BUCKET_DEPOSIT_POSITION = 0.78;
    public static double ARM_CLAW_FULL_OPEN = 0.45;
    public static double ARM_CLAW_FULL_CLOSE = 0.65;
    public static int ARM_INITIAL_ANGLE = 90; //deg

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

        public class WristServoTransfer implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                rightWristServo.setPosition(WRIST_SERVO_TRANSFER);
                return false;
            }
        }
        public Action wristServoTransfer() {
            return new WristServoTransfer();
        }

        public class WristServoGrabAndHold implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                rightWristServo.setPosition(WRIST_SERVO_GRAB_AND_HOLD);
                return false;
            }
        }

        public Action wristServoGrabAndHold() {return new WristServoGrabAndHold();}
    }
    public static class BucketServo {
        private final Servo bucketServo;

        public BucketServo(HardwareMap hardwareMap) {
            bucketServo = hardwareMap.get(Servo.class, "bucketServo");
        }

        public class BucketTransfer implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                bucketServo.setPosition(BUCKET_TRANSFER_POSITION);
                return false;
            }
        }
        public Action bucketServoTransferPosition() {return new BucketTransfer();}

        public class BucketDeposit implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                bucketServo.setPosition(BUCKET_DEPOSIT_POSITION);
                return false;
            }
        }
        public Action bucketServoDepositPosition() {return new BucketDeposit();}
    }

    public static class ClawServo {
        private final Servo clawServo;

        public ClawServo(HardwareMap hardwareMap) {
            clawServo = hardwareMap.get(Servo.class, "clawIntake");
        }

        public class ClawServoOpen implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                clawServo.setPosition(ARM_CLAW_FULL_OPEN);
                return false;
            }
        }
        public Action clawServoOpen() {return new ClawServoOpen();}

        public class ClawServoClose implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                clawServo.setPosition(ARM_CLAW_FULL_CLOSE);
                return false;
            }
        }
        public Action clawServoClose() {return new ClawServoClose();}
    }

    public static class Slides {
        private final PIDFMotorController slideController;

        public Slides(HardwareMap hardwareMap) {
            DcMotorEx rightSlideMotor = hardwareMap.get(DcMotorEx.class, "rightSlideMotor");
            rightSlideMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            double slideTicksInDegrees = 384.5 / 360.0;
            slideController = new PIDFMotorController(rightSlideMotor, 0.01, 0.6, 0.001, 0, slideTicksInDegrees, SLIDE_MAX_SPEED);
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

        public Action slidesToDeposit() {return new MoveSlidesAction(SLIDES_DEPOSIT);}
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

        public Action intakeArmIntakePosition() {
            return new moveArmAction(INTAKE_ARM_INTAKE);
        }

        public Action intakeArmHold() {return  new moveArmAction(INTAKE_ARM_HOLD);}

        public Action intakeArmDeposit() {
            return new moveArmAction(INTAKE_ARM_TRANSFER);
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
        PinpointDrive drive = new PinpointDrive(hardwareMap, beginPose);
        SpecClaw specClaw = new SpecClaw(hardwareMap);
        Slides slides = new Slides(hardwareMap);
        IntakeArm intakeArm = new IntakeArm(hardwareMap);
        WristServo wristServo = new WristServo(hardwareMap);

        TrajectoryActionBuilder moveAwayFromBarrier = drive.actionBuilder(beginPose)
                .splineToConstantHeading(new Vector2d(-56,-46), Math.PI)
                .waitSeconds(0.001);
        TrajectoryActionBuilder firstSampleDeposit = moveAwayFromBarrier.fresh()
                .waitSeconds(2)
                .strafeTo(new Vector2d(-56, -58))
                .waitSeconds(0.001);
        TrajectoryActionBuilder intakeSecondSample = firstSampleDeposit.fresh()
                .waitSeconds(2)
                .strafeTo(new Vector2d(-47,-37))
                .waitSeconds(0.001);
        TrajectoryActionBuilder secondSampleDeposit = intakeSecondSample.fresh()
                .waitSeconds(2)
                .splineToConstantHeading(new Vector2d(-56,-58), Math.PI)
                .waitSeconds(0.001);
        TrajectoryActionBuilder intakeThirdSample = secondSampleDeposit.fresh()
                .waitSeconds(2)
                .strafeTo(new Vector2d(-58,-37))
                .waitSeconds(0.001);
        TrajectoryActionBuilder thirdSampleDeposit = intakeThirdSample.fresh()
                .waitSeconds(2)
                .splineToConstantHeading(new Vector2d(-56,-58), Math.PI)
                .waitSeconds(0.001);
        TrajectoryActionBuilder strafeForward = thirdSampleDeposit.fresh()
                .waitSeconds(2)
                .strafeTo(new Vector2d(-41,-14))
                .waitSeconds(0.001);
        TrajectoryActionBuilder touchBar = strafeForward.fresh()
                .waitSeconds(0.001)
                .strafeToLinearHeading((new Vector2d(-20,-8)), Math.toRadians(0))
                .strafeTo(new Vector2d(-20,-8))
                .waitSeconds(0.001);


        Action moveAwayFromBarrierAction = moveAwayFromBarrier.build();
        Action firstSampleDepositAction = firstSampleDeposit.build();
        Action intakeSecondSampleAction = intakeSecondSample.build();
        Action secondSampleDepositAction = secondSampleDeposit.build();
        Action intakeThirdSampleAction = intakeThirdSample.build();
        Action thirdSampleDepositAction = thirdSampleDeposit.build();
        Action strafeForwardAction = strafeForward.build();
        Action touchBarAction = touchBar.build();

        Action autoSequence = new SequentialAction(
                moveAwayFromBarrierAction, // Move away from the barrier
                firstSampleDepositAction,
                intakeSecondSampleAction,
                secondSampleDepositAction,
                intakeThirdSampleAction,
                thirdSampleDepositAction,
                strafeForwardAction,
                touchBarAction
        );
        Action pidControlLoops = new ParallelAction(
                slides.slidesPIDIteration(), //cycling through slides to hold PID
                intakeArm.armPIDIteration() //cycling through intakeArm to hold PID
        );

        waitForStart();
        Actions.runBlocking(new ParallelAction(autoSequence, pidControlLoops));
    }
}*/