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

import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.teleop.PIDFMotorController;

@Config
@Autonomous
public class SpecAuto4SpecVeryGood extends LinearOpMode {
    public static boolean USER_INPUT_FLAG = false;
    public static int SLIDES_ABOVE_BAR = 1400;
    public static int SLIDES_BELOW_BAR = 950;
    public static int SLIDES_SPEC_PICKUP = 0;
    public static int SLIDES_SLIGHTLY_ABOVE_WALL = 100;
    public static double SPEC_CLAW_CLOSE = 0.3;
    public static double SPEC_CLAW_OPEN = 0.9;
    public static int INTAKE_ARM_UP = 10;
    public static int INTAKE_ARM_DOWN = 50;
    public static double SLIDE_MAX_SPEED = 0.9;
    public static double ARM_MAX_SPEED = 0.5;
    public static double WRIST_SERVO_DOWN = 0.05;
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

        public Action slidesToSlightlyAboveWall(){return  new MoveSlidesAction(SLIDES_SLIGHTLY_ABOVE_WALL);}
    }

    public static class IntakeArm {
        private final PIDFMotorController armController;

        public IntakeArm(HardwareMap hardwareMap) {
            DcMotorEx intakeArmMotor = hardwareMap.get(DcMotorEx.class, "intakeArmMotor");
            intakeArmMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            double armTicksInDegrees = 1425.1 / 360.0;
            armController = new PIDFMotorController(intakeArmMotor, 0.008, 0.32, 0.0005, 0.4, armTicksInDegrees, ARM_MAX_SPEED, ARM_INITIAL_ANGLE);
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
                return true;
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
        Pose2d beginPose = new Pose2d(15, -61.5, Math.toRadians(270));
        PinpointDrive drive = new PinpointDrive(hardwareMap, beginPose);
        SpecClaw specClaw = new SpecClaw(hardwareMap);
        Slides slides = new Slides(hardwareMap);
        IntakeArm intakeArm = new IntakeArm(hardwareMap);
        WristServo wristServo = new WristServo(hardwareMap);


        TrajectoryActionBuilder moveAwayFromBarrier = drive.actionBuilder(beginPose)
                .strafeTo(new Vector2d(15, -50))
                .waitSeconds(0.001);
        TrajectoryActionBuilder moveIntoSpec1Position = moveAwayFromBarrier.fresh()
                .waitSeconds(0.5)
                .strafeTo(new Vector2d(0, -32))
                .waitSeconds(0.5);
        TrajectoryActionBuilder driveBack = moveIntoSpec1Position.fresh()
                .waitSeconds(0.5)
                .strafeTo(new Vector2d(0, -40))
                .waitSeconds(0.001);
        TrajectoryActionBuilder push2SamplesGrabSpec = driveBack.fresh()
                .waitSeconds(0.001)
                .strafeToLinearHeading(new Vector2d(37, -40), Math.toRadians(90)) // go to the right
                .strafeTo(new Vector2d(37, -20))
                .splineToConstantHeading(new Vector2d(45, -20), Math.toRadians(270))
                .strafeTo(new Vector2d(45, -55))
                .strafeTo(new Vector2d(45,-20))
                .splineToConstantHeading(new Vector2d(57,-20), Math.toRadians(270))
                .strafeTo(new Vector2d(57,-58))
                .strafeTo(new Vector2d(57,-40))
                .waitSeconds(0.2)
                .strafeTo(new Vector2d(57,-64.5))
                .waitSeconds(0.001);
        TrajectoryActionBuilder driveOutOfZoneSecondSpec = push2SamplesGrabSpec.fresh()
                .waitSeconds(0.5)
                .strafeTo(new Vector2d(57,-60))
                .waitSeconds(0.001);
        TrajectoryActionBuilder goToSubSecondSpec = driveOutOfZoneSecondSpec.fresh()
                .waitSeconds(0.2)
                .strafeToLinearHeading(new Vector2d(2,-45), Math.toRadians(270)) //change heading
                .strafeTo(new Vector2d(2, -32))
                .waitSeconds(0.001);
        TrajectoryActionBuilder driveBackToPutSlidesDownThirdSpec = goToSubSecondSpec.fresh()
                .waitSeconds(0.001)
                .waitSeconds(0.3)
                .strafeTo(new Vector2d(2,-40))
                .strafeToLinearHeading(new Vector2d(40,-53), Math.toRadians(90))
                .waitSeconds(0.001);
        TrajectoryActionBuilder goToZoneThirdSpec = driveBackToPutSlidesDownThirdSpec.fresh()
                .waitSeconds(0.2)
                .strafeTo(new Vector2d(58,-64.5))
                .waitSeconds(0.001);
        TrajectoryActionBuilder driveOutOfZoneThirdSpec = goToZoneThirdSpec.fresh()
                .waitSeconds(0.5)
                .strafeTo(new Vector2d(58,-58))
                .waitSeconds(0.001);
        TrajectoryActionBuilder goToSubThirdSpec = driveOutOfZoneThirdSpec.fresh()
                .waitSeconds(0.3)
                .strafeToLinearHeading(new Vector2d(-1,-35), Math.toRadians(270)) //change heading
                .waitSeconds(0.001)
                .strafeTo(new Vector2d(-1,-31))
                .waitSeconds(0.3);
        TrajectoryActionBuilder driveBackToPutSlidesDownFourthSpec = goToSubThirdSpec.fresh()
                .waitSeconds(0.001)
                .strafeTo(new Vector2d(-1,-40))
                .strafeToLinearHeading(new Vector2d(40,-53), Math.toRadians(90))
                .waitSeconds(0.001);
        TrajectoryActionBuilder goToZoneFourthSpec = driveBackToPutSlidesDownFourthSpec.fresh()
                .waitSeconds(0.001)
                .strafeTo(new Vector2d(58,-64.5))
                .waitSeconds(0.001);
        TrajectoryActionBuilder driveOutOfZoneFourthSpec = goToZoneFourthSpec.fresh()
                .waitSeconds(0.3)
                .strafeTo(new Vector2d(58,-58))
                .waitSeconds(0.001);
        TrajectoryActionBuilder goToSubFourthSpec = driveOutOfZoneFourthSpec.fresh()
                .waitSeconds(0.3)
                .strafeToLinearHeading(new Vector2d(1,-33), Math.toRadians(270)) //change heading
                .waitSeconds(0.001);
        TrajectoryActionBuilder goBackAndPark = goToSubFourthSpec.fresh()
                .waitSeconds(1)
                .strafeTo(new Vector2d(2,-40))
                .strafeToLinearHeading(new Vector2d(45,-58), Math.toRadians(90))
                .waitSeconds(0.001);

        Action moveAwayFromBarrierAction = moveAwayFromBarrier.build();
        Action moveIntoSpec1PositionAction = moveIntoSpec1Position.build();
        Action driveBackAction = driveBack.build();
        Action push2SamplesGrabSpecAction = push2SamplesGrabSpec.build();
        Action driveOutOfZoneSecondSpecAction = driveOutOfZoneSecondSpec.build();
        Action goToSubSecondSpecAction = goToSubSecondSpec.build();
        Action driveBackToPutSlidesDownThirdSpecAction = driveBackToPutSlidesDownThirdSpec.build();
        Action goToZoneThirdSpecAction = goToZoneThirdSpec.build();
        Action driveOutOfZoneThirdSpecAction = driveOutOfZoneThirdSpec.build();
        Action goToSubThirdSpecAction = goToSubThirdSpec.build();
        Action driveBackToPutSlidesDownFourthSpecAction = driveBackToPutSlidesDownFourthSpec.build();
        Action goToZoneFourthSpecAction = goToZoneFourthSpec.build();
        Action driveOutOfZoneFourthSpecAction = driveOutOfZoneFourthSpec.build();
        Action goToSubFourthSpecAction = goToSubFourthSpec.build();
        Action goBackAndParkAction = goBackAndPark.build();

        Action autoSequence = new SequentialAction(
                specClaw.closeClaw(),
                moveAwayFromBarrierAction,// Move away from the barrier
                intakeArm.intakeArmDown(),
                slides.slidesToAboveBar(), // Move the slides to above the bar to prepare for the first spec
                moveIntoSpec1PositionAction, // Move into position to place the first spec
                slides.slidesToBelowBar(), // Clip the spec onto the bar
                new SleepAction(0.2),
                specClaw.openClaw(), // Release the spec
                driveBackAction, //drive back to put slides fully down
                slides.slidesToSpecPickup(), //brings slides to pos 0 (fully down)
                push2SamplesGrabSpecAction, //pushes sample into player person zone, then grabs spec
                new SleepAction(0.5), //wait for player person
                specClaw.closeClaw(), // Hold onto Spec
                slides.slidesToSlightlyAboveWall(), //brings slides slightly above wall to reduce belt slip and let drivetrain drive back
                driveOutOfZoneSecondSpecAction, //drive out of zone to put slides up
                slides.slidesToAboveBar(), //bring slides up to Spec Position
                goToSubSecondSpecAction, //go to sub to clip second spec
                slides.slidesToBelowBar(), //clip on spec
                new SleepAction(0.2),
                specClaw.openClaw(), // Release the spec
                driveBackToPutSlidesDownThirdSpecAction, //drive back to put slides down
                slides.slidesToSpecPickup(), //brings slides to pos 0 (fully down)
                goToZoneThirdSpecAction, //go to zone to get third spec
                new SleepAction(0.5), //wait for player person
                specClaw.closeClaw(), //close claw to hold onto 3rd spec
                slides.slidesToSlightlyAboveWall(), //brings slides slightly above wall to reduce belt slip and let drivetrain drive back
                driveOutOfZoneThirdSpecAction, //drive out of zone to bring slides up
                slides.slidesToAboveBar(), //bring slides up to Spec Position
                goToSubThirdSpecAction,// go clip third spec
                slides.slidesToBelowBar(), //clip on spec
                new SleepAction(0.2),
                specClaw.openClaw(), // Release the spec
                driveBackToPutSlidesDownFourthSpecAction, //drive back to put slides down
                slides.slidesToSpecPickup(), //brings slides to pos 0 (fully down)
                goToZoneFourthSpecAction, //go to zone to get fourth spec
                new SleepAction(0.5), //wait for player person
                specClaw.closeClaw(), //close claw to hold onto 4th spec
                new SleepAction(0.2),
                slides.slidesToSlightlyAboveWall(), //brings slides slightly above wall to reduce belt slip and let drivetrain drive back
                driveOutOfZoneFourthSpecAction, //drive out of zone to bring slides up
                slides.slidesToAboveBar(), //bring slides up to Spec Position
                goToSubFourthSpecAction,// go clip fourth spec
                slides.slidesToBelowBar(), //clip on spec
                new SleepAction(0.2),
                specClaw.openClaw(), // Release the spec
                goBackAndParkAction, //park in player person zone
                slides.slidesToSpecPickup(), //brings slides to pos 0 (fully down)
                new SleepAction(1) //wait for slides to come down
        );
        Action pidControlLoops = new ParallelAction(
                slides.slidesPIDIteration(), //cycling through slides to hold PID
                intakeArm.armPIDIteration() //cycling through intakeArm to hold PID
        );

        waitForStart();
        Actions.runBlocking(new ParallelAction(autoSequence, pidControlLoops));
    }
}