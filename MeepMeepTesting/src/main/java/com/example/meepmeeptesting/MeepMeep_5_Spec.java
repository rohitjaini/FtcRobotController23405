package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeep_5_Spec {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(140, 90, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        Pose2d beginPose = new Pose2d(15, -61.5, Math.toRadians(270));

        TrajectoryActionBuilder moveAwayFromBarrier = myBot.getDrive().actionBuilder(beginPose)
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
                .strafeTo(new Vector2d(57,-20))
                .splineToConstantHeading(new Vector2d(63,-20), Math.toRadians(270))
                .strafeTo(new Vector2d(63,-58))
                .strafeTo(new Vector2d(63,-40))
                .waitSeconds(0.2)
                .strafeTo(new Vector2d(63,-64.5))
                .waitSeconds(0.001);
        TrajectoryActionBuilder driveOutOfZoneSecondSpec = push2SamplesGrabSpec.fresh()
                .waitSeconds(0.5)
                .strafeTo(new Vector2d(57,-55))
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
                .waitSeconds(0.3)
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
                .waitSeconds(0.001)
                .strafeToLinearHeading(new Vector2d(1,-33), Math.toRadians(270)) //change heading
                .waitSeconds(0.3);
        TrajectoryActionBuilder driveBackToPutSlidesDownFifthSpec = goToSubFourthSpec.fresh()
                .waitSeconds(0.001)
                .strafeTo(new Vector2d(1,-40))
                .strafeToLinearHeading(new Vector2d(40,-53), Math.toRadians(90))
                .waitSeconds(0.001);
        TrajectoryActionBuilder goToZoneFifthSpec = driveBackToPutSlidesDownFifthSpec.fresh()
                .waitSeconds(0.001)
                .strafeTo(new Vector2d(58,-64.5))
                .waitSeconds(0.001);
        TrajectoryActionBuilder driveOutOfZoneFifthSpec = goToZoneFifthSpec.fresh()
                .waitSeconds(0.3)
                .strafeTo(new Vector2d(58,-58))
                .waitSeconds(0.001);
        TrajectoryActionBuilder goToSubFifthSpec = driveOutOfZoneFifthSpec.fresh()
                .waitSeconds(0.001)
                .strafeToLinearHeading(new Vector2d(3,-33), Math.toRadians(270)) //change heading
                .waitSeconds(0.001);
        TrajectoryActionBuilder goBackAndPark = goToSubFifthSpec.fresh()
                .waitSeconds(1)
                .strafeTo(new Vector2d(3,-40))
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
        Action driveBackToPutSlidesDownFifthSpecAction = driveBackToPutSlidesDownFifthSpec.build();
        Action goToZoneFifthSpecAction = goToZoneFifthSpec.build();
        Action driveOutOfZoneFifthSpecAction = driveOutOfZoneFifthSpec.build();
        Action goToSubFifthSpecAction = goToSubFifthSpec.build();
        Action goBackAndParkAction = goBackAndPark.build();

        Action autoSequence = new SequentialAction(
                moveAwayFromBarrierAction, // Move away from the barrier
                moveIntoSpec1PositionAction, // Move into position to place the first spec
                driveBackAction, //drive back to put slides fully down
                push2SamplesGrabSpecAction, //pushes sample into player person zone, then grabs spec
                driveOutOfZoneSecondSpecAction, //drive out of zone to put slides up
                goToSubSecondSpecAction, //go to sub to clip second spec
                driveBackToPutSlidesDownThirdSpecAction, //drive back to put slides down
                goToZoneThirdSpecAction, //go to zone to get third spec
                driveOutOfZoneThirdSpecAction, //drive out of zone to bring slides up
                goToSubThirdSpecAction, //clip third spec
                driveBackToPutSlidesDownFourthSpecAction, //drive back to put slides down
                goToZoneFourthSpecAction, //go to zone to get fourth spec
                driveOutOfZoneFourthSpecAction, //drive out of zone to bring slides up
                goToSubFourthSpecAction, //clip fourth spec
                driveBackToPutSlidesDownFifthSpecAction,
                goToZoneFifthSpecAction,
                driveOutOfZoneFifthSpecAction,
                goToSubFifthSpecAction,
                goBackAndParkAction //park in player person zone
        );

        myBot.runAction(autoSequence);

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}