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

public class MeepMeep2 {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(140, 90, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        Pose2d beginPose = new Pose2d(13, -61.5, Math.toRadians(270));

        TrajectoryActionBuilder moveAwayFromBarrier = myBot.getDrive().actionBuilder(beginPose)
                .strafeTo(new Vector2d(13, -50))
                .waitSeconds(0.001);
        TrajectoryActionBuilder moveIntoSpec1Position = moveAwayFromBarrier.fresh()
                .waitSeconds(1.5)
                .strafeTo(new Vector2d(0, -33))
                .waitSeconds(0.001);
        TrajectoryActionBuilder driveBack = moveIntoSpec1Position.fresh()
                .waitSeconds(0.3)
                .strafeTo(new Vector2d(0, -40))
                .waitSeconds(0.001);
        TrajectoryActionBuilder push2SamplesGrabSpec = driveBack.fresh()
                .waitSeconds(0.001)
                .strafeTo(new Vector2d(45, -40)) // go to the right
                .strafeTo(new Vector2d(45, -20))
                .splineTo(new Vector2d(57, -20), Math.toRadians(270))
                .strafeTo(new Vector2d(57, -57))
                .strafeTo(new Vector2d(57, -20))
                .waitSeconds(0.5)
                .strafeTo(new Vector2d(67,-20))
                .strafeTo(new Vector2d(67,-58))
                .strafeTo(new Vector2d(58,-62.5))
                .waitSeconds(0.001);
        TrajectoryActionBuilder driveOutOfZoneSecondSpec = push2SamplesGrabSpec.fresh()
                .waitSeconds(0.2)
                .strafeTo(new Vector2d(53, -45)) //strafe up field
                .waitSeconds(0.001);
        TrajectoryActionBuilder goToSubSecondSpec = driveOutOfZoneSecondSpec.fresh()
                .waitSeconds(0.2)
                .strafeToLinearHeading(new Vector2d(2,-45), Math.toRadians(270)) //change heading
                .strafeTo(new Vector2d(2, -33))
                .waitSeconds(0.001);
        TrajectoryActionBuilder driveBackToPutSlidesDownThirdSpec = goToSubSecondSpec.fresh()
                .waitSeconds(0.001)
                .waitSeconds(0.3)
                .strafeTo(new Vector2d(2,-35))
                .strafeToLinearHeading(new Vector2d(40,-53), Math.toRadians(90))
                .waitSeconds(0.001);
        TrajectoryActionBuilder goToZoneThirdSpec = driveBackToPutSlidesDownThirdSpec.fresh()
                .waitSeconds(0.2)
                .strafeTo(new Vector2d(58,-63.5))
                .waitSeconds(0.001);
        TrajectoryActionBuilder driveOutOfZoneThirdSpec = goToZoneThirdSpec.fresh()
                .waitSeconds(0.5)
                .strafeTo(new Vector2d(58, -45)) //strafe up field
                .waitSeconds(0.001);
        TrajectoryActionBuilder goToSubThirdSpec = driveOutOfZoneThirdSpec.fresh()
                .waitSeconds(0.3)
                .strafeToLinearHeading(new Vector2d(-4,-45), Math.toRadians(270)) //change heading
                .strafeTo(new Vector2d(-4, -33))
                .waitSeconds(0.001);
        TrajectoryActionBuilder goBackAndPark = goToSubThirdSpec.fresh()
                .waitSeconds(1)
                .strafeTo(new Vector2d(-4, -45))
                .strafeToLinearHeading(new Vector2d(55,-60), Math.toRadians(90))
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