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

        Pose2d beginPose = new Pose2d(15, -61.5, Math.toRadians(270));

        TrajectoryActionBuilder moveAwayFromBarrier = myBot.getDrive().actionBuilder(beginPose)
                .strafeTo(new Vector2d(15, -50))
                .waitSeconds(0.001);
        TrajectoryActionBuilder moveIntoSpec1Position = moveAwayFromBarrier.fresh()
                .waitSeconds(1)
                .strafeTo(new Vector2d(0, -25))
                .waitSeconds(0.001);
        TrajectoryActionBuilder driveBack = moveIntoSpec1Position.fresh()
                .waitSeconds(0.5)
                .strafeTo(new Vector2d(0, -37))
                .waitSeconds(0.001);
        TrajectoryActionBuilder push2SamplesGrabSpec = driveBack.fresh()
                .splineToConstantHeading(new Vector2d(45,-9),Math.PI/2)
                .splineTo(new Vector2d(53, -10), Math.toRadians(270))
                .strafeTo(new Vector2d(53, -57))
                .strafeTo(new Vector2d(53, -10))
                .splineToConstantHeading(new Vector2d(64,-58), Math.toRadians(270))
                .strafeTo(new Vector2d(58,-63.5))
                .waitSeconds(0.001);
        TrajectoryActionBuilder goToSubSecondSpec = push2SamplesGrabSpec.fresh()
                .waitSeconds(0.5)
                .strafeTo(new Vector2d(53, -45)) //strafe up field
                .strafeToLinearHeading(new Vector2d(4,-45), Math.toRadians(270)) //change heading
                .strafeTo(new Vector2d(4, -25))
                .waitSeconds(0.001)
                .waitSeconds(0.5);
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
                .waitSeconds(0.001)
                .waitSeconds(0.5);
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
                moveAwayFromBarrierAction,
                moveIntoSpec1PositionAction,
                driveBackAction,
                push2SamplesGrabSpecAction,
                goToSubSecondSpecAction,
                goToZoneThirdSpecAction,
                goToSubThirdSpecAction,
                goBackAndParkAction
        );

        myBot.runAction(autoSequence);

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}