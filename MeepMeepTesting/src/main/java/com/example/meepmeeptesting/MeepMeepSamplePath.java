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

public class MeepMeepSamplePath {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(90, 80, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        Pose2d beginPose = new Pose2d(-17, -61.5, Math.toRadians(90));

        TrajectoryActionBuilder moveAwayFromBarrier = myBot.getDrive().actionBuilder(beginPose)
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

        Action moveAwayFromBarrierAction = moveAwayFromBarrier.build();
        Action firstSampleDepositAction = firstSampleDeposit.build();
        Action intakeSecondSampleAction = intakeSecondSample.build();
        Action secondSampleDepositAction = secondSampleDeposit.build();

        Action autoSequence = new SequentialAction(
                moveAwayFromBarrierAction, // Move away from the barrier
                firstSampleDepositAction,
                intakeSecondSampleAction,
                secondSampleDepositAction
        );

        myBot.runAction(autoSequence);

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}