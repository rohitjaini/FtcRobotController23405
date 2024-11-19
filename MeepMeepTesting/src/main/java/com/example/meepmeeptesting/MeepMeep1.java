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

public class MeepMeep1 {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(90, 80, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        Pose2d beginPose = new Pose2d(13, -61.5, Math.toRadians(270));

        TrajectoryActionBuilder moveAwayFromBarrier = myBot.getDrive().actionBuilder(beginPose)
                .strafeTo(new Vector2d(13, -50)) //move forward to let arm go back
                .waitSeconds(0.001);
        TrajectoryActionBuilder moveIntoSpec1Position = moveAwayFromBarrier.fresh()
                .waitSeconds(1.5) // wait for slides to go up
                .strafeTo(new Vector2d(0, -25)) // go to sub to clip spec
                .waitSeconds(0.001);
        TrajectoryActionBuilder driveBack = moveIntoSpec1Position.fresh()
                .strafeTo(new Vector2d(0, -35)) // drive back from the sub to push sample
                .waitSeconds(0.001);
        TrajectoryActionBuilder pushSampleGrabSpec = driveBack.fresh()
                .strafeTo(new Vector2d(43, -35)) // go to the right
                .strafeTo(new Vector2d(43, -10)) // go up field
                .splineTo(new Vector2d(53, -10), Math.toRadians(270)) //spline to push sample (turns 180 NOT relative)
                .strafeTo(new Vector2d(53, -57)) //push spec into player person zone
                .strafeTo(new Vector2d(53, -45)) //come out to let player person clip spec on wall
                .strafeTo(new Vector2d(53, -63.7)) //go in to zone again to grab spec
                .waitSeconds(0.001);
        TrajectoryActionBuilder goToSubSecondSpec = pushSampleGrabSpec.fresh()
                .waitSeconds(0.5)
                .strafeTo(new Vector2d(53, -45)) //strafe up field
                .strafeToLinearHeading(new Vector2d(4,-45),Math.toRadians(270)) //change heading
                .strafeTo(new Vector2d(4, -25))
                .waitSeconds(0.001);
        TrajectoryActionBuilder goBackAndPark = goToSubSecondSpec.fresh()
                .waitSeconds(1)
                .strafeTo(new Vector2d(4, -45))
                .strafeToLinearHeading(new Vector2d(47,-45), Math.toRadians(90))
                .strafeTo(new Vector2d(47, -45))
                .strafeTo(new Vector2d(47, -54))
                .waitSeconds(0.001);

        Action moveAwayFromBarrierAction = moveAwayFromBarrier.build();
        Action moveIntoSpec1PositionAction = moveIntoSpec1Position.build();
        Action driveBackAction = driveBack.build();
        Action pushSampleGrabSpecAction = pushSampleGrabSpec.build();
        Action goToSubSecondSpecAction = goToSubSecondSpec.build();
        Action goBackAndParkAction = goBackAndPark.build();

        Action autoSequence = new SequentialAction(
                moveAwayFromBarrierAction, // Move away from the barrier
                moveIntoSpec1PositionAction, // Move into position to place the first spec
                driveBackAction, //drive back to put slides fully down
                pushSampleGrabSpecAction, //pushes sample into player person zone, then grabs spec
                goToSubSecondSpecAction, //go to sub to put on second spec
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