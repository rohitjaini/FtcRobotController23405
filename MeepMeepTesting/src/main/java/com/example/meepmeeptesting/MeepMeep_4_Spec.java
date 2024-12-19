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

public class MeepMeep_4_Spec {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(200, 150, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        Pose2d beginPose = new Pose2d(14, -61.5, Math.toRadians(270));// where the sigma is it starting

        TrajectoryActionBuilder moveAwayFromBarrier = myBot.getDrive().actionBuilder(beginPose)
                .strafeTo(new Vector2d(13, -50))// move the sigma away from the wall
                .waitSeconds(0.001);
        TrajectoryActionBuilder moveIntoSpec1Position = moveAwayFromBarrier.fresh()
                .waitSeconds(1.5)
                .strafeTo(new Vector2d(0, -33))//move the sigma into spec1 scoring position
                .waitSeconds(0.001);
        TrajectoryActionBuilder driveBack = moveIntoSpec1Position.fresh()// OK FINE. ILL STOP SAYING "SIGMA"
                .waitSeconds(0.5)// SIGMA
                .strafeTo(new Vector2d(0, -37))// move out of specs mode
                .waitSeconds(0.001);
        TrajectoryActionBuilder push3SamplesGrabSpec = driveBack.fresh()
                .waitSeconds(0.001)
                .strafeTo(new Vector2d(38, -37)) // go to the right
                .strafeTo(new Vector2d(38, -20)) // go forward
                .splineTo(new Vector2d(48, -20), Math.toRadians(270)) // right there about to make contact with sample 1
                .strafeTo(new Vector2d(47, -57)) //push sample 1 into raddy zone
                .strafeTo(new Vector2d(47, -13)) //get ready to push sample 2
                .waitSeconds(0.5)
                .strafeTo(new Vector2d(57,-13)) // almost made contact with sample 2
                .strafeTo(new Vector2d(57,-58)) // put sample 2 in raddy zone. super sigma
                .strafeTo(new Vector2d(58,-60))// navigate near raddy
                .strafeTo(new Vector2d(58,-63.5))// RADDY put the spec down
                .waitSeconds(0.001);
        TrajectoryActionBuilder goToSubSecondSpec = push3SamplesGrabSpec.fresh()
                .waitSeconds(0.5)
                .strafeTo(new Vector2d(53, -45)) //strafe up field
                .strafeToLinearHeading(new Vector2d(0,-45), Math.toRadians(270)) //change heading
                .strafeTo(new Vector2d(2, -33))
                .waitSeconds(0.001);
        TrajectoryActionBuilder goToZoneThirdSpec = goToSubSecondSpec.fresh()
                .waitSeconds(0.001)
                .waitSeconds(0.3)
                .strafeTo(new Vector2d(2,-35))
                .strafeToLinearHeading(new Vector2d(40,-53), Math.toRadians(90))
                .strafeTo(new Vector2d(58,-63.5))
                .waitSeconds(0.001);
        TrajectoryActionBuilder goToSubThirdSpec = goToZoneThirdSpec.fresh()
                .waitSeconds(0.5)
                .strafeTo(new Vector2d(58, -45)) //strafe up field
                .strafeToLinearHeading(new Vector2d(4,-45), Math.toRadians(270)) //change heading
                .strafeTo(new Vector2d(4, -33))
                .waitSeconds(0.001)
                .waitSeconds(0.3);
        TrajectoryActionBuilder goToZoneFourthSpec = goToSubThirdSpec.fresh()
                .waitSeconds(0.001)
                .strafeTo(new Vector2d(1,-35))
                .strafeToLinearHeading(new Vector2d(40,-53), Math.toRadians(90))
                .strafeTo(new Vector2d(58,-63.5))
                .waitSeconds(0.001);
        TrajectoryActionBuilder goToSubFourthSpec = goToZoneFourthSpec.fresh()
                .waitSeconds(0.5)
                .strafeTo(new Vector2d(58, -45)) //strafe up field
                .strafeToLinearHeading(new Vector2d(-2,-45), Math.toRadians(270)) //change heading
                .strafeTo(new Vector2d(-2, -33))
                .waitSeconds(0.001)
                .waitSeconds(0.5);
        TrajectoryActionBuilder goBackAndPark = goToSubFourthSpec.fresh()
                .waitSeconds(0.3)
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(55,-58), Math.toRadians(90))
                .waitSeconds(0.001);

        Action moveAwayFromBarrierAction = moveAwayFromBarrier.build();
        Action moveIntoSpec1PositionAction = moveIntoSpec1Position.build();
        Action driveBackAction = driveBack.build();
        Action push3SamplesGrabSpecAction = push3SamplesGrabSpec.build();
        Action goToSubSecondSpecAction = goToSubSecondSpec.build();
        Action goToZoneThirdSpecAction = goToZoneThirdSpec.build();
        Action goToSubThirdSpecAction = goToSubThirdSpec.build();
        Action goToZoneFourthSpecAction = goToZoneFourthSpec.build();
        Action goToSubFourthSpecAction = goToSubFourthSpec.build();
        Action goBackAndParkAction = goBackAndPark.build();

        Action autoSequence = new SequentialAction(
                moveAwayFromBarrierAction,
                moveIntoSpec1PositionAction,
                driveBackAction,
                push3SamplesGrabSpecAction,
                goToSubSecondSpecAction,
                goToZoneThirdSpecAction,
                goToSubThirdSpecAction,
                goToZoneFourthSpecAction,
                goToSubFourthSpecAction,
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