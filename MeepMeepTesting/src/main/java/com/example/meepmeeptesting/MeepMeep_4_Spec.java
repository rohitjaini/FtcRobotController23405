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

        RoadRunnerBotEntity drive = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(140, 90, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        Pose2d beginPose = new Pose2d(15, -61.5, Math.toRadians(270));

        TrajectoryActionBuilder moveAwayFromBarrier = drive.getDrive().actionBuilder(beginPose)
                .strafeTo(new Vector2d(15, -50))
                .waitSeconds(0.001);
        TrajectoryActionBuilder moveIntoSpec1Position = moveAwayFromBarrier.fresh()
                .waitSeconds(0.2)
                .strafeTo(new Vector2d(0, -32))
                .waitSeconds(0.001);
        TrajectoryActionBuilder driveBack = moveIntoSpec1Position.fresh()
                .waitSeconds(0.3)
                .strafeTo(new Vector2d(0, -40))
                .waitSeconds(0.001);
        TrajectoryActionBuilder push2SamplesGrabSpec = driveBack.fresh()
                .waitSeconds(0.001)
                .strafeToLinearHeading(new Vector2d(37, -40), Math.toRadians(90)) // go to the right
                .strafeTo(new Vector2d(37, -16))
                .splineToConstantHeading(new Vector2d(48, -16), Math.toRadians(270))
                .strafeTo(new Vector2d(47, -50))
                .strafeTo(new Vector2d(47,-19))
                .splineToConstantHeading(new Vector2d(57,-20), Math.toRadians(270))
                .strafeTo(new Vector2d(57,-50))
                .waitSeconds(0.2)
                .strafeTo(new Vector2d(52,-44))
                .strafeTo(new Vector2d(47,-61.5))
                .waitSeconds(0.001);
        TrajectoryActionBuilder driveOutOfZoneSecondSpec = push2SamplesGrabSpec.fresh()
                .waitSeconds(0.5)

                .waitSeconds(0.001);
        TrajectoryActionBuilder goToSubSecondSpec = driveOutOfZoneSecondSpec.fresh()
                .waitSeconds(0.2)
                .strafeToLinearHeading(new Vector2d(2,-34), Math.toRadians(270)) //change heading
                .strafeTo(new Vector2d(2, -32))
                .waitSeconds(0.001);
        TrajectoryActionBuilder driveBackToPutSlidesDownThirdSpec = goToSubSecondSpec.fresh()
                .waitSeconds(0.001)
                .waitSeconds(0.3)
                .strafeTo(new Vector2d(2,-35))
                .strafeToLinearHeading(new Vector2d(40,-53), Math.toRadians(90))
                .waitSeconds(0.001);
        TrajectoryActionBuilder goToZoneThirdSpec = driveBackToPutSlidesDownThirdSpec.fresh()
                .waitSeconds(0.2)
                .strafeTo(new Vector2d(58,-61.5))
                .waitSeconds(0.001);
        TrajectoryActionBuilder driveOutOfZoneThirdSpec = goToZoneThirdSpec.fresh()
                .waitSeconds(0.5)
                .waitSeconds(0.001);
        TrajectoryActionBuilder goToSubThirdSpec = driveOutOfZoneThirdSpec.fresh()
                .waitSeconds(0.3)
                .strafeToLinearHeading(new Vector2d(-1,-34), Math.toRadians(270)) //change heading
                .strafeTo(new Vector2d(-1, -33))
                .waitSeconds(0.001);
        TrajectoryActionBuilder driveBackToPutSlidesDownFourthSpec = goToSubThirdSpec.fresh()
                .waitSeconds(0.001)
                .waitSeconds(0.3)
                .strafeTo(new Vector2d(0,-35))
                .strafeToLinearHeading(new Vector2d(40,-53), Math.toRadians(90))
                .waitSeconds(0.001);
        TrajectoryActionBuilder goToZoneFourthSpec = driveBackToPutSlidesDownFourthSpec.fresh()
                .waitSeconds(0.2)
                .strafeTo(new Vector2d(58,-61.5))
                .waitSeconds(0.001);
        TrajectoryActionBuilder driveOutOfZoneFourthSpec = goToZoneFourthSpec.fresh()
                .waitSeconds(0.5)
                .waitSeconds(0.001);
        TrajectoryActionBuilder goToSubFourthSpec = driveOutOfZoneFourthSpec.fresh()
                .waitSeconds(0.3)
                .strafeToLinearHeading(new Vector2d(1,-45), Math.toRadians(270)) //change heading
                .strafeTo(new Vector2d(1, -33))
                .waitSeconds(0.001);
        TrajectoryActionBuilder goBackAndPark = goToSubFourthSpec.fresh()
                .waitSeconds(1)
                .strafeTo(new Vector2d(1, -45))
                .strafeToLinearHeading(new Vector2d(55,-57), Math.toRadians(90))
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
                goBackAndParkAction //park in player person zone
        );

        drive.runAction(autoSequence);

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(drive)
                .start();
    }
}