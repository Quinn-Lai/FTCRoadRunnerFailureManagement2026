package com.example.meepmeep;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTest {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        Pose2d spawn = new Pose2d(-52, 48.5, Math.toRadians(305.954));
        Pose2d shootingPos = new Pose2d(-15, 15, Math.toRadians(135));

        Pose2d lineBot = new Pose2d(34, 28, Math.toRadians(90));
        Pose2d lineBotCollect = new Pose2d(34, 52, Math.toRadians(90));
        Pose2d lineMid = new Pose2d(11, 28, Math.toRadians(90));
        Pose2d lineMidCollect = new Pose2d(11, 52, Math.toRadians(90));
        Pose2d lineTop = new Pose2d(-12, 28, Math.toRadians(90));
        Pose2d lineTopCollect = new Pose2d(-12, 49, Math.toRadians(90));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(16.945,17.87)
                .build();


        myBot.runAction(myBot.getDrive().actionBuilder(spawn)

                        .setTangent(Math.toRadians(330))
                        .splineToLinearHeading(shootingPos,Math.toRadians(330))

                        .setTangent(Math.toRadians(45))
                        .splineToLinearHeading(lineTop, Math.toRadians(90))

                        .setTangent(Math.toRadians(90))
                        .splineToLinearHeading(lineTopCollect,Math.toRadians(90))

                        .setTangent(Math.toRadians(270))
                        .splineToLinearHeading(shootingPos,Math.toRadians(270))

                        .setTangent(Math.toRadians(0))
                        .splineToLinearHeading(lineMid, Math.toRadians(90))

                        .setTangent(Math.toRadians(90))
                        .splineToLinearHeading(lineMidCollect,Math.toRadians(90))

                        .setTangent(Math.toRadians(225))
                        .splineToLinearHeading(shootingPos,Math.toRadians(270))

                        .setTangent(Math.toRadians(0))
                        .splineToLinearHeading(lineBot, Math.toRadians(90))

                        .setTangent(Math.toRadians(90))
                        .splineToLinearHeading(lineBotCollect,Math.toRadians(90))

                        .setTangent(Math.toRadians(225))
                        .splineToLinearHeading(shootingPos,Math.toRadians(225))


                        .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}

//.setTangent(Math.toRadians(180))
//                        .splineToLinearHeading(shootingPos,Math.toRadians(180))
//                        .setTangent(Math.toRadians(200))
//                        .splineToLinearHeading(lineBot, Math.toRadians(270))
//                        .setTangent(Math.toRadians(270))
//                        .splineToLinearHeading(lineBotCollect, Math.toRadians(270), new TranslationalVelConstraint(5))
//                        .setTangent(Math.toRadians(90))
//                        .splineToLinearHeading(shootingPos,Math.toRadians(20))
//                        .setTangent(Math.toRadians(200))
//                        .splineToLinearHeading(lineMid, Math.toRadians(180))
//                        .setTangent(Math.toRadians(270))
//                        .splineToLinearHeading(lineMidCollect, Math.toRadians(270), new TranslationalVelConstraint(5))
//                        .setTangent(Math.toRadians(45))
//                        .splineToLinearHeading(shootingPos,Math.toRadians(20))
//                        .setTangent(Math.toRadians(200))
//                        .splineToLinearHeading(lineTop, Math.toRadians(180))
//                        .setTangent(Math.toRadians(270))
//                        .splineToLinearHeading(lineTopCollect, Math.toRadians(270), new TranslationalVelConstraint(5))
//                        .setTangent(Math.toRadians(10))
//                        .splineToLinearHeading(shootingPos,Math.toRadians(20))

//                .setTangent(Math.toRadians(180))
//                .splineToLinearHeading(shootingPos,Math.toRadians(180))
//
//                .setTangent(Math.toRadians(150))
//                .splineToSplineHeading(lineBot, Math.toRadians(90))
//                .setTangent(Math.toRadians(90))
//                .splineToSplineHeading(lineBotCollect, Math.toRadians(90), new TranslationalVelConstraint(10))
//                .setTangent(Math.toRadians(270))
//                .splineToLinearHeading(shootingPos,Math.toRadians(340))
//
//                .setTangent(Math.toRadians(150))
//                .splineToLinearHeading(lineMid, Math.toRadians(180)) //270 maybe
//                .setTangent(Math.toRadians(90))
//                .splineToLinearHeading(lineMidCollect, Math.toRadians(90),  new TranslationalVelConstraint(10))
//                .setTangent(Math.toRadians(315))
//                .splineToLinearHeading(shootingPos,Math.toRadians(315))
//
//                .setTangent(Math.toRadians(150))
//                .splineToLinearHeading(lineTop, Math.toRadians(180))
//                .setTangent(Math.toRadians(90))
//                .splineToLinearHeading(lineTopCollect, Math.toRadians(90),  new TranslationalVelConstraint(10))
//                .setTangent(Math.toRadians(350))
//                .splineToLinearHeading(shootingPos,Math.toRadians(340))