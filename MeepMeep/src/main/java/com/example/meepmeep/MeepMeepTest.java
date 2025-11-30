package com.example.meepmeep;

import com.acmerobotics.roadrunner.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTest {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        Pose2d spawn = new Pose2d(57, -10, Math.toRadians(180));
        Pose2d shootingPos = new Pose2d(57, -10, Math.toRadians(200));
        Pose2d lineBot = new Pose2d(34, -28, Math.toRadians(270));
        Pose2d lineBotCollect = new Pose2d(34, -55, Math.toRadians(270));
        Pose2d lineMid = new Pose2d(11, -28, Math.toRadians(270));
        Pose2d lineMidCollect = new Pose2d(11, -55, Math.toRadians(270));
        Pose2d lineTop = new Pose2d(-12, -28, Math.toRadians(270));
        Pose2d lineTopCollect = new Pose2d(-12, -51, Math.toRadians(270));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(spawn)

                .setTangent(Math.toRadians(200))
                .splineToLinearHeading(lineBot, Math.toRadians(180))
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(lineBotCollect, Math.toRadians(270))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(lineBot, Math.toRadians(90))
                .setTangent(0)
                .splineToLinearHeading(shootingPos,Math.toRadians(90))

                .setTangent(Math.toRadians(200))
                .splineToLinearHeading(lineMid, Math.toRadians(180))
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(lineMidCollect, Math.toRadians(270))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(lineMid, Math.toRadians(90))
                .setTangent(0)
                .splineToLinearHeading(shootingPos,Math.toRadians(90))

                .setTangent(Math.toRadians(200))
                .splineToLinearHeading(lineTop, Math.toRadians(180))
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(lineTopCollect, Math.toRadians(270))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(lineTop, Math.toRadians(90))
                .setTangent(0)
                .splineToLinearHeading(shootingPos,Math.toRadians(90))

                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}