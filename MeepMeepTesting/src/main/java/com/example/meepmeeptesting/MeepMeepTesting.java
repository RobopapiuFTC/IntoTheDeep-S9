package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class MeepMeepTesting {
    public static void  main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setColorScheme(new ColorSchemeBlueLight())
                .setConstraints(100, 100, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(10.2, -60.3, Math.toRadians(270)))
                .strafeTo(new Vector2d(7,-30.3))
                .strafeToSplineHeading(new Vector2d(27,-50), Math.toRadians(30))
                .strafeToSplineHeading(new Vector2d(28.5,-32), Math.toRadians(25))
                .strafeToSplineHeading(new Vector2d(35,-35), Math.toRadians(285))
                .strafeToSplineHeading(new Vector2d(35,-31), Math.toRadians(15))
                .strafeToSplineHeading(new Vector2d(37,-35), Math.toRadians(285))
                .strafeToSplineHeading(new Vector2d(35,-55), Math.toRadians(90))
                .strafeToSplineHeading(new Vector2d(35,-67.5), Math.toRadians(90))
                        .setTangent(1)
                .splineToLinearHeading(new Pose2d(-4, -28.3, Math.toRadians(270)), Math.toRadians(90))
                        .setTangent(-1)
                .splineToLinearHeading(new Pose2d(40, -55, Math.toRadians(90)), Math.toRadians(180))
                .strafeToSplineHeading(new Vector2d(40,-67.5), Math.toRadians(90))
                .setTangent(1)
                .splineToLinearHeading(new Pose2d(-8, -28.3, Math.toRadians(270)), Math.toRadians(90))
                .setTangent(-1)
                .splineToLinearHeading(new Pose2d(40, -55, Math.toRadians(90)), Math.toRadians(270))
                .strafeToSplineHeading(new Vector2d(40,-67.5), Math.toRadians(90))
                .setTangent(1)
                .splineToLinearHeading(new Pose2d(-12, -28.3, Math.toRadians(270)), Math.toRadians(90))
                .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}