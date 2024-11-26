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
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setColorScheme(new ColorSchemeBlueLight())
                .setConstraints(100, 100, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-12, 63, Math.toRadians(270)))
                .strafeTo(new Vector2d(-47,40))
                .waitSeconds(0.5)
                .strafeToSplineHeading(new Vector2d(-50,50), Math.toRadians(135))
                .waitSeconds(0.5)
                .strafeToSplineHeading(new Vector2d(-58,40), Math.toRadians(270))
                .waitSeconds(0.5)
                .strafeToSplineHeading(new Vector2d(-50,50), Math.toRadians(135))
                .waitSeconds(0.5)
                .strafeToSplineHeading(new Vector2d(-58,40), Math.toRadians(235))
                .waitSeconds(0.5)
                .strafeToSplineHeading(new Vector2d(-50,50), Math.toRadians(135))
                .waitSeconds(0.5)
                .strafeToSplineHeading(new Vector2d(-36,0), Math.toRadians(0))
                .strafeToSplineHeading(new Vector2d(-23,10), Math.toRadians(0))
                .waitSeconds(1)
                //.strafeToSplineHeading(new Vector2d(-30,0), Math.toRadians(0))
                .setTangent(Math.toRadians(90))
                .strafeToSplineHeading(new Vector2d(-45,8), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(0,50,Math.toRadians(0)), Math.toRadians(-10))
                .strafeToSplineHeading(new Vector2d(55,54), Math.toRadians(45))
                .waitSeconds(1)
                .strafeToSplineHeading(new Vector2d(-50,58), Math.toRadians(0))
                .build());

        RoadRunnerBotEntity mySecondBot = new DefaultBotBuilder(meepMeep)
                // We set this bot to be red
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(100, 100, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        mySecondBot.runAction(mySecondBot.getDrive().actionBuilder(new Pose2d(32, 63, Math.toRadians(270)))
                .strafeTo(new Vector2d(47,40))
                .waitSeconds(0.5)
                .strafeToSplineHeading(new Vector2d(55,52), Math.toRadians(45))
                .waitSeconds(0.5)
                .strafeToSplineHeading(new Vector2d(58,40), Math.toRadians(270))
                .waitSeconds(0.5)
                .strafeToSplineHeading(new Vector2d(55,54), Math.toRadians(45))
                .waitSeconds(0.5)
                .strafeToSplineHeading(new Vector2d(60,40), Math.toRadians(305))
                .waitSeconds(0.5)
                .strafeToSplineHeading(new Vector2d(55,53), Math.toRadians(45))
                .waitSeconds(0.5)
                .strafeToSplineHeading(new Vector2d(30,10), Math.toRadians(180))
                .waitSeconds(0.5)
                .setTangent(Math.toRadians(90))
                .strafeToSplineHeading(new Vector2d(40,10), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(53,52,Math.toRadians(45)), Math.toRadians(10))
                .waitSeconds(1)
                .strafeToSplineHeading(new Vector2d(50,50), Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(23,10,Math.toRadians(0)), Math.toRadians(150))
                .build());
        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .addEntity(mySecondBot)
                .start();
    }
}