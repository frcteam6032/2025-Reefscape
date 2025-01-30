// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.filter.SlewRateLimiter;

/** Add your docs here. */
public class Utils {
    private static final double BASE_SPEED = 0.4;

    public static double scaleDriverController(double controllerInput, SlewRateLimiter limiter, double trigger) {
        return limiter.calculate(
                controllerInput * (BASE_SPEED
                        + trigger * (1 - BASE_SPEED)));
    }
}
