package org.firstinspires.ftc.teamcode.commandbase;

public class armFeedForward {

    public static double calculate(double targetPosition, double ticksInDegrees, double initialPositionForFF, double f) {
        return Math.cos(Math.toRadians(targetPosition / ticksInDegrees) - Math.toRadians(initialPositionForFF) + Math.toRadians(90)) * f;
    }

}