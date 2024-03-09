package com.tearabite.ielib.localization;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.LineSegment;

import org.apache.commons.math3.geometry.partitioning.BSPTree;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

public class InvisibleBarrierCore {
    public static double BACKOFF_FACTOR = 0.2;

//    public static Pose2d limitDriverInputRayCast(Pose2d inputVector, Pose2d robotPose, BSPTree<Vector2d> barriers) {
//        return null;
//    }


    public static Pose2d limitDriveInput(
            Pose2d driverInput,
            Pose2d currentRobotPose,
            double maximumXCoordinate,
            double robotRadius,
            double k) {
        Pose2d currentPose = currentRobotPose.plus(new Pose2d(robotRadius, 0, 0));
        Vector2d inputVector = driverInput.vec().rotated(-currentPose.getHeading());
        if (inputVector.getX() < 0) {
            return driverInput;
        }

        double projectedX = currentPose.getX() + (inputVector.getX() * k);
        double clampedX = Math.min(maximumXCoordinate, projectedX);
        double ratio = Math.max(-1, Math.min(1, clampedX / projectedX));
        if (currentPose.getX() > maximumXCoordinate && inputVector.getX() > 0) {
            ratio *= -1 * BACKOFF_FACTOR;
        }
        Vector2d clampedInputVector = new Vector2d(inputVector.getX() * ratio, inputVector.getY()).rotated(currentPose.getHeading());
        return new Pose2d(clampedInputVector, driverInput.getHeading());
    }



}
