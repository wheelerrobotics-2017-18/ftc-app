package org.wheelerschool.robotics.library.navigation;

import android.util.Log;

import com.qualcomm.robotcore.util.Range;

/**
 * This library allows for navigation by keeping a constant distance from a wall or side.
 * @author luciengaitskell
 * @version 1.0
 * @since 161202
 */

public class ConstantDistanceMotorNavigation {
    // Configuration:
    public static String LOG_TAG = "Motor Nav Calculation";

    // Constructor Set Config Values:
    private double nominalDistance;
    private double maximumValueSeparation;


    public ConstantDistanceMotorNavigation(double nominalDistance, double maximumValueSeparation) {
        this.nominalDistance = nominalDistance;
        this.maximumValueSeparation = maximumValueSeparation;
    }

    public class NavigationData {
        // Motor Power:
        public double closerMotorPower;
        public double fartherMotorPower;

        // Debug:
        public double rotationPower;
    }

    public NavigationData calculateNavigationData(double forwardPower, double sideDistance) {
        // Create NavigationData class:
        NavigationData navigationData = new NavigationData();

        // Calculate rotation power and clip to minimum and maximum values:
        double valueSeparation = (sideDistance - this.nominalDistance);
        // Calculate rotation power (center value: 0):
        navigationData.rotationPower = Range.clip(valueSeparation / this.maximumValueSeparation, -1, 1);

        // Farther motor power calculation:
        navigationData.closerMotorPower = (forwardPower) * (1 - navigationData.rotationPower);
        if (Math.abs(navigationData.closerMotorPower) > 1) {
            Log.w(LOG_TAG, "CLOSER MOTOR BANDED!");
        }
        // Clip motor values:
        navigationData.closerMotorPower = Range.clip(navigationData.closerMotorPower, -1, 1);

        // Farther motor power calulation:
        navigationData.fartherMotorPower = (forwardPower) * (1 + navigationData.rotationPower);
        if (Math.abs(navigationData.closerMotorPower) > 1) {
            Log.w(LOG_TAG, "FARTHER MOTOR BANDED!");
        }
        // Clip motor values:
        navigationData.fartherMotorPower = Range.clip(navigationData.fartherMotorPower, -1, 1);

        // Log Data:
        Log.d(LOG_TAG, String.format("RotPwr: %.3f, ClsrPwr: %.3f, FrthrPwr: %.3f",
                navigationData.rotationPower, navigationData.closerMotorPower,
                navigationData.fartherMotorPower));

        return navigationData;
    }
}
