package com.team254.frc2021.controlboard;

public interface IDriveControlBoard {
    double getThrottle();

    double getStrafe();

    double getRotation();

    boolean getDriveLowPower();

    boolean getFieldRelative();

    double getDPad();
}