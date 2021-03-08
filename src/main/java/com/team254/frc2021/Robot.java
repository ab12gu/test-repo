// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team254.frc2021;

import com.team254.frc2021.controlboard.ControlBoard;
import com.team254.frc2021.controlboard.IControlBoard;
import com.team254.frc2021.loops.Looper;
import com.team254.frc2021.subsystems.Drive;
import com.team254.frc2021.subsystems.RobotStateEstimator;
import com.team254.frc2021.subsystems.Subsystem;
import com.team254.lib.control.SwerveHeadingController;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.util.CrashTracker;
import com.team254.lib.util.LatchedBoolean;
import com.team254.lib.util.TimeDelayedBoolean;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
    private final Looper mEnabledLooper = new Looper();
    private final Looper mDisabledLooper = new Looper();

    private final SubsystemManager mSubsystemManager = SubsystemManager.getInstance();

    private final RobotState mRobotState = RobotState.getInstance();
    private final RobotStateEstimator mRobotStateEstimator = RobotStateEstimator.getInstance();
    private final Drive mDrive = Drive.getInstance();

    private IControlBoard mControlBoard = ControlBoard.getInstance();

    private final SwerveHeadingController mSwerveHeadingController = SwerveHeadingController.getInstance();


    @Override
    public void robotInit() {
        CrashTracker.logRobotInit();

        mSubsystemManager.setSubsystems(new Subsystem[] {mRobotStateEstimator, mDrive},
                mDrive.getSwerveModules());

        mSubsystemManager.registerEnabledLoops(mEnabledLooper);
        mSubsystemManager.registerDisabledLoops(mDisabledLooper);
    }

    @Override
    public void robotPeriodic() {
        mSubsystemManager.outputToSmartDashboard();
    }

    @Override
    public void disabledInit() {
        try {
            CrashTracker.logDisabledInit();
            mEnabledLooper.stop();
            mDisabledLooper.start();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void disabledPeriodic() {
        try {
            mDrive.setHeading(Rotation2d.identity());
            mSwerveHeadingController.setHeadingControllerState(SwerveHeadingController.HeadingControllerState.OFF);
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void testInit() {}

    @Override
    public void testPeriodic() {}

    @Override
    public void autonomousInit() {}

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        try {
            CrashTracker.logTeleopInit();
            mDisabledLooper.stop();
            mEnabledLooper.start();

            // Force true on first iteration of teleop periodic
            shouldChangeAzimuthSetpoint.update(false);
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }


    LatchedBoolean shouldChangeAzimuthSetpoint = new LatchedBoolean();
    TimeDelayedBoolean mShouldMaintainAzimuth = new TimeDelayedBoolean();
    @Override
    public void teleopPeriodic() {
        try {
            manualControl();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    public void manualControl() {
        // drive
        boolean maintainAzimuth = mShouldMaintainAzimuth.update(mControlBoard.getRotation() == 0, 0.2);
        boolean changeAzimuthSetpoint = shouldChangeAzimuthSetpoint.update(maintainAzimuth);

        if (mControlBoard.getDPad() != -1) {
            mSwerveHeadingController.setHeadingControllerState(SwerveHeadingController.HeadingControllerState.SNAP);
            double heading_goal = mControlBoard.getDPad();
            SmartDashboard.putNumber("Heading Goal", heading_goal);
            mSwerveHeadingController.setGoal(heading_goal);
        } else {
            if (!maintainAzimuth) {
                mSwerveHeadingController.setHeadingControllerState(SwerveHeadingController.HeadingControllerState.OFF);
            } else if ((mSwerveHeadingController
                    .getHeadingControllerState() == SwerveHeadingController.HeadingControllerState.SNAP
                    && mSwerveHeadingController.isAtGoal()) || changeAzimuthSetpoint) {
                mSwerveHeadingController
                        .setHeadingControllerState(SwerveHeadingController.HeadingControllerState.MAINTAIN);
                mSwerveHeadingController.setGoal(mDrive.getHeading().getDegrees());
            }
        }

        if (mSwerveHeadingController.getHeadingControllerState() != SwerveHeadingController.HeadingControllerState.OFF) {
            mDrive.setTeleopInputs(mControlBoard.getThrottle(), mControlBoard.getStrafe(), mSwerveHeadingController.update(),
                    mControlBoard.getDriveLowPower(), mControlBoard.getFieldRelative(), true);
        } else {
            mDrive.setTeleopInputs(mControlBoard.getThrottle(), mControlBoard.getStrafe(), mControlBoard.getRotation(),
                    mControlBoard.getDriveLowPower(), mControlBoard.getFieldRelative(), false);
        }
    }
}
