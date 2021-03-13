package com.team254.frc2021;

import com.team254.frc2021.subsystems.SwerveModule.SwerveModuleConstants;

import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.Enumeration;

/**
 * A list of constants used by the rest of the robot code. This includes physics
 * constants as well as constants determined through calibration.
 */
public class Constants {
    public static final double kLooperDt = 0.01;

    public static final int kCANTimeoutMs = 10; // use for important on the fly updates
    public static final int kLongCANTimeoutMs = 100; // use for constructors

    // controls
    public static final int kDriveGamepadPort = 0;
    public static final int kButtonGamepadPort = 2;
    public static final int kMainThrottleJoystickPort = 0;
    public static final int kMainTurnJoystickPort = 1;
    public static final double kJoystickThreshold = 0.5;

    // drivebase
    public static final double kDriveWheelbase = 9.5;
    public static final double kDriveTrackwidth = 9.5;

    // TODO get wheel home positions and check encoder ports are correct
    // swerve modules
    // zero - bezels to left
    public static final SwerveModuleConstants kBackLeftModuleConstants = new SwerveModuleConstants();

    static {
        kBackLeftModuleConstants.kName = "Back Left";
        kBackLeftModuleConstants.kDriveTalonId = 1;
        kBackLeftModuleConstants.kAzimuthTalonId = 2;
        kBackLeftModuleConstants.kAzimuthEncoderPortA = 2;
        kBackLeftModuleConstants.kAzimuthEncoderPortB = 3;
        // kBackLeftModuleConstants.kAzimuthEncoderHomeOffset = 255.5;
        kBackLeftModuleConstants.kInvertDrive = false;
        kBackLeftModuleConstants.kInvertAzimuth = false;
        kBackLeftModuleConstants.kInvertDriveSensorPhase = false;
        kBackLeftModuleConstants.kInvertAzimuthSensorPhase = false;
    }

    public static final SwerveModuleConstants kBackRightModuleConstants = new SwerveModuleConstants();

    static {
        kBackRightModuleConstants.kName = "Back Right";
        kBackRightModuleConstants.kDriveTalonId = 3;
        kBackRightModuleConstants.kAzimuthTalonId = 4;
        kBackRightModuleConstants.kAzimuthEncoderPortA = 0;
        kBackRightModuleConstants.kAzimuthEncoderPortB = 1;
        // kBackRightModuleConstants.kAzimuthEncoderHomeOffset = -34.0;
        kBackRightModuleConstants.kInvertDrive = true;
        kBackRightModuleConstants.kInvertAzimuth = false;
        kBackRightModuleConstants.kInvertDriveSensorPhase = false;
        kBackRightModuleConstants.kInvertAzimuthSensorPhase = false;
    }

    public static final SwerveModuleConstants kFrontRightModuleConstants = new SwerveModuleConstants();

    static {
        kFrontRightModuleConstants.kName = "Front Right";
        kFrontRightModuleConstants.kDriveTalonId = 5;
        kFrontRightModuleConstants.kAzimuthTalonId = 6;
        kFrontRightModuleConstants.kAzimuthEncoderPortA = 4;
        kFrontRightModuleConstants.kAzimuthEncoderPortB = 5;
        // kFrontLeftModuleConstants.kAzimuthKp 
        // kFrontRightModuleConstants.kAzimuthEncoderHomeOffset = -505.75;
        kFrontRightModuleConstants.kInvertDrive = true;
        kFrontRightModuleConstants.kInvertAzimuth = false;
        kFrontRightModuleConstants.kInvertDriveSensorPhase = false;
        kFrontRightModuleConstants.kInvertAzimuthSensorPhase = false;
    }

    public static final SwerveModuleConstants kFrontLeftModuleConstants = new SwerveModuleConstants();

    static {
        kFrontLeftModuleConstants.kName = "Front Left";
        kFrontLeftModuleConstants.kDriveTalonId = 7;
        kFrontLeftModuleConstants.kAzimuthTalonId = 8;
        kFrontLeftModuleConstants.kAzimuthEncoderPortA = 6;
        kFrontLeftModuleConstants.kAzimuthEncoderPortB = 7;
        // kFrontLeftModuleConstants.kAzimuthEncoderHomeOffset = 99.5;
        kFrontLeftModuleConstants.kInvertDrive = false;
        kFrontLeftModuleConstants.kInvertAzimuth = false;
        kFrontLeftModuleConstants.kInvertDriveSensorPhase = false;
        kFrontLeftModuleConstants.kInvertAzimuthSensorPhase = false;
    }

    // TODO tune
    // Swerve Heading Controller
    public static final double kSwerveHeadingControllerErrorTolerance = 0.0; // degrees

    // good for snapping (dpad)
    public static final double kSnapSwerveHeadingKp = 0;
    public static final double kSnapSwerveHeadingKi = 0;
    public static final double kSnapSwerveHeadingKd = 0;

    // good for maintaining heading
    public static final double kMaintainSwerveHeadingKp = 0;
    public static final double kMaintainSwerveHeadingKi = 0;
    public static final double kMaintainSwerveHeadingKd = 0;

    // gyro
    public static final int kPigeonId = 0;



    /**
     * @return the MAC address of the robot
     */
    public static String getMACAddress() {
        try {
            Enumeration<NetworkInterface> nwInterface = NetworkInterface.getNetworkInterfaces();
            StringBuilder ret = new StringBuilder();
            while (nwInterface.hasMoreElements()) {
                NetworkInterface nis = nwInterface.nextElement();
                if (nis != null) {
                    byte[] mac = nis.getHardwareAddress();
                    if (mac != null) {
                        for (int i = 0; i < mac.length; i++) {
                            ret.append(String.format("%02X%s", mac[i], (i < mac.length - 1) ? "-" : ""));
                        }
                        return ret.toString();
                    } else {
                        System.out.println("Address doesn't exist or is not accessible");
                    }
                } else {
                    System.out.println("Network Interface for the specified address is not found.");
                }
            }
        } catch (SocketException | NullPointerException e) {
            e.printStackTrace();
        }

        return "";
    }
}
