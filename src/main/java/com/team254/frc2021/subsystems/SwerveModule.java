package com.team254.frc2021.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.team254.frc2021.Constants;
import com.team254.frc2021.loops.ILooper;
import com.team254.frc2021.loops.Loop;
import com.team254.lib.drivers.TalonFXFactory;
import com.team254.lib.drivers.TalonUtil;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.util.ReflectingCSVWriter;
import com.team254.lib.util.RollingAverage;
import com.team254.lib.util.SynchronousPIDF;
import com.team254.lib.util.Util;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule extends Subsystem {
    public static class PeriodicIO {
        // INPUTS
        public double drive_encoder_ticks;
        public double azimuth_encoder_ticks; // actual position of module in encoder units, adjusted for home offset
        public int position_ticks;
        public int distance;
        public int velocity_ticks_per_100ms;

        // OUTPUTS
        public double drive_demand;
        public double azimuth_demand; // actual desired demand in encoder units, not adjusted for home offset
    }

    public enum ControlState {
        OPEN_LOOP
    }

    public static class SwerveModuleConstants {
        public String kName = "Name";
        public int kDriveTalonId = -1;
        public int kAzimuthTalonId = -1;
        public int kAzimuthEncoderPort = -1;

        // general azimuth
        public boolean kInvertAzimuth = false;
        public boolean kInvertAzimuthSensorPhase = false;
        public NeutralMode kAzimuthInitNeutralMode = NeutralMode.Brake; // neutral mode could change
        public double kAzimuthTicksPerRadian = 4096.0 / (2 * Math.PI); // for azimuth
        public double kAzimuthEncoderHomeOffset = 0;

        // azimuth motion TODO tune
        public double kAzimuthKp = 0.00103;
        public double kAzimuthKi = 0.0;
        public double kAzimuthKd = 0.00002;
        public double kAzimuthKf = 0;
        public int kAzimuthIZone = 0;
        public int kAzimuthCruiseVelocity = 0;
        public int kAzimuthAcceleration = 0; // 12 * kAzimuthCruiseVelocity
        public int kAzimuthClosedLoopAllowableError = 20;

        // azimuth current/voltage TODO verify
        public int kAzimuthContinuousCurrentLimit = 30; // amps
        public int kAzimuthPeakCurrentLimit = 60; // amps
        public int kAzimuthPeakCurrentDuration = 200; // ms
        public boolean kAzimuthEnableCurrentLimit = true;
        public double kAzimuthMaxVoltage = 10.0; // volts
        public int kAzimuthVoltageMeasurementFilter = 8; // # of samples in rolling average

        // azimuth measurement
//        public int kAzimuthStatusFrame2UpdateRate = 10; // feedback for selected sensor, ms
//        public int kAzimuthStatusFrame10UpdateRate = 10; // motion magic, ms
//        public VelocityMeasPeriod kAzimuthVPMeasurementPeriod = VelocityMeasPeriod.Period_100Ms; // dt for velocity measurements, ms
        public int kAzimuthPositionMeasurementWindow = 1; // # of samples in rolling average

        // general drive
        public boolean kInvertDrive = true;
        public boolean kInvertDriveSensorPhase = false;
        public NeutralMode kDriveInitNeutralMode = NeutralMode.Brake; // neutral mode could change
        public double kWheelDiameter = 4.0; // Probably should tune for each individual wheel maybe
        public double kDriveTicksPerUnitDistance = (1.0 / 2048.0) / (34.0 / 14.0 * 16.0 / 28.0 * 60.0 / 15.0)
                * (Math.PI * kWheelDiameter);   // TODO double check
        public double kDriveDeadband = 0.01;

        // drive current/voltage TODO verify
        public int kDriveContinuousCurrentLimit = 30; // amps
        public int kDrivePeakCurrentLimit = 50; // amps
        public int kDrivePeakCurrentDuration = 200; // ms
        public boolean kDriveEnableCurrentLimit = true;
        public double kDriveMaxVoltage = 10.0; // volts
        public int kDriveVoltageMeasurementFilter = 8; // # of samples in rolling average

        // drive measurement
        public int kDriveStatusFrame2UpdateRate = 15; // feedback for selected sensor, ms
        public int kDriveStatusFrame10UpdateRate = 200; // motion magic, ms
        public VelocityMeasPeriod kDriveVelocityMeasurementPeriod = VelocityMeasPeriod.Period_100Ms; // dt for velocity measurements, ms
        public int kDriveVelocityMeasurementWindow = 64; // # of samples in rolling average
    }

    private PeriodicIO mPeriodicIO = new PeriodicIO();
    private ControlState mControlState = ControlState.OPEN_LOOP;
    private final SwerveModuleConstants mConstants;

    private TalonFX mDriveTalon;
    private TalonFX mAzimuthTalon;

    private DutyCycleEncoder mAzimuthEncoder;
    private SynchronousPIDF mAzimuthPIDF;

    private ReflectingCSVWriter<PeriodicIO> mCSVWriter = null;

    private RollingAverage mAzimuthAverage;

    public SwerveModule(SwerveModuleConstants constants) {
        mConstants = constants;

        mDriveTalon = TalonFXFactory.createDefaultTalon(mConstants.kDriveTalonId);
        mAzimuthTalon = TalonFXFactory.createDefaultTalon(mConstants.kAzimuthTalonId);

        mAzimuthEncoder = new DutyCycleEncoder(new DutyCycle(new DigitalInput(mConstants.kAzimuthEncoderPort)));

        // config sensors
        TalonUtil.checkError(
                mDriveTalon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0,
                        Constants.kLongCANTimeoutMs),
                "Error in " + mConstants.kName + " Module: Unable to config drive encoder");

        // config azimuth motion
//        TalonUtil.checkError(
//                mAzimuthTalon.configMotionCruiseVelocity(mConstants.kAzimuthCruiseVelocity,
//                        Constants.kLongCANTimeoutMs),
//                "Error in " + mConstants.kName + "Module: Unable to config azimuth cruise vel");
//        TalonUtil.checkError(
//                mAzimuthTalon.configMotionAcceleration(mConstants.kAzimuthAcceleration, Constants.kLongCANTimeoutMs),
//                "Error in " + mConstants.kName + "Module: Unable to config azimuth max acc");

        // config azimuth current/voltage settings
        TalonUtil.checkError(
                mAzimuthTalon.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(
                        mConstants.kAzimuthEnableCurrentLimit,
                        mConstants.kAzimuthContinuousCurrentLimit,
                        mConstants.kAzimuthPeakCurrentLimit,
                        mConstants.kAzimuthPeakCurrentDuration)),
                mConstants.kName + ": Could not set azi supply current limit.");
        TalonUtil.checkError(
                mAzimuthTalon.configVoltageMeasurementFilter(mConstants.kAzimuthVoltageMeasurementFilter,
                        Constants.kLongCANTimeoutMs),
                "Error in " + mConstants.kName + "Module: Unable to config azimuth voltage measurement filter");
        TalonUtil.checkError(
                mAzimuthTalon.configVoltageCompSaturation(mConstants.kAzimuthMaxVoltage, Constants.kLongCANTimeoutMs),
                "Error in " + mConstants.kName + "Module: Unable to config azimuth voltage comp saturation");
        mAzimuthTalon.enableVoltageCompensation(true);

        // config azimuth measurement settings
        mAzimuthPIDF = new SynchronousPIDF(mConstants.kAzimuthKp, mConstants.kAzimuthKi, mConstants.kAzimuthKd, mConstants.kAzimuthKf);
        mAzimuthPIDF.setIZone(mConstants.kAzimuthIZone);
        mAzimuthPIDF.setDeadband(mConstants.kAzimuthClosedLoopAllowableError);
//        TalonUtil.checkError(
//                mAzimuthTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0,
//                        mConstants.kAzimuthStatusFrame2UpdateRate, Constants.kLongCANTimeoutMs),
//                "Error in " + mConstants.kName + "Module: Unable to config azimuth status frame 2 period");
//        TalonUtil.checkError(
//                mAzimuthTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic,
//                        mConstants.kAzimuthStatusFrame10UpdateRate, Constants.kLongCANTimeoutMs),
//                "Error in " + mConstants.kName + "Module: Unable to config azimuth status frame 10 period");
//        TalonUtil.checkError(
//                mAzimuthTalon.configVelocityMeasurementPeriod(mConstants.kAzimuthVelocityMeasurementPeriod,
//                        Constants.kLongCANTimeoutMs),
//                "Error in " + mConstants.kName + "Module: Unable to config azimuth velocity measurement period");
//        TalonUtil.checkError(
//                mAzimuthTalon.configVelocityMeasurementWindow(mConstants.kAzimuthVelocityMeasurementWindow,
//                        Constants.kLongCANTimeoutMs),
//                "Error in " + mConstants.kName + "Module: Unable to config azimuth velocity measurement window");

        // config drive current/voltage settings
        TalonUtil.checkError(
                mAzimuthTalon.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(
                        mConstants.kDriveEnableCurrentLimit,
                        mConstants.kDriveContinuousCurrentLimit,
                        mConstants.kDrivePeakCurrentLimit,
                        mConstants.kDrivePeakCurrentDuration)),
                mConstants.kName + ": Could not set azi supply current limit.");
        TalonUtil.checkError(
                mDriveTalon.configVoltageMeasurementFilter(mConstants.kDriveVoltageMeasurementFilter,
                        Constants.kLongCANTimeoutMs),
                "Error in " + mConstants.kName + "Module: Unable to config drive voltage measurement filter");
        TalonUtil.checkError(
                mDriveTalon.configVoltageCompSaturation(mConstants.kDriveMaxVoltage, Constants.kLongCANTimeoutMs),
                "Error in " + mConstants.kName + "Module: Unable to config drive voltage comp saturation");
        mDriveTalon.enableVoltageCompensation(true);

        // config drive measurement settings
        TalonUtil.checkError(
                mDriveTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0,
                        mConstants.kDriveStatusFrame2UpdateRate, Constants.kLongCANTimeoutMs),
                "Error in " + mConstants.kName + "Module: Unable to config drive status frame 2 period");
        TalonUtil.checkError(
                mDriveTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic,
                        mConstants.kDriveStatusFrame10UpdateRate, Constants.kLongCANTimeoutMs),
                "Error in " + mConstants.kName + "Module: Unable to config drive status frame 10 period");
        TalonUtil.checkError(
                mDriveTalon.configVelocityMeasurementPeriod(mConstants.kDriveVelocityMeasurementPeriod,
                        Constants.kLongCANTimeoutMs),
                "Error in " + mConstants.kName + "Module: Unable to config drive velocity measurement period");
        TalonUtil.checkError(
                mDriveTalon.configVelocityMeasurementWindow(mConstants.kDriveVelocityMeasurementWindow,
                        Constants.kLongCANTimeoutMs),
                "Error in " + mConstants.kName + "Module: Unable to config drive velocity measurement window");

        // config general drive settings
        mDriveTalon.setInverted(mConstants.kInvertDrive);
        mDriveTalon.setSensorPhase(mConstants.kInvertDriveSensorPhase);
        mDriveTalon.setNeutralMode(mConstants.kDriveInitNeutralMode);
        TalonUtil.checkError(mDriveTalon.configForwardSoftLimitEnable(false, Constants.kLongCANTimeoutMs),
                "Error in " + mConstants.kName + "Module: Unable to disable drive forward soft limit");
        TalonUtil.checkError(mDriveTalon.configReverseSoftLimitEnable(false, Constants.kLongCANTimeoutMs),
                "Error in " + mConstants.kName + "Module: Unable to disable drive reverse soft limit");

        // config general azimuth settings
        mAzimuthTalon.setInverted(mConstants.kInvertAzimuth);
        mAzimuthEncoder.setDistancePerRotation(mConstants.kInvertAzimuthSensorPhase ? -4096 : 4096);
        mAzimuthAverage = new RollingAverage(mConstants.kAzimuthPositionMeasurementWindow);

        mAzimuthTalon.setNeutralMode(mConstants.kAzimuthInitNeutralMode);

        zeroSensors();
    }

    public synchronized void setOpenLoop(double speed, Rotation2d azimuth) {
        if (mControlState != ControlState.OPEN_LOOP) {
            mControlState = ControlState.OPEN_LOOP;
        }

        Rotation2d current = getAngle();
        double raw_error = azimuth.getRadians() - current.getRadians();
        // raw_error = raw_error - Math.floor(raw_error/(2*Math.PI)) * 2*Math.PI; // bound error between 0 and 2pi
       
        if (Math.abs(raw_error) > Math.PI) {
            raw_error -= (Math.PI * 2 * Math.signum(raw_error));
        }

        // error is -180 to 180
        // is wheel reversible logic
        if (Math.abs(raw_error) > Math.PI / 2.0) {
            speed *= -1;
            raw_error -= Math.PI * Math.signum(raw_error);
        }


        double final_setpoint = current.getRadians() + raw_error;
        // double adjusted_speed = speed * Math.abs(Math.cos(raw_error));

        mPeriodicIO.drive_demand = speed;
        mPeriodicIO.azimuth_demand = radiansToEncoderUnits(final_setpoint);
    }

    

    @Override
    public void readPeriodicInputs() {
        mPeriodicIO.drive_encoder_ticks = mDriveTalon.getSelectedSensorPosition(0);
        mPeriodicIO.distance = (int) encoderUnitsToDistance(mPeriodicIO.drive_encoder_ticks);
        mPeriodicIO.velocity_ticks_per_100ms = (int) mDriveTalon.getSelectedSensorVelocity(0);
        mAzimuthAverage.addObservation(mAzimuthEncoder.getDistance());
        SmartDashboard.putNumber(mConstants.kName + " Module average", mAzimuthAverage.getValue());
        mPeriodicIO.azimuth_encoder_ticks = mAzimuthAverage.getValue();

        if (mCSVWriter != null) {
            mCSVWriter.add(mPeriodicIO);
        }
    }

    @Override
    public void writePeriodicOutputs() {
        if (mControlState == ControlState.OPEN_LOOP) {
            if (Util.epsilonEquals(mPeriodicIO.drive_demand, 0.0, mConstants.kDriveDeadband)) { // don't move if
                // throttle is 0
                stop();
            } else {
                mAzimuthPIDF.setSetpoint(mPeriodicIO.azimuth_demand + mConstants.kAzimuthEncoderHomeOffset);
                double x = mAzimuthPIDF.calculate(mPeriodicIO.azimuth_encoder_ticks);
                SmartDashboard.putNumber(mConstants.kName + " pidf value: ", x);
                mAzimuthTalon.set(ControlMode.PercentOutput, x);

                mDriveTalon.set(ControlMode.PercentOutput, mPeriodicIO.drive_demand);
            }
        }
    }

    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        mEnabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                synchronized (SwerveModule.this) {
                    stop();
                    startLogging();
                }
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (SwerveModule.this) {
                    switch (mControlState) {
                        case OPEN_LOOP:
                            break;
                        default:
                            System.out.println("Unexpected control state: " + mControlState);
                            break;
                    }
                }
            }

            @Override
            public void onStop(double timestamp) {
                stop();
                stopLogging();
            }
        });
    }

    @Override
    public void zeroSensors() {
        mDriveTalon.setSelectedSensorPosition(0, 0, Constants.kCANTimeoutMs);
        /* Azimuth Talon should be in absolute mode */
    }

    @Override
    public void stop() {
        mDriveTalon.set(ControlMode.PercentOutput, 0.0);
        mAzimuthTalon.set(ControlMode.PercentOutput, 0.0);
    }

    @Override
    public boolean checkSystem() {
        return true;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber(mConstants.kName + " Module: Get Raw Value", mAzimuthEncoder.getDistance());
        SmartDashboard.putNumber(mConstants.kName + " Module: Encoding Scale", mAzimuthEncoder.getDistancePerRotation());
        SmartDashboard.putNumber(mConstants.kName + " Module: Module Angle", getAngle().getDegrees());
        SmartDashboard.putNumber(mConstants.kName + " Module: Linear Velocity", getLinearVelocity());
        SmartDashboard.putNumber(mConstants.kName + " Module: Distance Driven", mPeriodicIO.distance);
        SmartDashboard.putNumber(mConstants.kName + " Module: Azimuth Ticks", mPeriodicIO.azimuth_encoder_ticks);

        SmartDashboard.putNumber(mConstants.kName + " Module: Drive Demand", mPeriodicIO.drive_demand);
        SmartDashboard.putNumber(mConstants.kName + " Module: Azimuth Tick Demand", mPeriodicIO.azimuth_demand);
        SmartDashboard.putNumber(mConstants.kName + " Module: Azimuth Angle Demand",
                Math.toDegrees(Util.bound0To2PIRadians(encoderUnitsToRadians(mPeriodicIO.azimuth_demand))));
        SmartDashboard.putNumber(mConstants.kName + " Module: Azimuth Degree Error",
                Math.toDegrees(encoderUnitsToRadians(mPeriodicIO.azimuth_demand - getAngleEncoderUnits())));
        SmartDashboard.putNumber(mConstants.kName + " Module: Azimuth Tick Error",
                mPeriodicIO.azimuth_demand - getAngleEncoderUnits());

        SmartDashboard.putNumber(mConstants.kName + " Module: Actual Drive Percent Output", getDrivePercentOutput());
        SmartDashboard.putBoolean(mConstants.kName + " Module: Drive Demand Equals Actual",
                Util.epsilonEquals(mPeriodicIO.drive_demand, getDrivePercentOutput()));

        SmartDashboard.putBoolean(mConstants.kName + " Module: Azimuth At Target", isAzimuthAtTarget());
        SmartDashboard.putNumber(mConstants.kName + " Module: Current", mAzimuthTalon.getStatorCurrent());
        SmartDashboard.putNumber(mConstants.kName + " Module: Voltage", mAzimuthTalon.getMotorOutputVoltage());

        if (mCSVWriter != null) {
            mCSVWriter.write();
        }
    }

    /**
     * @param ticks ticks
     */
    public synchronized double encoderUnitsToRadians(double ticks) {
        return ticks / mConstants.kAzimuthTicksPerRadian;
    }

    /**
     * @return azimuth ticks
     */
    public synchronized double radiansToEncoderUnits(double radians) {
        return radians * mConstants.kAzimuthTicksPerRadian;
    }

    /**
     * @param ticks drive encoder ticks
     */
    public synchronized double encoderUnitsToDistance(double ticks) {
        return ticks * mConstants.kDriveTicksPerUnitDistance;
    }

    /**
     * @return drive ticks
     */
    public synchronized double distanceToEncoderUnits(double distance) {
        return distance / mConstants.kDriveTicksPerUnitDistance;
    }

    public synchronized double getAngleEncoderUnits() {
        return mPeriodicIO.azimuth_encoder_ticks - mConstants.kAzimuthEncoderHomeOffset;
    }

    public synchronized Rotation2d getAngle() {
        return Rotation2d.fromRadians(encoderUnitsToRadians(getAngleEncoderUnits()));
    }

    public synchronized double getRawAngle() {
        return encoderUnitsToRadians(getAngleEncoderUnits());
    }

    public synchronized double getUnwrappedAngleDegrees() {
        return Math.toDegrees(encoderUnitsToRadians(getAngleEncoderUnits()));
    }

    public synchronized double getRawLinearVelocity() {
        return mPeriodicIO.velocity_ticks_per_100ms * 10;
    }

    public synchronized double getLinearVelocity() {
        return encoderUnitsToDistance(getRawLinearVelocity());
    }

    public synchronized void setDriveBrakeMode(boolean brake_mode) {
        mDriveTalon.setNeutralMode(brake_mode ? NeutralMode.Brake : NeutralMode.Coast);
    }

    public synchronized void setAzimuthBrakeMode(boolean brake_mode) {
        mAzimuthTalon.setNeutralMode(brake_mode ? NeutralMode.Brake : NeutralMode.Coast);
    }

    public synchronized double getDrivePercentOutput() {
        return mDriveTalon.getMotorOutputPercent();
    }

    public synchronized boolean isAzimuthAtTarget() {
        return Util.epsilonEquals(mPeriodicIO.azimuth_demand, getAngleEncoderUnits(),
                mConstants.kAzimuthClosedLoopAllowableError);
    }

    public synchronized void startLogging() {
        if (mCSVWriter == null) {
            mCSVWriter = new ReflectingCSVWriter<>("/home/lvuser/" + mConstants.kName + "-MODULE-LOGS.csv",
                    PeriodicIO.class);
        }
    }

    public synchronized void stopLogging() {
        if (mCSVWriter != null) {
            mCSVWriter.flush();
            mCSVWriter = null;
        }
    }
}