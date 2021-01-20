package com.team254.lib.drivers;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.team254.frc2021.subsystems.Subsystem;

import java.util.ArrayList;

public class TalonFXChecker extends MotorChecker<BaseTalon> {
    private static class StoredTalonFXConfiguration {
        public ControlMode mMode;
        public double mSetValue;
    }

    protected ArrayList<StoredTalonFXConfiguration> mStoredConfigurations = new ArrayList<>();

    public static boolean checkMotors(Subsystem subsystem,
                                      ArrayList<MotorConfig<BaseTalon>> motorsToCheck,
                                      CheckerConfig checkerConfig) {
        TalonFXChecker checker = new TalonFXChecker();
        return checker.checkMotorsImpl(subsystem, motorsToCheck, checkerConfig);
    }

    @Override
    protected void storeConfiguration() {
        // record previous configuration for all talons
        for (MotorConfig<BaseTalon> config : mMotorsToCheck) {
            LazyTalonFX talon = (LazyTalonFX) config.mMotor;

            StoredTalonFXConfiguration configuration = new StoredTalonFXConfiguration();
            configuration.mMode = talon.getControlMode();
            configuration.mSetValue = talon.getLastSet();

            mStoredConfigurations.add(configuration);
        }
    }

    @Override
    protected void restoreConfiguration() {
        for (int i = 0; i < mMotorsToCheck.size(); ++i) {
            mMotorsToCheck.get(i).mMotor.set(mStoredConfigurations.get(i).mMode,
                    mStoredConfigurations.get(i).mSetValue);
        }
    }

    @Override
    protected void setMotorOutput(BaseTalon motor, double output) {
        motor.set(ControlMode.PercentOutput, output);
    }

    @Override
    protected double getMotorCurrent(BaseTalon motor) {
        return motor.getSupplyCurrent();
    }
}
