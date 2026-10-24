package com.stuypulse.robot.subsystems.intake;

import static org.wpilib.units.Units.*;

import com.revrobotics.sim.SparkMaxSim;
import com.stuypulse.robot.subsystems.intake.IntakeConstants.IntakeMotorConfiguration;

public class IntakeIOSparkSim extends IntakeIOSpark {
    private final SparkMaxSim rollerMotorSim;

    public IntakeIOSparkSim() {
        super();

        this.rollerMotorSim = new SparkMaxSim(super.rollerMotor, IntakeMotorConfiguration.CURRENT_CONFIGURATION.getSimMotors(1));
    }

    @Override
    public void updateInputs(IntakeIORollerInputs inputs) {
        inputs.rollerMotorStatorCurrent = Amps.of(rollerMotorSim.getMotorCurrent());
        inputs.rollerMotorSupplyCurrent = Amps.of(rollerMotorSim.getMotorCurrent() * Math.abs(rollerMotorSim.getAppliedOutput()));
        inputs.rollerMotorAngularVelocity = RPM.of(rollerMotorSim.getVelocity());
        inputs.rollerMotorTemperature = Celsius.zero();
        inputs.rollerMotorAppliedVoltage = Volts.of(rollerMotorSim.getBusVoltage());
    }

    @Override
    public void applyOutputs(IntakeIORollerOutputs outputs) {
        switch(outputs.outputMode) {
            case STOP -> rollerMotor.stopMotor();
            case DUTY_CYCLE -> rollerMotor.setThrottle(outputs.targetDutyCycle);
            case VOLTAGE -> rollerMotor.setVoltage(outputs.targetVoltage);
        }
    }
}
