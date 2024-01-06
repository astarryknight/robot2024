package frc.robot.swerve;

import static frc.robot.constants.SwerveConstants.*;

import frc.robot.util.Conversions;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;

/**
 * Implementation of the SwerveModuleIO interface for MK4i Swerve Modules with
 * a NEO (SparkMax) for drive, NEO (SparkMax) for turn, and a Thrifty encoder
 */
public class SwerveModuleIOReal implements SwerveModuleIO {
	private CANSparkMax driveMotor;
	private SparkPIDController driveSparkPidController;
	private RelativeEncoder driveRelativeEncoder;
	private CANSparkMax angleMotor;
	private SparkPIDController angleSparkPidController;
	private SparkAbsoluteEncoder angleAbsoluteEncoder;
	private double angleOffset_rad;
	private SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(driveKs, driveKv);
	// private SimpleMotorFeedforward angleFeedforward = new SimpleMotorFeedforward(angleKs, angleKv);

	/**
	 * @param moduleConstants module config
	 * @param moduleNumber module number used for identification
	 */
	public SwerveModuleIOReal(SwerveModuleConstants moduleConstants, int moduleNumber) {
		this.angleOffset_rad = moduleConstants.angleOffset_rad();

		angleMotor = new CANSparkMax(moduleConstants.angleMotorId(), CANSparkMax.MotorType.kBrushless);
		angleMotor.restoreFactoryDefaults();
		angleSparkPidController = angleMotor.getPIDController();
		angleSparkPidController.setP(angleKp);
		angleSparkPidController.setI(angleKi);
		angleSparkPidController.setD(angleKd);
		angleSparkPidController.setFF(angleKf);
		// angleSparkPidController.setOutputRange(-angleMaxPercentOutput, angleMaxPercentOutput);
		angleMotor.setSmartCurrentLimit(angleCurrentLimit_amp);
		angleMotor.setInverted(angleInverted);
		angleMotor.setIdleMode(angleIdleMode);
		angleMotor.setClosedLoopRampRate(angleRampTime_s);
		angleMotor.enableVoltageCompensation(12);

		angleAbsoluteEncoder = angleMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
		angleAbsoluteEncoder.setPositionConversionFactor(angleEncoderPositionFactor_rad);
		angleAbsoluteEncoder.setVelocityConversionFactor(angleEncoderPositionFactor_rad / 60.0 /* s */);
		angleAbsoluteEncoder.setZeroOffset(Conversions.twoPi);
		angleSparkPidController.setFeedbackDevice(angleAbsoluteEncoder);

		// wrap betwee 0 and 2pi radians
		angleSparkPidController.setPositionPIDWrappingEnabled(true);
		angleSparkPidController.setPositionPIDWrappingMinInput(0);
		angleSparkPidController.setPositionPIDWrappingMaxInput(Conversions.twoPi);

		// turn down frequency as we only log them, not needed for calculations
		// angleMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 40); // percent output
		// angleMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 40); // velocity, current
		// angleMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500); // position
		// angleMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500); // analog sensor
		// angleMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 500); // alternate encoder
		// angleMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 500); // duty cycle position
		// angleMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 500); // duty cycle velocity

		angleMotor.burnFlash();

		driveMotor = new CANSparkMax(moduleConstants.driveMotorId(), CANSparkMax.MotorType.kBrushless);
		driveMotor.restoreFactoryDefaults();
		driveSparkPidController = driveMotor.getPIDController();
		driveSparkPidController.setP(driveKp);
		driveSparkPidController.setI(driveKi);
		driveSparkPidController.setD(driveKd);
		driveSparkPidController.setFF(driveKf);
		// driveSparkPidController.setOutputRange(-driveMaxPercentOutput, driveMaxPercentOutput);
		driveMotor.setSmartCurrentLimit(driveCurrentLimit_amp);
		driveMotor.setInverted(driveInverted);
		driveMotor.setIdleMode(driveIdleMode);
		driveMotor.setClosedLoopRampRate(driveRampTime_s);
		driveMotor.enableVoltageCompensation(12);

		driveRelativeEncoder = driveMotor.getEncoder();
		driveRelativeEncoder.setPositionConversionFactor(driveEncoderPositionFactor_m);
		driveRelativeEncoder.setVelocityConversionFactor(driveEncoderPositionFactor_m / 60.0 /* s */);

		// turn down frequency as we only log them, not needed for calculations
		// driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 40); // percent output
		// driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 40); // velocity, current
		// driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 40); // position
		// driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500); // analog sensor
		// driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 500); // alternate encoder
		// driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 500); // duty cycle position
		// driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 500); // duty cycle velocity

		driveMotor.burnFlash();
	}

	private double getAbsoluteEncoder_rad() {
		return Conversions.angleModulus2pi(angleAbsoluteEncoder.getPosition() - angleOffset_rad);
	}

	private double calculateDriveFeedforward(double velocity) {
		double percentage = driveFeedforward.calculate(velocity);
		return Math.min(percentage, 1.0);
	}

	@Override
	public void updateInputs(SwerveModuleIOInputs inputs) {
		inputs.angleAbsolutePosition_rad = getAbsoluteEncoder_rad();

		inputs.driveDistance_m = driveRelativeEncoder.getPosition();
		inputs.driveVelocity_mps = driveRelativeEncoder.getVelocity();
		inputs.driveAppliedPercentage = driveMotor.getAppliedOutput();
		inputs.driveCurrent_A = driveMotor.getOutputCurrent();
		inputs.driveTemperature_C = driveMotor.getMotorTemperature();

		inputs.angleAbsolutePosition_rad = angleAbsoluteEncoder.getPosition();
		inputs.angleVelocity_radps = angleAbsoluteEncoder.getVelocity();
		inputs.angleAppliedPercentage = angleMotor.getAppliedOutput();
		inputs.angleCurrent_A = angleMotor.getOutputCurrent();
		inputs.angleTemperature_C = angleMotor.getMotorTemperature();
	}

	@Override
	public void setDriveMotorPercentage(double percentage) {
		driveMotor.set(percentage);
	}

	@Override
	public void setDriveVelocity(double velocity) {
		driveSparkPidController.setReference(velocity, ControlType.kVelocity, 0, calculateDriveFeedforward(velocity));
	}

	@Override
	public void setAngleVoltage(double voltage) {
		angleSparkPidController.setReference(voltage, CANSparkMax.ControlType.kVoltage);
	}

	@Override
	public void setAnglePosition(Rotation2d angle) {
		angleSparkPidController.setReference(Conversions.angleModulus2pi(angle.getRadians()), ControlType.kPosition);
	}

	@Override
	public void setDriveBrakeMode(boolean enable) {
		driveMotor.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
	}

	@Override
	public double getCharacterizationVelocity_radps() {
		return Conversions.twoPi * (driveRelativeEncoder.getVelocity() / wheelCircumference_m);
	}

	@Override
	public void setOffset(double offset_rad) {
		angleOffset_rad = offset_rad;
	}
}
