package frc.robot.subsystems.swerve;

import static frc.robot.constants.Constants.tuningMode;
import static frc.robot.constants.SwerveConstants.*;

import frc.robot.util.AutoSetterTunableNumber;
import frc.robot.util.Conversions;
import frc.robot.util.SparkUtil;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

/**
 * Implementation of the SwerveModuleIO interface for MK4i Swerve Modules with
 * a NEO (SparkMax) for drive, NEO (SparkMax) for turn, and a Thrifty encoder
 */
public class SwerveModuleIOReal implements SwerveModuleIO {
	private CANSparkMax driveMotor;
	private SparkPIDController drivePidController;
	private RelativeEncoder driveRelativeEncoder;
	private SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(driveKs, driveKv, driveKa);

	private CANSparkMax angleMotor;
	private SparkPIDController angleMotorPidController;
	private AnalogInput angleAbsoluteEncoder;
	private RelativeEncoder angleRelativeEncoder;
	private double angleOffset_rad;
	// private SimpleMotorFeedforward angleFeedforward = new SimpleMotorFeedforward(angleKs, angleKv, angleKa);

	/**
	 * @param moduleConstants module config
	 * @param moduleNumber module number used for identification
	 */
	public SwerveModuleIOReal(SwerveModuleConstants moduleConstants, int moduleNumber) {
		this.angleOffset_rad = moduleConstants.angleOffset_rad();

		if (tuningMode) {
			final String key = "SwerveModuleIO" + moduleNumber + "/";
			new AutoSetterTunableNumber(key + "driveKp", driveKp, (double value) -> drivePidController.setP(value));
			new AutoSetterTunableNumber(key + "driveKd", driveKd, (double value) -> drivePidController.setD(value));
			new AutoSetterTunableNumber(key + "angleKp", angleKp, (double value) -> angleMotorPidController.setP(value));
			new AutoSetterTunableNumber(key + "angleKd", angleKd, (double value) -> angleMotorPidController.setD(value));
		}

		driveMotor = new CANSparkMax(moduleConstants.driveMotorId(), CANSparkMax.MotorType.kBrushless);
		driveMotor.restoreFactoryDefaults();
		drivePidController = driveMotor.getPIDController();
		drivePidController.setP(driveKp);
		drivePidController.setI(driveKi);
		drivePidController.setD(driveKd);
		drivePidController.setFF(driveKf);
		// driveSparkPidController.setOutputRange(-driveMaxPercentOutput, driveMaxPercentOutput);
		driveMotor.setSmartCurrentLimit(driveCurrentLimit_amp);
		driveMotor.setInverted(driveInverted);
		driveMotor.setIdleMode(IdleMode.kBrake);
		driveMotor.setClosedLoopRampRate(driveRampTime_s);
		driveMotor.enableVoltageCompensation(12);

		driveRelativeEncoder = driveMotor.getEncoder();
		driveRelativeEncoder.setPositionConversionFactor(wheelCircumference_m / driveGearRatio); // m
		driveRelativeEncoder.setVelocityConversionFactor(wheelCircumference_m / driveGearRatio / 60.0); // mps

		SparkUtil.setPeriodicFrames(driveMotor, false, true, true, false, false, false, false);

		driveMotor.burnFlash();

		angleMotor = new CANSparkMax(moduleConstants.angleMotorId(), CANSparkMax.MotorType.kBrushless);
		angleMotor.restoreFactoryDefaults();
		angleMotorPidController = angleMotor.getPIDController();
		angleMotorPidController.setP(angleKp);
		angleMotorPidController.setI(angleKi);
		angleMotorPidController.setD(angleKd);
		angleMotorPidController.setFF(angleKf);
		// angleSparkPidController.setOutputRange(-angleMaxPercentOutput, angleMaxPercentOutput);
		angleMotor.setSmartCurrentLimit(angleCurrentLimit_amp);
		angleMotor.setInverted(angleInverted);
		angleMotor.setIdleMode(IdleMode.kCoast);
		// angleMotor.setClosedLoopRampRate(angleRampTime_s);
		angleMotor.enableVoltageCompensation(12);

		angleAbsoluteEncoder = new AnalogInput(moduleNumber);

		// angleAbsoluteEncoder = angleMotor.getAnalog(SparkAnalogSensor.Mode.kAbsolute);
		// angleAbsoluteEncoder.setPositionConversionFactor(Conversions.truncate(Conversions.twoPi / 3.3 /* V */, 3)); //
		// rad
		// // absolute encoder velocity reading is meaningless (jumps around), using relative encoder's instead
		// // angleAbsoluteEncoder.setVelocityConversionFactor(Conversions.twoPi / 3.3 /* V */); // radps
		// anglePidController.setFeedbackDevice(angleAbsoluteEncoder);

		angleRelativeEncoder = angleMotor.getEncoder();
		angleRelativeEncoder.setPositionConversionFactor(Conversions.twoPi / angleGearRatio); // rad
		angleRelativeEncoder.setVelocityConversionFactor(Conversions.twoPi / angleGearRatio / 60.0); // radps

		// on first cycle, reset relative encoder to absolute
		angleRelativeEncoder.setPosition(Conversions
			.angleModulus2pi(
				Conversions.truncate(angleAbsoluteEncoder.getVoltage() / RobotController.getVoltage5V(), 3) * Conversions.twoPi
					- angleOffset_rad));

		// wrap betwee 0 and 2pi radians
		angleMotorPidController.setPositionPIDWrappingEnabled(true);
		angleMotorPidController.setPositionPIDWrappingMinInput(0);
		angleMotorPidController.setPositionPIDWrappingMaxInput(Conversions.twoPi);

		SparkUtil.setPeriodicFrames(angleMotor, false, true, true, true, false, false, false);

		angleMotor.burnFlash();
	}

	@Override
	public void updateInputs(SwerveModuleIOInputs inputs) {
		inputs.driveDistance_m = SparkUtil.cleanValue(inputs.driveDistance_m, driveRelativeEncoder.getPosition());
		inputs.driveVelocity_mps = SparkUtil.cleanValue(inputs.driveVelocity_mps, driveRelativeEncoder.getVelocity());
		inputs.driveVoltage_V = driveMotor.getBusVoltage() * driveMotor.getAppliedOutput();
		inputs.driveCurrent_A = driveMotor.getOutputCurrent();
		inputs.driveTemperature_C = driveMotor.getMotorTemperature();

		inputs.anglePosition_rad = SparkUtil.cleanValue(inputs.anglePosition_rad, angleRelativeEncoder.getPosition());
		// inputs.angleAbsolutePosition_rad = SparkUtil.cleanValue(inputs.angleAbsolutePosition_rad,
		// // ensure offset is properly wrapped
		// Conversions.angleModulus2pi(angleAbsoluteEncoder.getPosition() - angleOffset_rad));
		// inputs.angleVelocity_radps = angleAbsoluteEncoder.getVelocity(); // gives garbage data
		inputs.angleAbsolutePosition_rad = Conversions
			.angleModulus2pi(((angleAbsoluteEncoder.getVoltage() / RobotController.getVoltage5V()) * 2.0
				* Math.PI) - angleOffset_rad);
		inputs.angleVelocity_radps = SparkUtil.cleanValue(inputs.angleVelocity_radps, angleRelativeEncoder.getVelocity());
		inputs.angleVoltage_V = angleMotor.getBusVoltage() * angleMotor.getAppliedOutput();
		inputs.angleCurrent_A = angleMotor.getOutputCurrent();
		inputs.angleTemperature_C = angleMotor.getMotorTemperature();
	}

	@Override
	public void setDriveVoltage(double voltage) {
		driveMotor.setVoltage(voltage);
	}

	@Override
	public void setDriveVelocity(double velocity) {
		drivePidController.setReference(velocity, ControlType.kVelocity, 0, driveFeedforward.calculate(velocity));
	}

	@Override
	public void setAngleVoltage(double voltage) {
		angleMotorPidController.setReference(voltage, ControlType.kVoltage);
	}

	@Override
	public void setAnglePosition(Rotation2d angle) {
		final double targetAngle_rad = Conversions.angleModulus2pi(angle.getRadians() + angleOffset_rad);
		// anglePidController.setReference(targetAngle_rad, ControlType.kPosition);
		// 0, angleFeedforward.calculate(targetAngle_rad));
	}

	@Override
	public void setDriveBrakeMode(boolean enable) {
		driveMotor.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
	}

	@Override
	public void setOffset(double offset_rad) {
		angleOffset_rad = offset_rad;
	}

	@Override
	public double getDriveCharacterizationVelocity_radps() {
		return Conversions.twoPi * (driveRelativeEncoder.getVelocity() / wheelCircumference_m); // undo unit conversion
	}
}
