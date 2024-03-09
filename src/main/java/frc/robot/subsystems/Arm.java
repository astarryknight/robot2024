package frc.robot.subsystems;

import static frc.robot.constants.ArmConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import lombok.Getter;

public class Arm extends SubsystemBase {
	private CANSparkMax motor = new CANSparkMax(motorId, MotorType.kBrushless);
	private SparkPIDController pidController = motor.getPIDController();
	private RelativeEncoder encoder = motor.getEncoder();

	private ArmFeedforward feedforwardController = new ArmFeedforward(kS, kG, kV, kA);

	@Getter
	@AutoLogOutput
	private double targetPosition_rad = 0;

	public Arm() {
		motor.setIdleMode(IdleMode.kBrake);

		pidController.setP(kP);
		pidController.setD(kD);
		pidController.setI(kI);

		motor.setSmartCurrentLimit(50);
		motor.setSecondaryCurrentLimit(65);

		encoder.setPositionConversionFactor((1 / gearRatio) * 2 * Math.PI); // motor revolutions * arm rev / motor rev * arm
		// radians / arm rev = arm radians
		encoder.setVelocityConversionFactor((1 / gearRatio) * (2 * Math.PI) * (1 / 60)); // RPM * arm rev / motor rev *
																																											// radians /
		// revolution * min / sec = rad/s
	}

	@Override
	public void periodic() {
		updateInputs(inputs);
		Logger.processInputs(getName(), inputs);
	}

	@AutoLog
	public static class ArmIOInputs {
		public double current_A = 0.0;
		public double voltage_V = 0.0;
		public double temperature_C = 0.0;
		public double position_rad = 0.0;
		public double velocity_radps = 0.0;
	}

	public final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

	private void updateInputs(ArmIOInputs inputs) {
		inputs.current_A = motor.getOutputCurrent();
		inputs.voltage_V = motor.getBusVoltage();
		inputs.temperature_C = motor.getMotorTemperature();
		inputs.position_rad = encoder.getPosition();
		inputs.velocity_radps = encoder.getVelocity();
	}


	/* NEW CODE */
	/** Enum to store and access distance and arm rotation values from one constructor	 */
	enum testedValue {
		private double distanceFromTarget; //total distance (hypotenuse from x and z directions) from the center of the robot to the target, in meters
		private double armPosition_deg; //position of the arm motor(s), in degrees (how the data was taken)

		/** enum constructor to inizialize values */
		public testedValue(double distanceFromTarget, double armPosition_rad){ //main constructor
			this.distanceFromTarget = distanceFromTarget;
			this.armPosition_rad = armPosition_rad;
		}
		//Use @Getter instead?
		public double getDistance(){
			return distanceFromTarget;
		}
		public double getDegreesRad(){
			return Math.toRadians(armPosition_rad);
		}
	}

	/** list of tested values sorted in ascending order of distance */
	private testedValue[] testedValues = [new testedValue(0,-1), new testedValue(6,14)]

	public double interpolateYaw(double distanceFromTarget){
		/** values that bound the distance (ie. low<distanceFromTarget<high) */
		private double closestVal_high;
		private double closestVal_low;

		for(int i=0; i<testedValues.length; i++){
			if(distanceFromTarget<testedValues[i].getDistance()){
				closestVal_high=testedValues[i];
				closestVal_low=testedValues[i-1];
				break;
			} 
			if(distanceFromTarget==testedValues[i].getDistance()){
				return testedValues[i].getDegreesRad();
			}
		}

		return closestVal_low.getDegreesRad()+(((distanceFromTarget-closestVal_low.getDistance())/(closestVal_high.getDistance()-closestVal_low.getDistance()))*(closestVal_high.getDegreesRad()-closestVal_low.getDegreesRad()));
	}
	/** END NEW CODE */


	public void setPosition(double position_rad) {
		targetPosition_rad = position_rad;
		pidController.setReference(targetPosition_rad, ControlType.kPosition, 0,
			feedforwardController.calculate(targetPosition_rad, 0));
	}

	public void stopArm() {
		pidController.setReference(0, ControlType.kDutyCycle, 0, feedforwardController.calculate(inputs.position_rad, 0));
		// Set
	}

	public boolean closeEnough() {
		if (MathUtil.isNear(targetPosition_rad, inputs.position_rad, closeVar)) {
			return true;
		} else {
			return false;
		}
	}
}
