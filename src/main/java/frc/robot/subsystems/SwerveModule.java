/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */

public class SwerveModule extends Subsystem {

  public WPI_TalonSRX angleMotor;
  public CANSparkMax speedMotor;

  public final double[] pidConstants;
  public double error;
  public double output;
  public PIDController pidController;
  public AnalogInput encoder;
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public SwerveModule(int swerveModIndex, boolean angleMotorInverted){
    speedMotor = new CANSparkMax(swerveModIndex + 11, MotorType.kBrushless);
    angleMotor = new WPI_TalonSRX(swerveModIndex);

    angleMotor.setInverted(angleMotorInverted);
		encoder = new AnalogInput(swerveModIndex);

		pidConstants = RobotMap.SWERVE_PID_CONSTANTS[swerveModIndex];
		pidController = new PIDController(pidConstants[0], pidConstants[1], pidConstants[2], 
				encoder, angleMotor, RobotMap.SWERVE_LOOP_TIME);

		pidController.setInputRange(0.0, RobotMap.SWERVE_ENC_CIRC);
		pidController.setOutputRange(-1.0, 1.0);
		// pidController.setContinuous(true);
		pidController.setAbsoluteTolerance(RobotMap.SWERVE_PID_TOLERANCE);
		pidController.enable();

		angleMotor.configContinuousCurrentLimit(RobotMap.SWERVE_MAX_CURRENT, 0);
		angleMotor.configPeakCurrentLimit(RobotMap.SWERVE_MAX_CURRENT, 0);
		angleMotor.configPeakCurrentDuration(RobotMap.SWERVE_CURRENT_DUR, 0);
		angleMotor.enableCurrentLimit(true);
    
  }

  public void moveModule (double speed, double angle) {
		pidController.setSetpoint(angle);
		// angleMotor.set(angle);

		speedMotor.set (speed);
		error = pidController.getError();
		output = pidController.get();
	}

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
