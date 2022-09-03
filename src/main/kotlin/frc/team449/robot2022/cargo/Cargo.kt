package frc.team449.robot2022.cargo

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.wpilibj.DoubleSolenoid
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.system.motor.WrappedMotor

class Cargo(
  val intakeMotor: WrappedMotor,
  val shooterMotor: WrappedMotor,
  var shooterMotorVoltage: Double,
  var shooterdesiredVelocity: Double,
  val motor: SimpleMotorFeedforward,
  val shooterPIDController: PIDController,
  val deployIntake: DoubleSolenoid,
  val deployHood: DoubleSolenoid
) : SubsystemBase() {

  private var flywheelOn: Boolean = false

  val trueDesiredVelocity: Double = shooterdesiredVelocity

  fun runIntake() {
    intakeMotor.set(CargoConstants.FEEDER_OUTPUT)
    shooterMotor.set(-shooterMotorVoltage)
  }

  fun runIntakeReverse() {
    intakeMotor.set(-CargoConstants.FEEDER_OUTPUT)
    shooterMotor.set(-shooterMotorVoltage)
  }

  fun deployIntake() {
    deployIntake.set(DoubleSolenoid.Value.kReverse)
  }

  fun retractIntake() {
    deployIntake.set(DoubleSolenoid.Value.kForward)
  }

  fun deployHood() {
    deployHood.set(DoubleSolenoid.Value.kReverse)
  }

  fun removeHood() {
    deployHood.set(DoubleSolenoid.Value.kForward)
  }

  override fun periodic() {
    if (flywheelOn) {
      shooterMotor.setVoltage(
        motor.calculate(shooterdesiredVelocity) + shooterPIDController.calculate(
          shooterMotor.velocity,
          shooterdesiredVelocity
        )
      )
    }
  }


  fun shoot() {
    flywheelOn = true
  }

  fun stopShoot() {
    shooterMotor.setVoltage(0.0)
  }
}
