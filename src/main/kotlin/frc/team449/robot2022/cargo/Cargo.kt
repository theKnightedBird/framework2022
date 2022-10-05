package frc.team449.robot2022.cargo

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.wpilibj.DoubleSolenoid
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.system.motor.WrappedMotor

class Cargo(
  val intakeMotor: WrappedMotor,
  val spitterMotor: WrappedMotor,
  var spitterMotorVoltage: Double,
  var spitterDesiredVelocity: Double,
  // val motor: SimpleMotorFeedforward,
  // val shooterPIDController: PIDController,
  val deployIntake: DoubleSolenoid,
  val deployHood: DoubleSolenoid
) : SubsystemBase() {

  private var flywheelOn: Boolean = false

  // val trueDesiredVelocity: Double = spitterDesiredVelocity
  // why is trueDesiredVelocity needed if it's the same thing as spitterDesiredVelocity

  fun runIntake() {
    intakeMotor.set(CargoConstants.FEEDER_OUTPUT)
    spitterMotor.set(-spitterMotorVoltage)
  }

  fun runIntakeReverse() {
    intakeMotor.set(-CargoConstants.FEEDER_OUTPUT)
    spitterMotor.set(-spitterMotorVoltage)
  }

  fun deployIntake() {
    deployIntake.set(DoubleSolenoid.Value.kReverse)
  }

  fun retractIntake() {
    deployIntake.set(DoubleSolenoid.Value.kForward)
  }

  // deploy hood increases shooting height
  fun deployHood() {
    deployHood.set(DoubleSolenoid.Value.kReverse)
  }

  fun retractHood() {
    deployHood.set(DoubleSolenoid.Value.kForward)
  }

  override fun periodic() {
    if (flywheelOn) {
      spitterMotor.set(spitterDesiredVelocity)
    }
  }

  /*
  if (flywheelOn) {
    spitterMotor.setVoltage(
      motor.calculate(spitterDesiredVelocity) + shooterPIDController.calculate(
        spitterMotor.velocity,
        spitterDesiredVelocity
      )
    )
  }
  the above was replaced with "spitterMotor.set(spitterDesiredVelocity)"
   */

  fun shoot() {
    flywheelOn = true
  }

  fun stopShoot() {
    spitterMotor.setVoltage(0.0)
    flywheelOn = false
  }
}
