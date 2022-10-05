package frc.team449.robot2022.cargo

// import edu.wpi.first.math.controller.PIDController
// import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.wpilibj.DoubleSolenoid
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.system.motor.WrappedMotor

class Cargo(
  val intakeMotor: WrappedMotor,
  val spitterMotor: WrappedMotor,
  val shooterMotor: WrappedMotor,
  var shooterDesiredVelocity: Double,
  // var spitterMotorVoltage: Double,
  // replacing the motor voltage bc idt we ever need a voltage but we do need two separate speed for spitting and intaking
  var spitterDesiredVelocityIntake: Double,
  var spitterDesiredVelocitySpit: Double,
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
    spitterMotor.set(-spitterDesiredVelocityIntake)
  }

  fun runIntakeReverse() {
    intakeMotor.set(-CargoConstants.FEEDER_OUTPUT)
    spitterMotor.set(-spitterDesiredVelocityIntake)
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
      shooterMotor.set(shooterDesiredVelocity)
      spitterMotor.set(spitterDesiredVelocitySpit)
    }
  }

//    if (flywheelOn) {
//      spitterMotor.setVoltage(
//        motor.calculate(spitterDesiredVelocitySpit) + shooterPIDController.calculate(
//          spitterMotor.velocity,
//          spitterDesiredVelocitySpit
//        )
//      )
//    }
  // the above was replaced with "spitterMotor.set(spitterDesiredVelocity)"

  fun shoot() {
    flywheelOn = true
  }

  fun stopShoot() {
    spitterMotor.set(0.0)
    shooterMotor.set(0.0)
    flywheelOn = false
  }
}
