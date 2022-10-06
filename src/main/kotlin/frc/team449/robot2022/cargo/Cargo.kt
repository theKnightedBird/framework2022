package frc.team449.robot2022.cargo

// import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
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
  val motor: SimpleMotorFeedforward,
  // val shooterPIDController: PIDController,
  val intake: DoubleSolenoid,
  val hood: DoubleSolenoid
) : SubsystemBase() {

  private var flywheelOn: Boolean = false
  private var intakeDeployed: Boolean = false
  private var hoodDeployed: Boolean = false

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

  fun stopIntake() {
    intakeMotor.setVoltage(0.0)
    spitterMotor.setVoltage(0.0)
  }

  fun deployIntake() {
    intake.set(DoubleSolenoid.Value.kReverse)
  }

  fun retractIntake() {
    intake.set(DoubleSolenoid.Value.kForward)
  }

  fun changeIntakeState() {
    if (intakeDeployed) {
      retractIntake()
      intakeDeployed = false
    } else {
      deployIntake()
      intakeDeployed = true
    }
  }

  // deploy hood increases shooting height
  fun deployHood() {
    hood.set(DoubleSolenoid.Value.kForward)
  }

  fun retractHood() {
    hood.set(DoubleSolenoid.Value.kReverse)
  }

  fun changeHoodState() {
    if (hoodDeployed) {
      retractHood()
      hoodDeployed = false
    } else {
      deployHood()
      hoodDeployed = true
    }
  }

  override fun periodic() {
//    if (flywheelOn) {
//      shooterMotor.set(shooterDesiredVelocity)
//      spitterMotor.set(spitterDesiredVelocitySpit)
//    }
//

    // feedforward for shooting, replaced the above code
    if (flywheelOn) {

      shooterMotor.setVoltage(
        motor.calculate(shooterDesiredVelocity)
      )

      spitterMotor.setVoltage(
        motor.calculate(spitterDesiredVelocitySpit)
      )
    }
  }

  fun shoot() {
    flywheelOn = true
  }

  fun stopShoot() {
    spitterMotor.set(0.0)
    shooterMotor.set(0.0)
    flywheelOn = false
  }
}
