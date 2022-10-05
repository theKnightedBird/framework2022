package frc.team449.robot2022

import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.filter.SlewRateLimiter
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.wpilibj.*
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.button.JoystickButton
import frc.team449.RobotContainerBase
import frc.team449.control.auto.AutoRoutine
import frc.team449.control.differential.DifferentialDrive
import frc.team449.control.differential.DifferentialOIs
import frc.team449.robot2022.cargo.Cargo
import frc.team449.robot2022.cargo.CargoConstants
import frc.team449.robot2022.drive.DriveConstants
import frc.team449.system.AHRS
import frc.team449.system.encoder.NEOEncoder
import frc.team449.system.motor.createSparkMax
import io.github.oblarg.oblog.annotations.Log
import kotlin.math.absoluteValue

class RobotContainer2022() : RobotContainerBase() {

  // Other CAN IDs
  val PDP_CAN = 1

  val ahrs = AHRS(SerialPort.Port.kMXP)

  override val powerDistribution = PowerDistribution(PDP_CAN, PowerDistribution.ModuleType.kCTRE)

  override val autoChooser: SendableChooser<AutoRoutine> = addRoutines()

  // Instantiate/declare PDP and other stuff here

  val pdp = PowerDistribution()

  @Log.Include
  override val drive = createDrivetrain()

  override val driveSim = null

  val driveController = XboxController(DriveConstants.DRIVE_CONTROLLER_PORT)

  override val oi =
    DifferentialOIs.createCurvature(
      drive,
      { driveController.rightTriggerAxis - driveController.leftTriggerAxis },
      {
        if (driveController.leftX.absoluteValue < DriveConstants.DRIVE_TURNING_DEADBAND) .0
        else driveController.leftX
      },
      SlewRateLimiter(DriveConstants.LINEAR_ACC_LIMIT),
      SlewRateLimiter(DriveConstants.TURNING_ACC_LIMIT),
      { true }
    )

  /** Helper to make each side for the differential drive */
  private fun makeSide(
    name: String,
    motorId: Int,
    inverted: Boolean,
    wpiEnc: Encoder,
    followers: Map<Int, Boolean>
  ) =
    createSparkMax(
      name = name + "_Drive",
      // nameQuad = name + "_DriveQuad",
      id = motorId,
      enableBrakeMode = true,
      inverted = inverted,
      encCreator =

      NEOEncoder.creator(
        DriveConstants.DRIVE_UPR,
        DriveConstants.DRIVE_GEARING
      ),

      slaveSparks = followers,
      currentLimit = DriveConstants.DRIVE_CURRENT_LIM
    )

  private fun createDrivetrain() =
    DifferentialDrive(
      leftLeader = makeSide(
        "Left",
        DriveConstants.DRIVE_MOTOR_L,
        false,
        DriveConstants.DRIVE_ENC_LEFT,
        mapOf(
          DriveConstants.DRIVE_MOTOR_L1 to false,
          DriveConstants.DRIVE_MOTOR_L2 to false
        )
      ),
      rightLeader = makeSide(
        "Right",
        DriveConstants.DRIVE_MOTOR_R,
        true,
        DriveConstants.DRIVE_ENC_RIGHT,
        mapOf(
          DriveConstants.DRIVE_MOTOR_R1 to false,
          DriveConstants.DRIVE_MOTOR_R2 to false
        )
      ),
      ahrs,
      SimpleMotorFeedforward(
        DriveConstants.DRIVE_FF_KS,
        DriveConstants.DRIVE_FF_KV,
        DriveConstants.DRIVE_FF_KA
      ),
      {
        PIDController(
          DriveConstants.DRIVE_KP_VEL,
          DriveConstants.DRIVE_KI_VEL,
          DriveConstants.DRIVE_KD_VEL
        )
      },
      DriveConstants.TRACK_WIDTH,
      DriveConstants.MAX_LINEAR_SPEED
    )

  private fun createDriveSimController() =
    DifferentialDrive.SimController(
      drive,
      DifferentialDrivetrainSim(
        LinearSystemId.identifyDrivetrainSystem(
          DriveConstants.DRIVE_FF_KV,
          DriveConstants.DRIVE_FF_KA,
          DriveConstants.DRIVE_ANGLE_FF_KV,
          DriveConstants.DRIVE_ANGLE_FF_KA
        ),
        DCMotor.getNEO(3),
        DriveConstants.DRIVE_GEARING,
        DriveConstants.TRACK_WIDTH,
        DriveConstants.DRIVE_WHEEL_RADIUS,
        VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005)
      ),
      AHRS.SimController()
    )

  private fun addRoutines(): SendableChooser<AutoRoutine> {
    val chooser = SendableChooser<AutoRoutine>()
    // chooser.setDefaultOption("Example Differential Auto", exampleAuto.routine())

    return chooser
  }

  val intakeMotor = createSparkMax(
    "Intake Leader",
    8,
    NEOEncoder.creator(
      1.0,
      1.0
    ),
    slaveSparks = mapOf(10 to false)
  )

  val spitterMotor = createSparkMax(
    "Spitter",
    12,
    NEOEncoder.creator(
      1.0,
      1.0
    ),
  )

  val shooterMotor = createSparkMax(
    "Shooter",
    8,
    NEOEncoder.creator(
      1.0,
      1.0
    ),
  )

  val motorFF = SimpleMotorFeedforward(
    CargoConstants.SHOOTER_KS,
    CargoConstants.SHOOTER_KV,
    CargoConstants.SHOOTER_KA
  )

  // val shooterPIDController =

  val intakePiston = DoubleSolenoid(
    PneumaticsModuleType.CTREPCM,
    CargoConstants.INTAKE_PISTON_FWD_CHANNEL,
    CargoConstants.INTAKE_PISTON_REV_CHANNEL
  )

  val hoodPiston = DoubleSolenoid(
    PneumaticsModuleType.CTREPCM,
    CargoConstants.HOOD_PISTON_FWD_CHANNEL,
    CargoConstants.HOOD_PISTON_REV_CHANNEL
  )

  val intakeVelocity = CargoConstants.SPITTER_INTAKE_SPEED_RPS / 94.6
  val spitVelocity = CargoConstants.SPITTER_SPIT_SPEED_RPS / 94.6
  val shooterVelocity = CargoConstants.SHOOTER_SPEED_RPS / 94.6

  // dividing the rps by 94.6 to convert to motor percentage for .set()
  // idk if it's right but i just did some calculations based on the motor specs

  val roboCargo = Cargo(
    intakeMotor,
    spitterMotor,
    shooterMotor,
    shooterVelocity,
    intakeVelocity,
    spitVelocity,
    intakePiston,
    hoodPiston
  )

  override fun teleopInit() {
    JoystickButton(driveController, XboxController.Button.kA.value).whenPressed(
      InstantCommand()
    )
  }

  // will do stuff here
  override fun teleopPeriodic() {
  }

  override fun robotPeriodic() {
  }

  override fun simulationInit() {
    // DriverStationSim.setEnabled(true)
  }

  override fun simulationPeriodic() {
    // Update simulated mechanisms on Mechanism2d widget and stuff
  }
}
