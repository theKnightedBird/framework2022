package frc.team449.robot2022

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.filter.SlewRateLimiter
import edu.wpi.first.wpilibj.*
import edu.wpi.first.wpilibj.RobotBase.isReal
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import frc.team449.RobotContainerBase
import frc.team449.control.auto.AutoRoutine
import frc.team449.control.holonomic.OIHolonomic
import frc.team449.control.holonomic.SwerveDrive
import frc.team449.robot2022.auto.Example
import frc.team449.robot2022.drive.DriveConstants
import frc.team449.system.AHRS
import frc.team449.system.encoder.AbsoluteEncoder
import frc.team449.system.encoder.NEOEncoder
import frc.team449.system.motor.createSparkMax
import io.github.oblarg.oblog.annotations.Log
import kotlin.math.abs

class RobotContainer2022() : RobotContainerBase() {

  // Other CAN IDs
  private val PDP_CAN = 1
  private val PCM_MODULE = 0

  private val driveController = XboxController(0)

  private val ahrs = AHRS(SerialPort.Port.kMXP)

  // Instantiate/declare PDP and other stuff here

  override val powerDistribution: PowerDistribution = PowerDistribution(PDP_CAN, PowerDistribution.ModuleType.kCTRE)

  /**
   * Converts the drive to a SwerveSim if the robot is in simulation
   */
  @Log.Include
  override val drive = if (isReal()) createDrivetrain() else SwerveDrive.simOf(createDrivetrain())

  override val autoChooser = addRoutines()

  override val oi = OIHolonomic(
    drive,
    { if (abs(driveController.leftY) < .08) .0 else -driveController.leftY },
    { if (abs(driveController.leftX) < .08) .0 else -driveController.leftX },
    { if (abs(driveController.getRawAxis(4)) < .07) .0 else -driveController.getRawAxis(4) },
    SlewRateLimiter(10.5),
    4.5,
    { true }
  )

  /** Helper to make turning motors for swerve */
  private fun makeDrivingMotor(
    name: String,
    motorId: Int,
    inverted: Boolean
  ) =
    createSparkMax(
      name = name + "Drive",
      id = motorId,
      enableBrakeMode = true,
      inverted = inverted,
      encCreator =
      NEOEncoder.creator(
        DriveConstants.DRIVE_UPR,
        DriveConstants.DRIVE_GEARING
      )
    )

  /** Helper to make turning motors for swerve */
  private fun makeTurningMotor(
    motorId: Int,
    inverted: Boolean
  ): CANSparkMax {
    val enc = CANSparkMax(
      motorId,
      CANSparkMaxLowLevel.MotorType.kBrushless
    )
    enc.inverted = inverted
    enc.idleMode = CANSparkMax.IdleMode.kBrake
    return enc
  }

  /** Helper to make the turning absolute encoders for swerve */
  private fun makeTurningEncoder(
    name: String,
    encChannel: Int,
    offset: Double,
    inverted: Boolean
  ) = AbsoluteEncoder(
    name,
    DutyCycleEncoder(encChannel),
    DriveConstants.TURN_UPR,
    inverted,
    offset
  )
  private fun createDrivetrain() =
    SwerveDrive.swerveDrive(
      ahrs,
      DriveConstants.MAX_LINEAR_SPEED,
      DriveConstants.MAX_ROT_SPEED,
      makeDrivingMotor(
        "FL",
        DriveConstants.DRIVE_MOTOR_FL,
        false
      ),
      makeDrivingMotor(
        "FR",
        DriveConstants.DRIVE_MOTOR_FR,
        false
      ),
      makeDrivingMotor(
        "BL",
        DriveConstants.DRIVE_MOTOR_BL,
        false
      ),
      makeDrivingMotor(
        "BR",
        DriveConstants.DRIVE_MOTOR_BR,
        false
      ),
      makeTurningMotor(
        DriveConstants.TURN_MOTOR_FL,
        true
      ),
      makeTurningEncoder(
        "FL",
        DriveConstants.TURN_ENC_CHAN_FL,
        DriveConstants.TURN_ENC_OFFSET_FL,
        false
      ),
      makeTurningMotor(
        DriveConstants.TURN_MOTOR_FR,
        true
      ),
      makeTurningEncoder(
        "FR",
        DriveConstants.TURN_ENC_CHAN_FR,
        DriveConstants.TURN_ENC_OFFSET_FR,
        false
      ),
      makeTurningMotor(
        DriveConstants.TURN_MOTOR_BL,
        true
      ),
      makeTurningEncoder(
        "BL",
        DriveConstants.TURN_ENC_CHAN_BL,
        DriveConstants.TURN_ENC_OFFSET_BL,
        false
      ),
      makeTurningMotor(
        DriveConstants.TURN_MOTOR_BR,
        true
      ),
      makeTurningEncoder(
        "BR",
        DriveConstants.TURN_ENC_CHAN_BR,
        DriveConstants.TURN_ENC_OFFSET_BR,
        false
      ),
      DriveConstants.WHEELBASE,
      DriveConstants.TRACKWIDTH,
      { PIDController(DriveConstants.DRIVE_KP, DriveConstants.DRIVE_KI, DriveConstants.DRIVE_KD) },
      SimpleMotorFeedforward(DriveConstants.DRIVE_KS, DriveConstants.DRIVE_KV, DriveConstants.DRIVE_KA)
    )

  private fun addRoutines(): SendableChooser<AutoRoutine> {
    val chooser = SendableChooser<AutoRoutine>()
    val exampleAuto = Example(this)
    chooser.setDefaultOption("Example Auto", exampleAuto.routine())
    return chooser
  }

  override fun teleopInit() {
    // todo Add button bindings here
  }

  override fun robotPeriodic() {
  }

  override fun simulationInit() {
  }

  override fun simulationPeriodic() {
    // Update simulated mechanisms on Mechanism2d widget and stuff
  }

  override fun disabledInit() {
    drive.stop()
  }
}
