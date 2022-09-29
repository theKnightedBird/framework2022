package frc.team449.control.holonomic

import com.revrobotics.CANSparkMax
import com.revrobotics.RelativeEncoder
import com.revrobotics.SparkMaxPIDController
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj.RobotBase
import frc.team449.robot2022.drive.DriveConstants
import frc.team449.system.encoder.AbsoluteEncoder
import frc.team449.system.motor.WrappedMotor
import io.github.oblarg.oblog.Loggable
import io.github.oblarg.oblog.annotations.Log
import kotlin.math.PI
import kotlin.math.abs

/**
 * @param name the name of the module (relevant for logging)
 * @param drivingMotor the motor that controls the speed of the module
 * @param turningMotor the motor that controls the turning(angle) of the module
 * @param driveController the velocity control PID for speed of the module
 * @param turnController the position control PID for turning(angle) of the module
 * @param driveFeedforward voltage predicting equation for a specified speed of the module
 * @param location the location of the module in reference to the center of the robot
 * NOTE: In relation to the robot [+X is forward, +Y is left, and +THETA is Counter Clock-Wise].
 */
open class SwerveModule constructor(
  private val name: String,
  private val drivingMotor: WrappedMotor,
  private val turningMotor: CANSparkMax,
  private val absEncoder: AbsoluteEncoder,
  private val driveController: PIDController,
  private val driveFeedforward: SimpleMotorFeedforward,
  val location: Translation2d,
  private val turnController: SparkMaxPIDController = turningMotor.pidController,
  private val turnEncoder: RelativeEncoder = turningMotor.encoder
) : Loggable {
  init {
    driveController.reset()

    turnController.p = DriveConstants.TURN_KP
    turnController.i = DriveConstants.TURN_KI
    turnController.d = DriveConstants.DRIVE_KD

    turnEncoder.positionConversionFactor = 2 * PI /** Measure in Radians */
    sync()
  }

  @Log.Graph
  private var desiredSpeed = 0.0
  @Log
  private var desiredAngle = turnEncoder.position

  open var state: SwerveModuleState
    @Log.ToString
    get() {
      return SwerveModuleState(
        drivingMotor.velocity,
        Rotation2d(turnEncoder.position)
      )
    }
    set(desiredState) {
      if (abs(desiredState.speedMetersPerSecond) < .001) {
        stop()
        return
      }
      /** Ensure the module doesn't turn the long way around */
      val state = SwerveModuleState.optimize(
        desiredState,
        Rotation2d(turnEncoder.position)
      )
      turnController.setReference(
        state.angle.radians,
        CANSparkMax.ControlType.kPosition
      )
      desiredSpeed = state.speedMetersPerSecond
      driveController.setpoint = state.speedMetersPerSecond
    }

  fun stop() {
    desiredAngle = turnEncoder.position
    desiredSpeed = 0.0
  }

  fun sync() {
    turnEncoder.position = absEncoder.position
  }

  override fun configureLogName() = this.name

  fun update() {
    val drivePid = driveController.calculate(
      drivingMotor.velocity
    )
    val driveFF = driveFeedforward.calculate(desiredSpeed)

    drivingMotor.setVoltage(drivePid + driveFF)
  }

  /**
   * Creates a simulated or a real robot based
   * on if the robot is being simulated.
   * @see SwerveModule for parameter description
   */
  companion object {
    fun create(
      name: String,
      drivingMotor: WrappedMotor,
      turningMotor: CANSparkMax,
      absEncoder: AbsoluteEncoder,
      driveController: PIDController,
      driveFeedforward: SimpleMotorFeedforward,
      location: Translation2d
    ): SwerveModule {
      if (RobotBase.isReal()) {
        return SwerveModule(
          name,
          drivingMotor,
          turningMotor,
          absEncoder,
          driveController,
          driveFeedforward,
          location
        )
      } else {
        return SwerveModuleSim(
          name,
          drivingMotor,
          turningMotor,
          absEncoder,
          driveController,
          driveFeedforward,
          location
        )
      }
    }
  }
}

/**
 * A "simulated" swerve module that just pretends
 * it immediately got to whatever desired state was given
 */
class SwerveModuleSim(
  name: String,
  drivingMotor: WrappedMotor,
  turningMotor: CANSparkMax,
  absEncoder: AbsoluteEncoder,
  driveController: PIDController,
  driveFeedforward: SimpleMotorFeedforward,
  location: Translation2d
) : SwerveModule(
  name,
  drivingMotor,
  turningMotor,
  absEncoder,
  driveController,
  driveFeedforward,
  location
) {

  override var state = SwerveModuleState()
}
