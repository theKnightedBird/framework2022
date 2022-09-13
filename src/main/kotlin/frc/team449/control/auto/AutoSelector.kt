package frc.team449.control.auto
import edu.wpi.first.networktables.NetworkTableEntry
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj2.command.CommandBase
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.WaitCommand


object AutoSelector {
  private var autonomousModeChooser: SendableChooser<AutoRoutine> = SendableChooser()
  private var waitBeforeCommandSlider: NetworkTableEntry
  private var secondaryWaitInAuto: NetworkTableEntry

  init {
    val autoTab = Shuffleboard.getTab("Auto settings")

    autoTab.add("Mode", autonomousModeChooser).withSize(5, 2).withPosition(3, 0)
    waitBeforeCommandSlider =
      autoTab
        .add("Wait Time before Running Auto", 0)
        .withSize(3, 2)
        .withPosition(0, 0)
        .withWidget(BuiltInWidgets.kTextView)
        .entry
    secondaryWaitInAuto =
      autoTab
        .add("Secondary Wait Time During Auto Path", 0)
        .withSize(3, 2)
        .withPosition(0, 2)
        .withWidget(BuiltInWidgets.kTextView)
        .entry
  }

  fun getWaitTime() : Double {
    return waitBeforeCommandSlider.getDouble(0.0)
  }

  fun getSecondaryWaitTime(): Double {
    return secondaryWaitInAuto.getDouble(0.0)
  }

  fun getCommand() : CommandBase {

    val mode = autonomousModeChooser.selected

    when (mode){

      else -> println("ERROR: unexpected auto mode: $mode")
    }
    return InstantCommand()
  }

  private enum class Auto {}


  }