// package frc.robot.subsystems.LEDs;

// import org.littletonrobotics.junction.Logger;

// import edu.wpi.first.units.measure.Distance;
// import edu.wpi.first.wpilibj.AddressableLED;
// import edu.wpi.first.wpilibj.LEDPattern;
// import edu.wpi.first.wpilibj.util.Color;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

// public class LEDs extends SubsystemBase {

//     private final LEDsIO io;
//     //I donno how to make this 
//     private final LEDsIOInputsAutoLogged inputs = new LEDsIOInputsAutoLogged();

//     public void setLEDS(LEDsConstants LED){
//         LEDsConstants.LEDsIOInputs.LEDStrip.setLength((LEDsConstants.LEDsIOInputs.buffer.getLength()));
//         LEDsConstants.LEDsIOInputs.LEDStrip.setData(LEDsConstants.LEDsIOInputs.buffer);
//         LEDsConstants.LEDsIOInputs.LEDStrip.start();
//     }


//     public LEDs(LEDsIO io) {
//         this.io = io;
//     }

//     public Command Default(){
//         return null;
//     }

//     public Command hasCoral(LEDPattern color, LEDsConstants buffer){
//         color.applyTo(LEDsConstants.LEDsIOInputs.buffer);
//         return Commands.run(() -> LEDsConstants.LEDsIOInputs.LEDStrip.setData(LEDsConstants.LEDsIOInputs.buffer), this);
//     }

//     public Command hasAlgea(LEDPattern color, LEDsConstants buffer){
//         color.applyTo(LEDsConstants.LEDsIOInputs.buffer);
//         return Commands.run(() -> LEDsConstants.LEDsIOInputs.LEDStrip.setData(LEDsConstants.LEDsIOInputs.buffer), this);
//     }

//     public Command goingToCoralIntake(LEDPattern color, LEDsConstants buffer){
//         color.applyTo(LEDsConstants.LEDsIOInputs.buffer);
//         return Commands.run(() -> LEDsConstants.LEDsIOInputs.LEDStrip.setData(LEDsConstants.LEDsIOInputs.buffer), this);
//     }

//     public Command goingToAlgaeIntake(){
//         return null;
//     }

//     public Command intakingCoral(){
//         return null;
//     }

//     public Command scoringCoral(LEDPattern color, LEDsConstants buffer){
//         color.applyTo(LEDsConstants.LEDsIOInputs.buffer);
//         return Commands.run(() -> LEDsConstants.LEDsIOInputs.LEDStrip.setData(LEDsConstants.LEDsIOInputs.buffer), this);
//     }

//     public Command scoringAlgae(LEDPattern color, LEDsConstants buffer){
//         color.applyTo(LEDsConstants.LEDsIOInputs.buffer);
//         return Commands.run(() -> LEDsConstants.LEDsIOInputs.LEDStrip.setData(LEDsConstants.LEDsIOInputs.buffer), this);
//     }

//     public Command makePatternScroll(LEDsConstant Length) {
//         Distance ledSpacing = .of(1 / 120.0);
//         LEDPattern base = LEDPattern.discontinuousGradient(Color.kRed, Color.kBlue);
//         LEDPattern pattern = base.scrollAtRelativeSpeed(Percent.per(Second).of(25));
//         LEDPattern absolute = base.scrollAtAbsoluteSpeed(Centimeters.per(Second).of(12.5), ledSpacing);
//     }
// }
