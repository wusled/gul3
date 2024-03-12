package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Swerve.Mod2;
import frc.robot.Constants.Swerve.Mod3;

public class Swerve extends SubsystemBase {

  public AHRS gyro = new AHRS(SPI.Port.kMXP);//gyro takilan porta tanimlandi
  
  private SwerveModule[] mSwerveMods = new SwerveModule[] { //her bir modul icin constantstan id ve angleoffset verileri swervemodsa atandi 
    new SwerveModule(0, Constants.Swerve.Mod0.constants),
    new SwerveModule(1, Constants.Swerve.Mod1.constants),
    new SwerveModule(2, Constants.Swerve.Mod2.constants),
    new SwerveModule(3, Constants.Swerve.Mod3.constants)
  };

  private Field2d field;//saha

  SwerveDriveOdometry m_SwerveOdometry;

  public Swerve() {
    
    zeroGyro();
    field = new Field2d();//saha 2d bicimli sekilde atandi 
    SmartDashboard.putData("Field", field);//smart dashboardda saha gorseli geldi 

    m_SwerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(),
    //Swerve odometry icin gerekilen  kinematics ==Her bir modul tekerinin uzakliklari
    // Yaw == gyro duz bir sekilde atandiysa ki buna Invert deniliyor Invert true ise Rotation2d cinsinden angle\
    //,Her bir modulden get position alindi get position drive motordan encdoder deger cekiyor ikinci deger olarakta getangle cekiyor Pose2d cinsinden ise ==baslangic robot pozisyonu veriliyor 
  new SwerveModulePosition[] {
     mSwerveMods[0].getPosition(),
     mSwerveMods[1].getPosition(),
     mSwerveMods[2].getPosition(),
     mSwerveMods[3].getPosition(),
   }, new Pose2d(Constants.Swerve.startingX , Constants.Swerve.startingY, new Rotation2d()));
  
   resetOdometry(new Pose2d(0,0, getYaw()));
  }  

   
//drive fonksiyonu 2d cinsinden translatino double cinsinden rotation ve field relative mi onlari aliyor 
  public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    SwerveModuleState[] swerveModuleStates =
        Constants.Swerve.swerveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    translation.getX(), translation.getY(), rotation, getYaw())
                : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {//her bir modul icin istenilen durumlar elde edliliyor ve bunu durmadan yapmasi icinde openloop 
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);//module durumlari ayarlaniyor burda max speed veriliyor constantstan hiz aliniyor 

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  public Pose2d getPose() {
    //getPose fonksiyonu swerveodometryden suanki durdugu konumun metre cinsinden x y tetha olarak veriyor
     //swerve modulde x y ne girilirse onun ustune konuluyor
    return m_SwerveOdometry.getPoseMeters();
  }


  public void resetOdometry(Pose2d pose) {//odometry reset  icin get yaw swervemoduleposition ve pose aliniyor modulepositon ve get yaw alinip burdan sifirlaniyor ve pose a esitleniyor 
    m_SwerveOdometry.resetPosition(getYaw(), new SwerveModulePosition[]{
      mSwerveMods[0].getPosition(),
      mSwerveMods[1].getPosition(),
      mSwerveMods[2].getPosition(),
      mSwerveMods[3].getPosition()
    }, pose);
  }

  public SwerveModuleState[] getStates() {//swervemodule states swerve modullerinin durumlarini het biri icin farkli bicimde alip states returnluyor 
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : mSwerveMods) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public void zeroGyro() {//gyrodan get yaw cekiliyor
      gyro.getYaw();
  }

  public Rotation2d getYaw() {//get yaw invert false mu olduguna yoksa true olduguna bakiyorum true ise tam tersini  false ise tam bir tur tersini 
    return (Constants.Swerve.invertGyro)
       ? Rotation2d.fromDegrees(360 - gyro.getYaw()) 
       : Rotation2d.fromDegrees(gyro.getYaw());
  }

  @Override
  public void periodic() {

    // get the rotation of the robot
    
    field.setRobotPose(getPose());//robotun  

    // update the pose
      m_SwerveOdometry.update(getYaw(),new SwerveModulePosition[]{
      mSwerveMods[0].getPosition(),
      mSwerveMods[1].getPosition(),
      mSwerveMods[2].getPosition(),
      mSwerveMods[3].getPosition()
    });



    SmartDashboard.putString("POSE" , getPose().toString());

    for (SwerveModule mod : mSwerveMods) {
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Integrated", mod.getState().angle.getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
    }
  }
}
