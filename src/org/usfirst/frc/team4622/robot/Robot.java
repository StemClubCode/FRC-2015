
package org.usfirst.frc.team4622.robot;

import com.ni.vision.NIVision;
import com.ni.vision.NIVision.Image;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Relay;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
		// Joysticks
		Joystick driver1;
		Joystick driver2;
		
		//Button Constants
		private static final int A = 1;
		private static final int B = 2;
		private static final int X = 3;
		private static final int Y = 4;
		private static final int LB = 5;
		private static final int RB = 6;
		private static final int LT = 2;
		private static final int RT = 3;
		private static final int START = 7;
		private static final int SELECT = 8;

		//Drivetrain
		RobotDrive drivetrain;
		Jaguar frontLeft;
		Jaguar backLeft;
		Jaguar frontRight;
		Jaguar backRight;

		//Misc. Motors
		Talon lift;
		Talon lift2;
		Talon picker;

		//IP Camera Servos
		Servo horCam;
		Servo verCam;

		//Gyros
		Gyro driveGyro;

		//USB camera
		CameraServer USBcameraServer;
		
		//IP camera
		int session;
		Image frame;

		//Dashboard
		SmartDashboard dash;

		//Variables
		double controllerSensitivity;
		double driveValue1;
		double driveValue2;
		double driveValue3;
		double strafeValue;
		
		double curAngle;

		double horPos;
		double verPos;
		
		double robotSpeed;
		
		boolean clicked;

		boolean reversed;
		boolean justReversed;
		
		boolean speedToggle;
		boolean justSpeedToggle;
		
		//Limits
		DigitalInput liftUpperLimit;
		boolean liftUpperValue;
		DigitalInput pickLimit;
		boolean pickLimitValue;
		DigitalInput liftLowerLimit;
		boolean liftLowerValue;

		//Encoder
		Encoder pickEncoder;
		
		Timer timer;

		// Pneumatic
		Compressor compressor;
		DoubleSolenoid liftSolenoidUpper;
		DoubleSolenoid liftSolenoidLower;
	
		// Autonomous
		int loopCount;
		
		//Relay
		Relay lightRelay;
		
	/**
	 * Instantiates variables, motors, gyros, cameras, solenoids, and compressor
	 */
    public void robotInit() {
    	//Drive motors
		frontLeft = new Jaguar(0);
		frontRight = new Jaguar(1);
		backRight = new Jaguar(2);
		backLeft = new Jaguar(3);
		
		//Lift motors
		lift = new Talon(6);
		lift2 = new Talon(5);
		
		//Picker motor
		picker = new Talon(4);
		
		//Joysticks
		driver1 = new Joystick(0);
		driver2 = new Joystick(1);
		
		//Camera Servos
		horCam = new Servo(7);
		verCam = new Servo(8);

		// Gyros
		driveGyro = new Gyro(0);
		driveGyro.startLiveWindowMode();

		// Sensors
		
		//IP camera settings
		frame = NIVision.imaqCreateImage(NIVision.ImageType.IMAGE_RGB, 0);
		session = NIVision.IMAQdxOpenCamera("cam1",
		NIVision.IMAQdxCameraControlMode.CameraControlModeController);
		NIVision.IMAQdxConfigureGrab(session);
		
		//USB camera settings
		USBcameraServer = CameraServer.getInstance();
		USBcameraServer.setQuality(50);
		USBcameraServer.startAutomaticCapture("cam2");
		
		// Dashboard
		dash = new SmartDashboard();

		timer = new Timer();
		
		// Variables
		horPos = 90;
		verPos = 60;
		
		//Sets robot's default speed
		robotSpeed = 2;
		
		//Toggle variables for drive train orientation
		reversed = false;
		justReversed = false;
		
		///Toggle variables for robot speed 
		speedToggle = false;
		justSpeedToggle = false;
		
		//Limit switches
		liftUpperLimit = new DigitalInput(5);
		pickLimit = new DigitalInput(6);
		liftLowerLimit = new DigitalInput(4);
		

		//Pneumatic
		compressor = new Compressor(0);
		liftSolenoidUpper = new DoubleSolenoid(1, 0);
		liftSolenoidLower = new DoubleSolenoid(2, 3);
		
		//Encoder
		pickEncoder = new Encoder(0,1, true, EncodingType.k4X);
		
		//Relay
		lightRelay = new Relay(0);
		
		SmartDashboard.putNumber("Autonomous", 0);
		
    }
    
    public void disabledInit(){
    	lightRelay.set(Relay.Value.kOff);
    	
    }

    
    public void autonomousInit(){
    	lightRelay.set(Relay.Value.kForward);
    	
    	loopCount = 0;
    	compressor.stop();
    	
    	horCam.setAngle(90);
    	verCam.setAngle(60);
    
    }
    
    /**
     * This function is called at the beginning of autonomous
     */
    public void autonomousPeriodic() {
		loopCount++;
    	
		int autoDelay = (int) SmartDashboard.getNumber("Auto Delay", 0);
		//max delay is 5s
		if(autoDelay > 5){
			autoDelay = 5;
		}
		//convert the seconds to 50Hz cycles
		int autoDelayCounts = 50*autoDelay;
		
		//hot close to the center a tote has to be to trigger the lineup code
		int toteXTargetingRange = 400;
		//how close to lined up it has to be to drive forward into a crate
		int toteXLinupTolerance = 50;
		int toteAngleLinupTolerance = 20;
		
		double targetingStrafeMotorSpeed = 0.2;
		double targetingDriveMotorSpeed = 0.2;
		double strafeMotorSpeed = 0.5;
		double driveMotorSpeed = 0.3;
		
		//update limit booleans
		limitValues();
		
		if(loopCount < 10){
			liftSolenoidUpper.set(DoubleSolenoid.Value.kReverse);
			liftSolenoidLower.set(DoubleSolenoid.Value.kReverse);
		}
    	
		if((int) SmartDashboard.getNumber("Autonomous", 0) < 0 || (int) SmartDashboard.getNumber("Autonomous", 0) > 1){
			SmartDashboard.putNumber("Autonomous",0);
		}
		
	    switch((int) SmartDashboard.getNumber("Autonomous", 0)){
	    	case 0:
	    		if(loopCount < autoDelayCounts){
	    			//waiting
	    		}
	    		
				if(loopCount < autoDelayCounts + 100){
					liftSet(-1);
				}
				
				if(autoDelayCounts + loopCount > 100 && autoDelayCounts + loopCount < 260){
					liftSet(0);
					frontLeft.set(-.5);
					backLeft.set(-.5);
					frontRight.set(.5);
					backRight.set(.5);
				}
				
				if(autoDelayCounts + loopCount > 260 && autoDelayCounts + loopCount < 400){
					frontLeft.set(0);
					backLeft.set(0);
					frontRight.set(0);
					backRight.set(0);
					if(liftLowerLimit.get()){
						liftSet(0.5);
					}else{
						liftSet(0);
					}
				}
				
				if(autoDelayCounts + loopCount > 400 && autoDelayCounts + loopCount < 500){
						liftSet(0);
				}
				
				break;
	    	case 1:
	    		if(loopCount < autoDelayCounts){
	    			//waiting
	    		}
	    		
	    		//lift up the first crate thats lined up
				if(loopCount < autoDelayCounts + 150){
					//raise the lift with the first tote in it
					liftSet(-1);
				}
				
				//back up with the first tote
				if(autoDelayCounts + loopCount > 150 && autoDelayCounts + loopCount < 180){
					//stop the lift
					liftSet(0);
					//drive backwards
					frontLeft.set(-driveMotorSpeed);
					backLeft.set(-driveMotorSpeed);
					frontRight.set(driveMotorSpeed);
					backRight.set(driveMotorSpeed);
					
					
				}
				
				//strafe right until the second tote
				if(autoDelayCounts + loopCount > 180 && autoDelayCounts + loopCount < 320){
					
					//if a tote is within targeting range
					if(Math.abs((int) SmartDashboard.getNumber("ToteX", 401)) < toteXTargetingRange && Math.abs((int) SmartDashboard.getNumber("ToteX", 401)) != 0){
						//if within toteXLinupTolerance
						if(Math.abs((int) SmartDashboard.getNumber("ToteX", 401)) < toteXLinupTolerance ){
							//stop strafing right, move on in autonomous
							loopCount = 320;
							//stop all motors
							frontLeft.set(0);
							backLeft.set(0);
							frontRight.set(0);
							backRight.set(0);
							
						}else{
							//tote not in toteXLinupTolerance, but within targeting range
							if((int) SmartDashboard.getNumber("ToteX", 401) < 0){
								//tote is to the left so strafe left
								frontLeft.set(-targetingStrafeMotorSpeed);
								backLeft.set(0.9*targetingStrafeMotorSpeed);
								frontRight.set(-targetingStrafeMotorSpeed);
								backRight.set(0.9*targetingStrafeMotorSpeed);
							}else if((int) SmartDashboard.getNumber("ToteX", 401) > 0){
								//tote is to the right so strafe right
								frontLeft.set(targetingStrafeMotorSpeed);
								backLeft.set(-0.9*targetingStrafeMotorSpeed);
								frontRight.set(targetingStrafeMotorSpeed);
								backRight.set(-0.9*targetingStrafeMotorSpeed);
							}
						}
					}else{
						//tote isn't in targeting range, so keep strafing right
						frontLeft.set(strafeMotorSpeed);
						backLeft.set(-0.9*strafeMotorSpeed);
						frontRight.set(strafeMotorSpeed);
						backRight.set(-0.9*strafeMotorSpeed);
					}
				}
				
				//go forwards into the next tote
				if(autoDelayCounts + loopCount > 320 && autoDelayCounts + loopCount < 400){
					//drive forwards
					frontLeft.set(driveMotorSpeed);
					backLeft.set(driveMotorSpeed);
					frontRight.set(-driveMotorSpeed);
					backRight.set(-driveMotorSpeed);
				}
				
				//stack first tote on second tote
				if(autoDelayCounts + loopCount > 400 && autoDelayCounts + loopCount < 430){
					//stop driving
					frontLeft.set(0);
					backLeft.set(0);
					frontRight.set(0);
					backRight.set(0);
					//set first tote on second
					liftSet(0.5);
				}
				
				//back up a bit and start lowering tote lift
				if(autoDelayCounts + loopCount > 430 && autoDelayCounts + loopCount < 450){
					//set first tote on second
					liftSet(0.5);
					//drive backwards
					frontLeft.set(-driveMotorSpeed);
					backLeft.set(-driveMotorSpeed);
					frontRight.set(driveMotorSpeed);
					backRight.set(driveMotorSpeed);
				}
				
				//lower tote lift all the way
				if(autoDelayCounts + loopCount > 450 && autoDelayCounts + loopCount < 500){
					//stop driving
					frontLeft.set(0);
					backLeft.set(0);
					frontRight.set(0);
					backRight.set(0);
					
					//check if lift is all the way down
					if(!liftLowerValue){
						liftSet(-1);
					}else{
						liftSet(0);
						//lift is all the way down, continue
						loopCount = 500;
					}
					
				}
				
				//go forward into tote
				if(autoDelayCounts + loopCount > 500 && autoDelayCounts + loopCount < 520){
					//drive forwards
					frontLeft.set(driveMotorSpeed);
					backLeft.set(driveMotorSpeed);
					frontRight.set(-driveMotorSpeed);
					backRight.set(-driveMotorSpeed);
				}
				
				//lift up the stack of two totes
				if(autoDelayCounts + loopCount > 520 && autoDelayCounts + loopCount < 650){
					//raise the lift with both totes in it
					liftSet(-1);//drive forwards
					frontLeft.set(targetingDriveMotorSpeed);
					backLeft.set(targetingDriveMotorSpeed);
					frontRight.set(-targetingDriveMotorSpeed);
					backRight.set(-targetingDriveMotorSpeed);
				}
				
				//stop tote lift
				if(autoDelayCounts + loopCount > 650 && autoDelayCounts + loopCount < 700){
					//stop raising stack of two totes
					liftSet(0);
					//stop driving
					frontLeft.set(0);
					backLeft.set(0);
					frontRight.set(0);
					backRight.set(0);
				}
				
				
				
	    		break;
	    }
    }

    public void teleopInit(){
    	lightRelay.set(Relay.Value.kForward);
    	
		drivetrain = new RobotDrive(frontLeft, backLeft, frontRight, backRight);
		drivetrain.setInvertedMotor(RobotDrive.MotorType.kFrontLeft, true);
		drivetrain.setInvertedMotor(RobotDrive.MotorType.kRearLeft, true);
	
		IPcameraInit();
		
		
		
		pickEncoder.reset();
		driveGyro.reset();
		driveGyro.setSensitivity(0.007);
    }
    
    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
    	
    	camerasStart();

    	limitValues();
		
		smartDashboardInfo();

		contDead();

		if(!driver1.getRawButton(LB)){
			drivetrain.mecanumDrive_Cartesian(driveValue1, driveValue2, driveValue3, 0);
		}else{
			driveForward(.2);
		}
		
		cameraMove();
		setSpeed();
		liftMove();
		strafe();
		airSystem();
		pickerMove();
		autoPick();
		//autoCrate();
		
		
		
		Timer.delay(.002);
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    
    }
    
    /**
     * Controls servos that rotates IP camera
     */
    private void cameraMove() {
		if (driver2.getRawAxis(1) - .2 > 0) {
			if (verPos < 179) {
				verPos += 5;
			}
		} else if (driver2.getRawAxis(1) + .2 < 0) {
			if (verPos > 0) {
				verPos -= 5;
			}
		} else {

		}

		if (driver2.getRawAxis(0) - .2 > 0) {
			if (horPos < 179) {
				horPos += 5;
			}
		} else if (driver2.getRawAxis(0) + .2 < 0) {
			if (horPos > 0) {
				horPos -= 5;
			}
		} else {

		}
		
		if (driver2.getRawButton(START)) {
			verPos = 60;
			horPos = 90;
		}
		
		if (driver2.getRawButton(SELECT)){
			verPos = 45;
			horPos = 10;
		}

		horCam.setAngle(horPos);
		verCam.setAngle(verPos);
	}
    
    /**
     * Sets deadzones and reverses drive direction
     */

	private void contDead() {
		if (driver1.getRawButton(Y)){
			if(!justReversed){
				reversed = !reversed;
				justReversed = true;
			}
		}else{
			justReversed = false;
		}
		
		//left-right
		if (Math.abs(driver1.getRawAxis(0)) > 0.2) {
			driveValue1 = driver1.getRawAxis(0) / robotSpeed;
		} else {
			driveValue1 = 0;
		}

		//up-down
		if (Math.abs(driver1.getRawAxis(1)) > 0.2) {
			driveValue2 = driver1.getRawAxis(1) / robotSpeed;
		} else {
			driveValue2 = 0;   
		}

		//right left-right
		if (Math.abs(driver1.getRawAxis(4)) > 0.2) {
			driveValue3 = -driver1.getRawAxis(4) / robotSpeed;
		} else {
			driveValue3 = 0;   
		}
		
		if(reversed){
			driveValue1 = -driveValue1;
			driveValue2 = -driveValue2;
		}
	}

	/**
	 * Controls the lift winch
	 */
	private void liftMove() {
		if (driver1.getRawButton(RB)) {
			liftSet(-1);
		} else if (driver1.getRawButton(B)) {
			liftSet(0.5);
		} else {
			liftSet(0);
		}
		
	}
	
	/**
	 * Controls picker winch
	 */

	private void pickerMove() {
		if (driver2.getRawButton(B)) {
			picker.set(.8);
		} else if (driver2.getRawButton(A)) {
			picker.set(-.8);
		} else {
			picker.set(0);
		}
	}
	
	/**
	 * Sets robot speed by toggle the X button
	 */

	private void setSpeed() {
		if (driver1.getRawButton(X)){
			if(!justSpeedToggle){
				speedToggle = !speedToggle;
				justSpeedToggle = true;
			}
		}else{
			justSpeedToggle = false;
		}
		
		if (speedToggle){
			robotSpeed = 4;
		} else {
			robotSpeed = 2;
		}
	}
	
	/**
	 * Strafes the robot
	 */

	private void strafe() {
		if (Math.abs(driver1.getRawAxis(RT)) > .2) {
			if(reversed){
				strafeValue = -driver1.getRawAxis(RT);
			}else{
				strafeValue = driver1.getRawAxis(RT);
			}
			frontLeft.set(-strafeValue / robotSpeed);
			backLeft.set(strafeValue / robotSpeed);
			frontRight.set(-strafeValue / robotSpeed);
			backRight.set(strafeValue / robotSpeed);

		} else if (Math.abs(driver1.getRawAxis(LT)) > .2) {
			if(reversed){
				strafeValue = -driver1.getRawAxis(LT);
			}else{
				strafeValue = driver1.getRawAxis(LT);
			}
			frontLeft.set(strafeValue / robotSpeed);
			backLeft.set(-strafeValue / robotSpeed);
			frontRight.set(strafeValue / robotSpeed);
			backRight.set(-strafeValue / robotSpeed);
		} else {

		}
	}
	
	/**
	 * Handles pneumatics on picker arm
	 */

	private void airSystem() {
		if (driver2.getRawButton(RB)) {
			liftSolenoidUpper.set(DoubleSolenoid.Value.kForward);
			liftSolenoidLower.set(DoubleSolenoid.Value.kForward);
		} else if (driver2.getRawAxis(RT) > .2) {
			liftSolenoidUpper.set(DoubleSolenoid.Value.kReverse);
			liftSolenoidLower.set(DoubleSolenoid.Value.kReverse);
		} else if (driver2.getRawButton(LB)) {
			compressor.start();
		} else if (driver2.getRawAxis(LT) > .2) { 
			liftSolenoidUpper.set(DoubleSolenoid.Value.kForward);
			liftSolenoidLower.set(DoubleSolenoid.Value.kReverse);
		} else {
			compressor.stop();
		}
		
		//Sends upper solenoid state to driver station
		if (liftSolenoidUpper.get().equals(DoubleSolenoid.Value.kForward)){
			SmartDashboard.putNumber("Upper Solenoid", 9.0);
		} else  if (liftSolenoidUpper.get().equals(DoubleSolenoid.Value.kReverse)) {
			SmartDashboard.putNumber("Upper Solenoid", 1.0);
		} 
		
		//Sends lower solenoid state to driver station
		if (liftSolenoidLower.get().equals(DoubleSolenoid.Value.kForward)){
			SmartDashboard.putNumber("Lower Solenoid", 9.0);
		} else  if (liftSolenoidLower.get().equals(DoubleSolenoid.Value.kReverse)) {
			SmartDashboard.putNumber("Lower Solenoid", 1.0);
		} 
	}
	
	/**
	 * When the picker limit switch touches the can, extends both actuator
	 */
	private void autoPick(){
		if(pickLimitValue){
			liftSolenoidLower.set(DoubleSolenoid.Value.kForward);
			liftSolenoidUpper.set(DoubleSolenoid.Value.kForward);
		}
	}
	
	/**
	 * Auto picks crates with A button
	 */
	private void autoCrate(){
		
		if(driver1.getRawButton(A)){
			liftSet(-.5);
			
			Timer.delay(.2);
			
			liftSet(0);
			
			driveBackwards(1);
			
			Timer.delay(.2);
			
			driveStop();
			
			timer.start();
			
			while(!liftLowerValue || timer.get() < 1.5){
				liftSet(-.5);
			}
			
			timer.stop();
			timer.reset();
			
			driveForward(1);
			
			Timer.delay(1);
			
			driveStop();
			
			liftSet(1);
			
			Timer.delay(1.5);
			
			liftSet(0);
		}
	}
	
	/**
	 * Robot drives forward at desired speed
	 */
	private void driveForward(double speed){
		frontLeft.set(speed);
		backLeft.set(speed);
		frontRight.set(-speed);
		backRight.set(-speed);
	}
	
	/**
	 * Robot drives backwards at desired speed
	 */
	private void driveBackwards(double speed){
		frontLeft.set(-speed);
		backLeft.set(-speed);
		frontRight.set(speed);
		backRight.set(speed);
	}
	
	/**
	 * Robot stops moving
	 */
	private void driveStop(){
		frontLeft.set(0);
		backLeft.set(0);
		frontRight.set(0);
		backRight.set(0);
	}
	
	/**
	 * Sets the lift motors speed
	 * @param speed (-1 to 1)
	 */
	private void liftSet(double speed){
		if(speed < 0 && liftUpperValue == false){
			lift.set(speed);
			lift2.set(speed);
		}else if(speed > 0 && liftLowerValue == false){
			lift.set(speed);
			lift2.set(speed);
		}else{
			lift.set(0);
			lift2.set(0);
		}

	}
	
	/**
	 * Display variables/info to Java dashboard
	 */
	private void smartDashboardInfo(){
		SmartDashboard.putNumber("Gyro", driveGyro.getAngle());
		SmartDashboard.putNumber("Gyro_360", formatGyro360(driveGyro.getAngle()));
		SmartDashboard.putBoolean(" Reversed", reversed);
		SmartDashboard.putBoolean(" Lift at Bottom", liftLowerValue);
		SmartDashboard.putBoolean(" Lift at Top", liftUpperValue);
		SmartDashboard.putBoolean(" Picker Touching", pickLimitValue);
		SmartDashboard.putBoolean(" Slow Mode", speedToggle);
		SmartDashboard.putNumber("Ver ", verPos);
		SmartDashboard.putNumber("Hor ", horPos);
		
		SmartDashboard.putNumber("Picker Distance", pickEncoder.getDistance());
		SmartDashboard.putBoolean(" Picker UP", pickEncoder.getDirection());
		SmartDashboard.putNumber("Picker Get", pickEncoder.get());
		
	}
	
	/**
	 * Init IP camera
	 */
	private void IPcameraInit(){
		try{
    		NIVision.IMAQdxStartAcquisition(session);
    		//NIVision.Rect rect = new NIVision.Rect(10, 10, 100, 100);
    		}catch(Exception e){
    			
    		}
	}
	
	/**
	 * Starts both IP and USB camera
	 */
	private void camerasStart(){
		NIVision.IMAQdxGrab(session, frame, 1);
		// NIVision.imaqDrawShapeOnImage(frame, frame, rect,
		// DrawMode.DRAW_VALUE, ShapeMode.SHAPE_OVAL, 0.0f);
		USBcameraServer.setImage(frame);
		
	}
	
	/**
	 * Sets limit values with current state of limits
	 * IMPORTANT: LOWER LIMIT IS INVERTED
	 */
	private void limitValues(){
		liftUpperValue = liftUpperLimit.get();
    	pickLimitValue = !pickLimit.get();
    	liftLowerValue = !liftLowerLimit.get();
	}
	
	/**
	 * Converts raw gyro between 0 to 360
	 * @param rawGyro
	 * @return formatted gyro value
	 */
	
	private double formatGyro360(double rawGyro){
		while(rawGyro < 0.0){
			rawGyro += 360.0;
		}
		while(rawGyro > 360){
			rawGyro -= 360.0;
		}
		return rawGyro;
	}
}
