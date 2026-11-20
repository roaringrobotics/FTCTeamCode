package org.firstinspires.ftc.teamcode.RobotHardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Implementations.AndroidLog;

public class Shooter {
    private final DcMotorEx intakeMotor;
    private final DcMotorEx shooterMotor;
    //public final CRServo blueServo;
    //public final CRServo blackServo;
    public long lastTime;
    public double lastVelocity;
    private int[] velocityBuffer = {(1000), (1100), (1345)};
    private int allowedError = 200;
//    private Intake intake;
    public AndroidLog log;
    VoltageSensor voltSensor;


    int ballsLaunched = 0;
    boolean empty = false;

    enum ShooterState {
        IDLE,
        SPINNING_UP,
        SHOOTING
    }
    private ShooterState ShooterState;
    private ShooterState lastShooterState;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();


    public Shooter(HardwareMap hardwareMap, AndroidLog logIn) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter");
        //intake = in;
        ShooterState = ShooterState.IDLE;
        log = logIn;

        double voltage = 0.0;
        for (VoltageSensor sensor : hardwareMap.getAll(VoltageSensor.class)) {
            double v = sensor.getVoltage();
            if (v > 0) {
                voltage = v;
                voltSensor = sensor;
                break;
            }
        }

        // Initialize servos (use hardware names configured in your robot config)
        //blueServo = hardwareMap.get(CRServo.class, "blueServo");
        //blackServo = hardwareMap.get(CRServo.class, "blackServo");

        //Reverse bottom motor so both spin the same physical direction
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        // Default to BRAKE when power is zero
        intakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        shooterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        // Reset encoder counts first (optional but recommended)
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Switch to RUN_USING_ENCODER mode
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void updateKinematics(long currentTime) {
        lastTime = currentTime;
        lastVelocity = intakeMotor.getVelocity();
    }

    // velocity is set in ticks per second
    // GoBilda's 6000 RPM motor max ticks per second is 2800
    public void startShooterMotor(double velocity) {
        shooterMotor.setVelocity(velocity);
    }
    public void startShooterMotor(double velocity, AngleUnit unit) {
        shooterMotor.setVelocity(velocity, unit);
    }

    // velocity is set in ticks per second
    // GoBilda's 6000 RPM motor max ticks per second is 2800
    public void startIntakeMotor(double velocity) {
        intakeMotor.setVelocity(velocity);
    }
    public void startIntakeMotor(double velocity, AngleUnit unit) {
        intakeMotor.setVelocity(velocity, unit);
    }
    public void startShooter(double velocity) {
        intakeMotor.setVelocity(velocity);
        shooterMotor.setVelocity(velocity);
    }

    public double getShooterMotorVelocity() {
       return shooterMotor.getVelocity();
    }
    public double getIntakeMotorVelocity() {
        return intakeMotor.getVelocity();
    }

    public void stopShooterMotor() {
        intakeMotor.setVelocity(0.0);
        shooterMotor.setVelocity(0.0);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        intakeMotor.setZeroPowerBehavior(behavior);
        shooterMotor.setZeroPowerBehavior(behavior);
    }

    private void setRunMode(DcMotor.RunMode mode) {
        intakeMotor.setMode(mode);
        shooterMotor.setMode(mode);
    }


   // public void stopShootServos() {
   //     blueServo.setPower(0);
   //     blackServo.setPower(0);
    // }

   // public void startShootServos() {
   //     blueServo.setPower(-1);
   //     blackServo.setPower(1);
   // }
//    public void reverseShootServos() {
//        blueServo.setPower(1);
//        blackServo.setPower(-1);
//    }

    public void toggleShooterMotor65(boolean toggle) {
        if (toggle) {
            if (intakeMotor.getPower() != 0)  // motor running
                stopShooterMotor();
            else
                startShooterMotor(0.65);
        }
    }

    public void toggleShooterMotor60(boolean toggle) {
        if (toggle) {
            if (intakeMotor.getPower() != 0)  // motor running
                stopShooterMotor();
            else
                startShooterMotor(0.6);
        }
    }

    public void toggleShooterMotor55(boolean toggle) {
        if (toggle) {
            if (intakeMotor.getPower() != 0)  // motor running
                stopShooterMotor();
            else
                startShooterMotor(0.55);
        }
    }

    public void toggleShooterMotor50(boolean toggle) {
        if (toggle) {
            if (intakeMotor.getPower() != 0)  // motor running
                stopShooterMotor();
            else
                startShooterMotor(0.5);
        }
    }

    public void toggleShooterFeeders(boolean toggle) {
        if (toggle) {
            //if (blueServo.getPower() != 0) // feeders running
            //    stopShootServos();
            //else
            //    startShootServos();
        }
    }

    public double getShooterVelocity() {
        return intakeMotor.getVelocity();
    }

    int settleCountRequired = 15;
    int settleCount = 0;
    // continousShoot for teleop
    public void continousShoot(boolean buttonShort, boolean buttonMid, boolean buttonLong) {
        boolean allOff = !buttonShort && !buttonMid && !buttonLong;
        boolean anyOn = buttonShort || buttonMid || buttonLong;

        // Continuously update voltage reading
        double voltage = 0.0;
        double v = voltSensor.getVoltage();
        int settleError = 100;
        dashboardTelemetry.addData("Voltage", v);
        int targetVelocity = 0;
        if(buttonShort) {
            targetVelocity = velocityBuffer[0];
        } else if (buttonMid) {
            targetVelocity = velocityBuffer[1];
        } else if (buttonLong) {
            targetVelocity = velocityBuffer[2];
        }

        if (allOff || ballsLaunched >= 3 && !empty) {
            settleCount = 0;
            if (allOff) {
                ballsLaunched = 0;
                // eventually we will add logic to detect if there are balls in the shooter
                empty = false;
            } else {
                empty = true;

            }


            resetShooter();
            //log.d("Shooter", "Setting State: " + ShooterState);
            ShooterState = ShooterState.IDLE;

        } else if (anyOn) {
            double currentVel = intakeMotor.getVelocity();
            double currentVelDeg = intakeMotor.getVelocity(AngleUnit.DEGREES);
            dashboardTelemetry.addData("vel", currentVel);
            dashboardTelemetry.update();
//            log.d("Shooter", "-----------------------------------");
//            log.d("Shooter", "Velocity (rag): " + currentVel);
//            log.d("Shooter", "Velocity (deg): " + currentVelDeg);
//            log.d("Shooter", "Target Velocity: " + targetVelocity);
//            log.d("Shooter", "State: " + ShooterState.toString());
//            log.d("Shooter", "Balls Launched: " + ballsLaunched);
            if(ShooterState == ShooterState.IDLE) {
                startShooterMotor(targetVelocity);
                ShooterState = ShooterState.SPINNING_UP;
            }
            else if (ShooterState == ShooterState.SPINNING_UP && (targetVelocity - currentVel) > allowedError) {
                if (lastShooterState == ShooterState.SHOOTING) {
                    ballsLaunched++;
                    log.d("Shooter", "Balls Launched: " + ballsLaunched);
                    log.d("Shooter", "Target: " + targetVelocity);
                    log.d("Shooter", "Current Velocity: " + currentVel);
                    lastShooterState = ShooterState.SPINNING_UP;
                }
                settleCount = 0;
                startShooterMotor(targetVelocity);

            }
            else if (ShooterState == ShooterState.SPINNING_UP && (targetVelocity - currentVel) < allowedError) {


                // Check settle condition
//                int lowerEnd = (targetVelocity - settleError);
//                int upperEnd = (targetVelocity + settleError);
//                dashboardTelemetry.addData("Settle Count", settleCount);
//                if (currentVel > lowerEnd && currentVel < upperEnd) {
//                    settleCount++;
//                    if (settleCount >= settleCountRequired) {
//                        ShooterState = ShooterState.SHOOTING;
//                        startShootServos();
//                        startIntake();
//                    }
//                } else {
//                    settleCount = 0;
//                }

                ShooterState = ShooterState.SHOOTING;
                //startShootServos();
                //startIntake();
                startShooterMotor(targetVelocity);


            } else if (ShooterState == ShooterState.SHOOTING && (targetVelocity - currentVel) > allowedError) {
                startShooterMotor(targetVelocity);
                //stopIntake();
                //stopShootServos();
                settleCount = 0;
                lastShooterState = ShooterState.SHOOTING;
                ShooterState = ShooterState.SPINNING_UP;
            }
        }
    }

    // continousShoot for autonomous as we need different velocity values for autonomous
    public void continousShoot(int targetVelocity) {

        // Continuously update voltage reading
        double voltage = 0.0;
        double v = voltSensor.getVoltage();

        // I doubt this logic is necessary but it stays just incase
        if (ballsLaunched >= 3 && !empty) {

            ballsLaunched = 0;
            empty = true;
            // eventually we will add logic to detect if there are balls in the shooter

            resetShooter();
            ShooterState = ShooterState.IDLE;

        } else {
            empty = false;
            double currentVel = intakeMotor.getVelocity();
            dashboardTelemetry.addData("vel", currentVel);
            dashboardTelemetry.update();
            log.d("Shooter", "-----------------------------------");
            log.d("Shooter", "Velocity (rag): " + currentVel);
            log.d("Shooter", "Target Velocity: " + targetVelocity);
            log.d("Shooter", "State: " + ShooterState.toString());
            log.d("Shooter", "Balls Launched: " + ballsLaunched);
            if(ShooterState == ShooterState.IDLE) {
                startShooterMotor(targetVelocity);
                ShooterState = ShooterState.SPINNING_UP;
            }
            else if (ShooterState == ShooterState.SPINNING_UP && (targetVelocity - currentVel) > allowedError) {
                if (lastShooterState == ShooterState.SHOOTING) {
                    ballsLaunched++;
                    log.d("Shooter", "Balls Launched: " + ballsLaunched);
                    log.d("Shooter", "Target: " + targetVelocity);
                    log.d("Shooter", "Current Velocity: " + currentVel);
                    lastShooterState = ShooterState.SPINNING_UP;
                }
                settleCount = 0;
                startShooterMotor(targetVelocity);

            }
            else if (ShooterState == ShooterState.SPINNING_UP && (targetVelocity - currentVel) < allowedError) {
                ShooterState = ShooterState.SHOOTING;
                //startShootServos();
                //startIntake();
                startShooterMotor(targetVelocity);


            } else if (ShooterState == ShooterState.SHOOTING && (targetVelocity - currentVel) > allowedError) {
                startShooterMotor(targetVelocity);
                //stopIntake();
                //stopShootServos();
                settleCount = 0;
                lastShooterState = ShooterState.SHOOTING;
                ShooterState = ShooterState.SPINNING_UP;
            }
        }
    }
    public void singleShoot(boolean buttonShort, boolean buttonMid, boolean buttonLong) {
        boolean allOff = !buttonShort && !buttonMid && !buttonLong;
        boolean anyOn = buttonShort || buttonMid || buttonLong;

        double targetVelocity = 0.0;
        if(buttonShort) {
            targetVelocity = velocityBuffer[0];
        } else if (buttonMid) {
            targetVelocity = velocityBuffer[1];
        } else if (buttonLong) {
            targetVelocity = velocityBuffer[2];
        }

        if (allOff || ballsLaunched == 3 && !empty) {
            if (allOff) {
                ballsLaunched = 0;
                // eventually we will add logic to detect if there are balls in the shooter
                empty = false;
            } else {
                empty = true;

            }

            resetShooter();
            ShooterState = ShooterState.IDLE;

        } else if (anyOn) {
            double currentVel = intakeMotor.getVelocity();
            double currentVelDeg = intakeMotor.getVelocity(AngleUnit.DEGREES);
            log.d("Shooter", "-----------------------------------");
            log.d("Shooter", "Velocity (rag): " + currentVel);
            log.d("Shooter", "Velocity (deg): " + currentVelDeg);
            log.d("Shooter", "Target Velocity: " + targetVelocity);
            log.d("Shooter", "State: " + ShooterState.toString());
            log.d("Shooter", "Balls Launched: " + ballsLaunched);
            if(ShooterState == ShooterState.IDLE) {
                startShooterMotor(targetVelocity);
                ShooterState = ShooterState.SPINNING_UP;
            }
            else if (ShooterState == ShooterState.SPINNING_UP && (targetVelocity - currentVel) > allowedError) {
                if (lastShooterState == ShooterState.SHOOTING) {
                    ballsLaunched++;
                    lastShooterState = ShooterState.SPINNING_UP;
                }

                startShooterMotor(targetVelocity);
                //
            }
            else if (ShooterState == ShooterState.SPINNING_UP && (targetVelocity - currentVel) < allowedError) {
                ShooterState = ShooterState.SHOOTING;
                //startShootServos();
            } else if (ShooterState == ShooterState.SHOOTING && (targetVelocity - currentVel) > allowedError) {
                startShooterMotor(targetVelocity);
                //stopShootServos();
                lastShooterState = ShooterState.SHOOTING;
                ShooterState = ShooterState.SPINNING_UP;



            }
        }

    }


    //public void startIntake() {
    //        intake.startIntake();
    //}

    //public void stopIntake() {
    //        intake.stopIntake();
    //}

    //public void reverseIntake() {
    //        intake.reverseIntake();
    //}

    public int getBallsLaunched() {
        return ballsLaunched;
    }
    public void resetShooter() {
        ShooterState = ShooterState.IDLE;
        lastShooterState = ShooterState.IDLE;
        ballsLaunched = 0;
        //stopIntake();
        stopShooterMotor();
        //stopShootServos();
    }
    public void setPIDValues(double kp, double ki, double kd) {
        intakeMotor.setVelocityPIDFCoefficients(kp, ki, kd, 0);
        shooterMotor.setVelocityPIDFCoefficients(kp, ki, kd, 0);
    }
}

