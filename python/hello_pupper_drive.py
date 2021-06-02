import pupper_drive
#print(dir(pupper_drive))

drive = pupper_drive.DriveSystem()
drive.SetIdle()
drive.SetMaxCurrent(2.0)
drive.SetPositionKp(8.0)
drive.SetPositionKd(2.0)
drive.SetActivations([1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1])
#self.drive.SetActivations([1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0])
drive.SetMaxCurrent(0.0)

measured_positions = [0]*12
measured_velocities = [0]*12

drive.SetMeasuredPositions(measured_positions)
drive.SetMeasuredVelocities(measured_velocities)

drive.Update()
currents = drive.GetLastCommandedCurrents()
print("currents=",currents)          
