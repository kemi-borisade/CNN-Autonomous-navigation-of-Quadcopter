



function sysCall_init()
    -- initialisation code is placed in here

    --get handle of quadcopter--
    uav = sim.getObjectAssociatedWithScript(sim.handle_self)
    handle = sim.getObjectHandle('Quadricopter')

    --get handle of quadcopter joints--
  
    propellerJoint1=sim.getObjectHandle('Quadricopter_propeller_joint1')
    propellerJoint2=sim.getObjectHandle('Quadricopter_propeller_joint2')
    propellerJoint3=sim.getObjectHandle('Quadricopter_propeller_joint3')
    propellerJoint4=sim.getObjectHandle('Quadricopter_propeller_joint4')

    --get handle of quadcopter respondables--

    propRespondable1 = sim.getObjectHandle('Quadricopter_propeller_respondable1')
    propRespondable2 = sim.getObjectHandle('Quadricopter_propeller_respondable2')
    propRespondable3 = sim.getObjectHandle('Quadricopter_propeller_respondable3')
    propRespondable4 = sim.getObjectHandle('Quadricopter_propeller_respondable4')



  

    -- set floating view display view of the camera--

    quadCamera = sim.getObjectHandle('screenshotSensor') --get the vision sensor handle
    floorCam=sim.getObjectHandle('Quadricopter_floorCamera')
    frontCam=sim.getObjectHandle('Quadricopter_frontCamera')
    floorView=sim.floatingViewAdd(0.9,0.9,0.2,0.2,0)
    frontView=sim.floatingViewAdd(0.7,0.9,0.2,0.2,0)
    sim.adjustView(floorView,floorCam,64)
    sim.adjustView(frontView,frontCam,64)


    matrix= sim.getObjectMatrix(uav,-1) 
    --set translational matrix to 0
    matrix[4] = 0 
    matrix[8] = 0
    matrix[12] = 0
    

    --get mass and gravity parameter--
    mass_uav = sim.getShapeMassAndInertia(uav)
    g = -sim.getArrayParameter(sim.arrayparam_gravity)[3]

    --PD control --
    z_t = 2 -- target height
    z_s = 0
    Kp = 10  --Proportional gain
    Kd = 10 -- Derivative gain
    z_c = sim.getObjectPosition(uav,-1)[3] 
    vel= sim.getObjectVelocity(uav,-1)
    zV_c = vel[3] -- get current velocity in z direction

    T = mass_uav*(g + Kp*(z_t-z_c) + Kd*(z_s-zV_c)) --height control
    velocity={0,0.3,0} -- set the velocity array for x y and z direction
    torque = {0,0,0} -- set the torque to 0 when the quadcopter does not need to rotate
    
end

function sysCall_actuation()  

    
    local t=sim.getSimulationTime() --get the simulation time


    --Actuate the Rotors--
    sim.setJointPosition(propellerJoint1,t*10)
    sim.setJointPosition(propellerJoint2,t*10)
    sim.setJointPosition(propellerJoint3,t*10)
    sim.setJointPosition(propellerJoint4,t*10)

    --Apply Torque  to the rotors of the propeller--
    sim.addForceAndTorque(propRespondable1, {0,0,T}, torque) 
    sim.addForceAndTorque(propRespondable2, {0,0,T}, torque)
    sim.addForceAndTorque(propRespondable3, {0,0,T}, torque)
    sim.addForceAndTorque(propRespondable4, {0,0,T}, torque)

  if ( t > 2.25 and t < 2.30) then
     
    sim.setObjectFloatParameter(16,3001,velocity[2]) -- the quacopter starts moving forward
    
    local options=0
    local cutOff=0
    if saveRgba then -- save the file as an Rgba file
      options=1
      cutOff=0.99
    end
    local image,resX,resY=sim.getVisionSensorCharImage(quadCamera,0,0,0,0,cutOff) -- get the vision sensor image and resolution 
    local filenameAndPath = ':\\H\rotRight-'.. i ..'.png'     
    sim.saveImage(image,{resX,resY},options,filenameAndPath,-1) --save the image with filename in the path

   end
   
end

    

    

    if (t > 40 and t < 157) then
        sim.resetDynamicObject(16)
       -- sim.setObjectFloatParameter(16,3001,0
        sim.setObjectFloatParameter(16,3000,velocity[1])
        print('now here')

    end