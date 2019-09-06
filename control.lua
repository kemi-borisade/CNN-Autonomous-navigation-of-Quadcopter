



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
    vel, oriV_curr= sim.getObjectVelocity(uav,-1)
    zV_c = vel[3] -- get current velocity in z direction

    kp_rot = 0.01
    kd_rot = 0.01
    ori_des = {(0)*3.14/180,(0)*3.14/180,(-90)*3.14/180}--turning degree of the quadcopter
    oriV_des = {0,0,0}

    T = mass_uav*(g + Kp*(z_t-z_c) + Kd*(z_s-zV_c)) --height control

    -- rotation control --
    rot_phi = kp_rot*(ori_des[1]-ori_curr[1]) + kd_rot*(oriV_des[1]-oriV_curr[1])    --angle along x axis of the quadrotor
    rot_alpha = kp_rot*(ori_des[2]-ori_curr[2]) + kd_rot*(oriV_des[2]-oriV_curr[2])  --angle along y axis of the quadrotor
    rot_psi = kp_rot*(ori_des[3]-ori_curr[3]) + kd_rot*(oriV_des[3]-oriV_curr[3])    --angle along z axis of the quadrotor
    tau = {rot_phi,rot_alpha,rot_psi} ---rotation
    velocity={0.3,0.3,0.3} -- set the velocity array for x y and z direction
    torque = {0,0,0} -- set the torque to 0 when the quadcopter does not need to rotate
    

end

function sysCall_actuation()  

    
    local t=sim.getSimulationTime() --get the simulation time


    --Actuate the Rotors--
    sim.setJointPosition(propellerJoint1,t*10)
    sim.setJointPosition(propellerJoint2,t*10)
    sim.setJointPosition(propellerJoint3,t*10)
    sim.setJointPosition(propellerJoint4,t*10)

    --Apply Torque  to the rotors of the propeller to increase altitude--
    sim.addForceAndTorque(propRespondable1, {0,0,T}, torque) 
    sim.addForceAndTorque(propRespondable2, {0,0,T}, torque)
    sim.addForceAndTorque(propRespondable3, {0,0,T}, torque)
    sim.addForceAndTorque(propRespondable4, {0,0,T}, torque)


    if ( t > 3) then
      local command=sim.getStringSignal("Comms") --start getting command from Remote API

      if (command == "forward") then
        sim.resetDynamicObject(16) --reset dynamic object
        sim.setObjectFloatParameter(16,3000,velocity[1]) --quadcopter moves forward 
      end

      if (command == "leftMove") then
        sim.resetDynamicObject(16)
        sim.setObjectFloatParameter(16,3001,velocity[2]) --quadcopter moves left
      end

      if (command == "rightMove") then
        sim.resetDynamicObject(16)
        sim.setObjectFloatParameter(16,3002,velocity[3]) --quadcopter moves right
      end

      if (command == "rotateRight") then
        sim.resetDynamicObject(16)
        sim.addForceAndTorque(propRespondable1,{0,0,T},tau)
        sim.addForceAndTorque(propRespondable2,{0,0,T},tau)
        sim.addForceAndTorque(propRespondable3,{0,0,T},tau)
        sim.addForceAndTorque(propRespondable4,{0,0,T},tau) --quadcopter rotate right
      end

      if (command == "stop") then
        sim.resetDynamicObject(16)
       sim.addForceAndTorque(propRespondable1, {0,0,0.2}, torque)
       sim.addForceAndTorque(propRespondable2, {0,0,0.2}, torque)
       sim.addForceAndTorque(propRespondable3, {0,0,0.2}, torque)
       sim.addForceAndTorque(propRespondable4, {0,0,0.2}, torque) --quadcopter stops and lands
     end
    end
 end

