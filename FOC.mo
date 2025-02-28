package FOC
  model SMPM_Inverter "Test example: PermanentMagnetSynchronousMachine with inverter"
    extends Modelica.Icons.Example;
    constant Integer m = 3 "Number of phases";
    parameter Modelica.Units.SI.Voltage VNominal = 100 "Nominal RMS voltage per phase";
    parameter Modelica.Units.SI.Frequency fNominal = 50 "Nominal frequency";
    parameter Modelica.Units.SI.Frequency f = 50 "Actual frequency";
    parameter Modelica.Units.SI.Time tRamp = 1 "Frequency ramp";
    parameter Modelica.Units.SI.Torque TLoad = 181.4 "Nominal load torque";
    parameter Modelica.Units.SI.Time tStep = 1.2 "Time of load torque step";
    parameter Modelica.Units.SI.Inertia JLoad = 0.29 "Load's moment of inertia";
    Modelica.Electrical.Machines.BasicMachines.SynchronousMachines.SM_PermanentMagnet smpm(p = smpmData.p, fsNominal = smpmData.fsNominal, Rs = smpmData.Rs, TsRef = smpmData.TsRef, Lszero = smpmData.Lszero, Lssigma = smpmData.Lssigma, Jr = smpmData.Jr, Js = smpmData.Js, frictionParameters = smpmData.frictionParameters, phiMechanical(fixed = true), wMechanical(fixed = true), statorCoreParameters = smpmData.statorCoreParameters, strayLoadParameters = smpmData.strayLoadParameters, VsOpenCircuit = smpmData.VsOpenCircuit, Lmd = smpmData.Lmd, Lmq = smpmData.Lmq, useDamperCage = smpmData.useDamperCage, Lrsigmad = smpmData.Lrsigmad, Lrsigmaq = smpmData.Lrsigmaq, Rrd = smpmData.Rrd, Rrq = smpmData.Rrq, TrRef = smpmData.TrRef, permanentMagnetLossParameters = smpmData.permanentMagnetLossParameters, TsOperational = 293.15, alpha20s = smpmData.alpha20s, ir(each fixed = true), TrOperational = 293.15, alpha20r = smpmData.alpha20r) annotation(
      Placement(transformation(extent = {{-20, -50}, {0, -30}})));
    Modelica.Electrical.Machines.Sensors.CurrentQuasiRMSSensor currentQuasiRMSSensor annotation(
      Placement(transformation(origin = {0, 30}, extent = {{-10, 10}, {10, -10}}, rotation = 270)));
    Modelica.Electrical.Machines.Sensors.RotorDisplacementAngle rotorDisplacementAngle(p = smpm.p) annotation(
      Placement(transformation(origin = {20, -40}, extent = {{-10, 10}, {10, -10}}, rotation = 270)));
    Modelica.Blocks.Sources.Ramp ramp(height = f, duration = tRamp) annotation(
      Placement(transformation(extent = {{-80, 50}, {-60, 70}})));
    Modelica.Electrical.Machines.Utilities.VfController vfController(final m = m, VNominal = VNominal, fNominal = fNominal, BasePhase = +Modelica.Constants.pi/2) annotation(
      Placement(transformation(extent = {{-40, 50}, {-20, 70}})));
    Modelica.Electrical.Polyphase.Sources.SignalVoltage signalVoltage(final m = m) annotation(
      Placement(transformation(origin = {0, 60}, extent = {{10, 10}, {-10, -10}}, rotation = 270)));
    Modelica.Electrical.Polyphase.Basic.Star star(final m = m) annotation(
      Placement(transformation(extent = {{-50, 80}, {-70, 100}})));
    Modelica.Electrical.Analog.Basic.Ground ground annotation(
      Placement(transformation(origin = {-90, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 270)));
    Modelica.Mechanics.Rotational.Components.Inertia loadInertia(J = JLoad) annotation(
      Placement(transformation(extent = {{40, -50}, {60, -30}})));
    Modelica.Mechanics.Rotational.Sources.TorqueStep loadTorqueStep(startTime = tStep, stepTorque = -TLoad, useSupport = false, offsetTorque = 0) annotation(
      Placement(transformation(extent = {{90, -50}, {70, -30}})));
    Modelica.Electrical.Machines.Utilities.TerminalBox terminalBox(terminalConnection = "Y") annotation(
      Placement(transformation(extent = {{-20, -34}, {0, -14}})));
    parameter Modelica.Electrical.Machines.Utilities.ParameterRecords.SM_PermanentMagnetData smpmData "Synchronous machine data" annotation(
      Placement(transformation(extent = {{-20, -80}, {0, -60}})));
  initial equation
    smpm.is[1:2] = zeros(2);
//conditional damper cage currents are defined as fixed start values
  equation
    connect(signalVoltage.plug_n, star.plug_p) annotation(
      Line(points = {{0, 70}, {0, 90}, {-50, 90}}, color = {0, 0, 255}));
    connect(star.pin_n, ground.p) annotation(
      Line(points = {{-70, 90}, {-80, 90}}, color = {0, 0, 255}));
    connect(ramp.y, vfController.u) annotation(
      Line(points = {{-59, 60}, {-42, 60}}, color = {0, 0, 255}));
    connect(vfController.y, signalVoltage.v) annotation(
      Line(points = {{-19, 60}, {-12, 60}}, color = {0, 0, 255}));
    connect(loadInertia.flange_b, loadTorqueStep.flange) annotation(
      Line(points = {{60, -40}, {70, -40}}));
    connect(signalVoltage.plug_p, currentQuasiRMSSensor.plug_p) annotation(
      Line(points = {{0, 50}, {0, 50}, {0, 40}}, color = {0, 0, 255}));
    connect(rotorDisplacementAngle.plug_n, smpm.plug_sn) annotation(
      Line(points = {{26, -30}, {26, -20}, {-16, -20}, {-16, -30}}, color = {0, 0, 255}));
    connect(rotorDisplacementAngle.plug_p, smpm.plug_sp) annotation(
      Line(points = {{14, -30}, {-4, -30}}, color = {0, 0, 255}));
    connect(terminalBox.plugSupply, currentQuasiRMSSensor.plug_n) annotation(
      Line(points = {{-10, -28}, {-10, 0}, {0, 0}, {0, 20}}, color = {0, 0, 255}));
    connect(terminalBox.plug_sn, smpm.plug_sn) annotation(
      Line(points = {{-16, -30}, {-16, -30}}, color = {0, 0, 255}));
    connect(terminalBox.plug_sp, smpm.plug_sp) annotation(
      Line(points = {{-4, -30}, {-4, -30}}, color = {0, 0, 255}));
    connect(smpm.flange, rotorDisplacementAngle.flange) annotation(
      Line(points = {{0, -40}, {10, -40}}));
    connect(smpm.flange, loadInertia.flange_a) annotation(
      Line(points = {{0, -40}, {40, -40}}));
    annotation(
      experiment(StopTime = 1.5, Interval = 1E-4, Tolerance = 1e-06),
      Documentation(info = "<html>
  <p>An ideal frequency inverter is modeled by using a VfController and a three-phase SignalVoltage.
  Frequency is raised by a ramp, causing the permanent magnet synchronous machine to start,
  and accelerating inertias. At time tStep a load step is applied.</p>
  
  <p>Simulate for 1.5 seconds and plot (versus time):</p>
  
  <ul>
  <li>currentQuasiRMSSensor.I: stator current RMS</li>
  <li>smpm.wMechanical: motor's speed</li>
  <li>smpm.tauElectrical: motor's torque</li>
  <li>rotorDisplacementAngle.rotorDisplacementAngle: rotor displacement angle</li>
  </ul>
  
  <p>Default machine parameters are used.</p>
  
  <p>
  In practice it is nearly impossible to drive a PMSMD without current controller.</p>
  </html>"));
  end SMPM_Inverter;

  model PMSMdiagram
    import Modelica.Electrical.Polyphase.Functions.factorY2DC;
    Modelica.Electrical.Analog.Basic.Ground ground annotation(
      Placement(transformation(origin = {-80, -92}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Electrical.Polyphase.Basic.Star star annotation(
      Placement(transformation(origin = {-80, -58}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
    Modelica.Electrical.Polyphase.Sources.SineVoltage sineVoltage(V = fill(400*sqrt(2/3), 3), f = fill(50, 3), phase = -Modelica.Electrical.Polyphase.Functions.symmetricOrientation(3), m = 3, offset = zeros(3), startTime = zeros(3)) annotation(
      Placement(transformation(origin = {-80, -24}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));
    Modelica.Electrical.Polyphase.Basic.Resistor resistor(R = fill(10e-3, 3), alpha = zeros(3), T = fill(20, 3), m = 3, T_ref = fill(20, 3)) annotation(
      Placement(transformation(origin = {-80, 38}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
    Modelica.Electrical.Polyphase.Basic.Inductor inductor(L = fill(500e-6, 3), m = 3) annotation(
      Placement(transformation(origin = {-80, 64}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
    Modelica.Electrical.PowerConverters.ACDC.DiodeBridge2mPulse rectifier annotation(
      Placement(transformation(origin = {-56, 82}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Electrical.Analog.Basic.Capacitor capacitor(C = 5e-3) annotation(
      Placement(transformation(origin = {-34, 82}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
    Modelica.Electrical.PowerConverters.DCAC.Polyphase2Level inverter annotation(
      Placement(transformation(origin = {-8, 82}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Electrical.PowerConverters.DCAC.Control.PWM pwm(f = 2000, uMax = factorY2DC(3)*400/sqrt(3)) annotation(
      Placement(transformation(origin = {-8, 46}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
    Modelica.Electrical.Machines.BasicMachines.SynchronousMachines.SM_PermanentMagnet smpm(p = 3, fsNominal = 50, Jr = 0.29, TsOperational = 293.15, TrOperational = 293.15, VsOpenCircuit = 112.3, Rs = 0.03, TsRef = 566.3, alpha20s = 0, Lssigma = 0.1/(2*3.1415*50), Lmd = 0.3/(2*3.1415*50), Lmq = 0.3/(2*3.1415*50), useDamperCage = false) annotation(
      Placement(transformation(origin = {32, -60}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Electrical.Machines.Utilities.TerminalBox terminalBox(terminalConnection = "Y") annotation(
      Placement(transformation(origin = {32, -44}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.Rotational.Components.Inertia inertia(J = 0.29) annotation(
      Placement(transformation(origin = {62, -60}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.Rotational.Sources.QuadraticSpeedDependentTorque quadraticSpeedDependentTorque(tau_nominal = -161.4, TorqueDirection = false, w_nominal = 1440.45*2*3.14/60) annotation(
      Placement(transformation(origin = {88, -60}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));
    Modelica.Electrical.Machines.Sensors.CurrentQuasiRMSSensor currentRMSSensor1 annotation(
      Placement(transformation(origin = {-80, 6}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));
    Modelica.Electrical.Machines.Sensors.SinCosResolver sinCosResolver(p = 3) annotation(
      Placement(transformation(origin = {76, -24}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));
    Modelica.Electrical.Machines.Utilities.SinCosEvaluation sinCosEvaluation annotation(
      Placement(transformation(origin = {76, 8}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
    CurrentController currentController(p = 3, fsNominal = smpm.fsNominal, Rs = Modelica.Electrical.Machines.Thermal.convertResistance(smpm.Rs, smpm.TsRef, smpm.alpha20s, smpm.TsOperational), Ld = smpm.Lssigma + smpm.Lmd, Lq = smpm.Lssigma + smpm.Lmq, VsOpenCircuit = smpm.VsOpenCircuit, decoupling = true, useRMS = true) annotation(
      Placement(transformation(origin = {-8, 8}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
    Modelica.Blocks.Sources.Constant id_ref(k = -53.5) annotation(
      Placement(transformation(origin = {-40, -24}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Blocks.Sources.Constant iq_ref(k = 84.6) annotation(
      Placement(transformation(origin = {-42, -64}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Electrical.Polyphase.Sensors.CurrentSensor currentSensor annotation(
      Placement(transformation(origin = {32, 2}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
    Modelica.Electrical.Machines.Sensors.RotorDisplacementAngle angleSensor(p = 3) annotation(
      Placement(transformation(origin = {48, -28}, extent = {{10, -10}, {-10, 10}}, rotation = -180)));
  equation
    connect(star.pin_n, ground.p) annotation(
      Line(points = {{-80, -68}, {-80, -82}}, color = {0, 0, 255}));
    connect(star.plug_p, sineVoltage.plug_n) annotation(
      Line(points = {{-80, -48}, {-80, -34}}, color = {0, 0, 255}));
    connect(inductor.plug_n, resistor.plug_p) annotation(
      Line(points = {{-80, 54}, {-80, 48}}, color = {0, 0, 255}));
    connect(inductor.plug_p, rectifier.ac) annotation(
      Line(points = {{-80, 74}, {-80, 82}, {-66, 82}}, color = {0, 0, 255}));
    connect(pwm.fire_p, inverter.fire_p) annotation(
      Line(points = {{-14, 57}, {-14, 69}}, color = {255, 0, 255}, thickness = 0.5));
    connect(pwm.fire_n, inverter.fire_n) annotation(
      Line(points = {{-2, 57}, {-2, 69}}, color = {255, 0, 255}, thickness = 0.5));
    connect(terminalBox.plug_sp, smpm.plug_sp) annotation(
      Line(points = {{38, -50}, {38, -50}}, color = {0, 0, 255}));
    connect(terminalBox.plug_sn, smpm.plug_sn) annotation(
      Line(points = {{26, -50}, {26, -50}}, color = {0, 0, 255}));
    connect(rectifier.dc_p, capacitor.p) annotation(
      Line(points = {{-46, 88}, {-40, 88}, {-40, 92}, {-34, 92}}, color = {0, 0, 255}));
    connect(rectifier.dc_n, capacitor.n) annotation(
      Line(points = {{-46, 76}, {-42, 76}, {-42, 72}, {-34, 72}}, color = {0, 0, 255}));
    connect(capacitor.n, inverter.dc_n) annotation(
      Line(points = {{-34, 72}, {-26, 72}, {-26, 76}, {-18, 76}}, color = {0, 0, 255}));
    connect(capacitor.p, inverter.dc_p) annotation(
      Line(points = {{-34, 92}, {-24, 92}, {-24, 88}, {-18, 88}}, color = {0, 0, 255}));
    connect(smpm.flange, inertia.flange_a) annotation(
      Line(points = {{42, -60}, {52, -60}}));
    connect(inertia.flange_b, quadraticSpeedDependentTorque.flange) annotation(
      Line(points = {{72, -60}, {78, -60}}));
    connect(currentRMSSensor1.plug_n, sineVoltage.plug_p) annotation(
      Line(points = {{-80, -4}, {-80, -14}}, color = {0, 0, 255}));
    connect(resistor.plug_n, currentRMSSensor1.plug_p) annotation(
      Line(points = {{-80, 28}, {-80, 16}}, color = {0, 0, 255}));
    connect(sinCosResolver.flange, smpm.flange) annotation(
      Line(points = {{76, -34}, {76, -48}, {46, -48}, {46, -60}, {42, -60}}));
    connect(sinCosResolver.y, sinCosEvaluation.u) annotation(
      Line(points = {{76, -13}, {76, -5}}, color = {0, 0, 127}, thickness = 0.5));
    connect(id_ref.y, currentController.id) annotation(
      Line(points = {{-28, -24}, {-14, -24}, {-14, -4}}, color = {0, 0, 127}));
    connect(iq_ref.y, currentController.iq) annotation(
      Line(points = {{-30, -64}, {-2, -64}, {-2, -4}}, color = {0, 0, 127}));
    connect(currentSensor.plug_p, inverter.ac) annotation(
      Line(points = {{32, 12}, {32, 82}, {2, 82}}, color = {0, 0, 255}));
    connect(currentSensor.i, currentController.iActual) annotation(
      Line(points = {{21, 2}, {4, 2}}, color = {0, 0, 127}, thickness = 0.5));
    connect(angleSensor.plug_p, terminalBox.plug_sp) annotation(
      Line(points = {{38, -34}, {38, -50}}, color = {0, 0, 255}));
    connect(angleSensor.plug_n, terminalBox.plug_sn) annotation(
      Line(points = {{38, -22}, {26, -22}, {26, -50}}, color = {0, 0, 255}));
    connect(angleSensor.flange, smpm.flange) annotation(
      Line(points = {{48, -38}, {48, -60}, {42, -60}}));
    connect(sinCosEvaluation.phi, currentController.phi) annotation(
      Line(points = {{76, 20}, {76, 32}, {18, 32}, {18, 14}, {4, 14}}, color = {0, 0, 127}));
    connect(currentController.y, pwm.u) annotation(
      Line(points = {{-8, 20}, {-8, 34}}, color = {0, 0, 127}, thickness = 0.5));
    connect(terminalBox.plugSupply, currentSensor.plug_n) annotation(
      Line(points = {{32, -48}, {32, -8}}, color = {0, 0, 255}));
    annotation(
      Diagram);
  end PMSMdiagram;

  model PWM1 "Test of pulse width modulation methods"
    extends Modelica.Icons.Example;
    import Modelica.Electrical.Polyphase.Functions.factorY2DC;
    import Modelica.Constants.pi;
    parameter Real RMS = 1 "Reference RMS Y";
    Modelica.Electrical.PowerConverters.DCAC.Polyphase2Level multiPhase2Level(m = 3) annotation(
      Placement(transformation(extent = {{-10, 40}, {10, 60}})));
    Modelica.Electrical.Analog.Sources.ConstantVoltage dcPos(V = 200) annotation(
      Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 270, origin = {-40, 70})));
    Modelica.Electrical.Analog.Basic.Ground ground annotation(
      Placement(transformation(origin = {-40, 24}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Electrical.Polyphase.Sensors.CurrentSensor currentSensor annotation(
      Placement(transformation(origin = {26, 50}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Electrical.Machines.Utilities.TerminalBox terminalBox(terminalConnection = "Y") annotation(
      Placement(transformation(origin = {78, 32}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Electrical.Machines.BasicMachines.SynchronousMachines.SM_PermanentMagnet smpm(p = smpmData.p, fsNominal = smpmData.fsNominal, TsOperational = 293.15, Rs = smpmData.Rs, TsRef = smpmData.TsRef, alpha20s = smpmData.alpha20s, Lssigma = smpmData.Lssigma, Jr = smpmData.Jr, VsOpenCircuit = smpmData.VsOpenCircuit, Lmd = smpmData.Lmd, Lmq = smpmData.Lmq, useDamperCage = smpmData.useDamperCage, useSupport = false, useThermalPort = false, phiMechanical(start = 0, fixed = true), wMechanical(start = 0, fixed = true), Lszero = smpmData.Lszero) annotation(
      Placement(transformation(origin = {78, 8}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Electrical.Machines.Sensors.SinCosResolver sinCosResolver(p = 3) annotation(
      Placement(transformation(origin = {140, -2}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
    Modelica.Electrical.Machines.Utilities.SinCosEvaluation sinCosEvaluation annotation(
      Placement(transformation(origin = {140, -32}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
    Modelica.Mechanics.Rotational.Components.Inertia inertia(J = 0.29) annotation(
      Placement(transformation(origin = {178, 8}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Electrical.Machines.Utilities.ParameterRecords.SM_PermanentMagnetData smpmData(useDamperCage = false, p = 3) annotation(
      Placement(transformation(origin = {-74, -16}, extent = {{-20, -80}, {0, -60}})));
    SVPWMcustom pwm(Udc = 200, f = 2500) annotation(
      Placement(transformation(origin = {0, 8}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
    Modelica.Electrical.Machines.Utilities.VfController vfController(VNominal = 200, fNominal = 50, EconomyMode = true) annotation(
      Placement(transformation(origin = {0, -56}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
    Modelica.Blocks.Sources.Ramp ramp(height = 50, duration = 0.5) annotation(
      Placement(transformation(origin = {0, -86}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  equation
    connect(dcPos.p, multiPhase2Level.dc_p) annotation(
      Line(points = {{-40, 80}, {-20, 80}, {-20, 56}, {-10, 56}}, color = {0, 0, 255}));
    connect(dcPos.n, ground.p) annotation(
      Line(points = {{-40, 60}, {-40, 34}}, color = {0, 0, 255}));
    connect(multiPhase2Level.ac, currentSensor.plug_p) annotation(
      Line(points = {{10, 50}, {16, 50}}, color = {0, 0, 255}));
    connect(currentSensor.plug_n, terminalBox.plugSupply) annotation(
      Line(points = {{36, 50}, {78, 50}, {78, 28}}, color = {0, 0, 255}));
    connect(smpm.plug_sn, terminalBox.plug_sn) annotation(
      Line(points = {{72, 18}, {72, 26}}, color = {0, 0, 255}));
    connect(smpm.plug_sp, terminalBox.plug_sp) annotation(
      Line(points = {{84, 18}, {84, 26}}, color = {0, 0, 255}));
    connect(sinCosResolver.flange, smpm.flange) annotation(
      Line(points = {{140, 8}, {88, 8}}));
    connect(sinCosResolver.y, sinCosEvaluation.u) annotation(
      Line(points = {{140, -13}, {140, -20}}, color = {0, 0, 127}, thickness = 0.5));
    connect(sinCosResolver.flange, inertia.flange_a) annotation(
      Line(points = {{140, 8}, {168, 8}}));
    connect(multiPhase2Level.dc_n, ground.p) annotation(
      Line(points = {{-10, 44}, {-40, 44}, {-40, 34}}, color = {0, 0, 255}));
    connect(pwm.fire_p, multiPhase2Level.fire_p) annotation(
      Line(points = {{-6, 20}, {-6, 38}}, color = {255, 0, 255}, thickness = 0.5));
    connect(pwm.fire_n, multiPhase2Level.fire_n) annotation(
      Line(points = {{6, 20}, {6, 38}}, color = {255, 0, 255}, thickness = 0.5));
    connect(ramp.y, vfController.u) annotation(
      Line(points = {{0, -75}, {0, -69}}, color = {0, 0, 127}));
    connect(vfController.y, pwm.u) annotation(
      Line(points = {{0, -44}, {0, -4}}, color = {0, 0, 127}, thickness = 0.5));
    annotation(
      experiment(StopTime = 2, Interval = 0.001, Tolerance = 1e-05),
      Documentation(info = "<html>
<p>
A reference space vector (formed by real part = cosine and imaginary part = sine) of length &radic;2*RMS and frequency 2 Hz is applied.
The resulting switching patterns are applied to a three-phase twolevel bridge with switching frequency 100 Hz, fed by DC voltage = &radic;2*&radic;3*1
where 1 is the theoretical maximum voltage from terminal to neutral.
The resulting voltages with reference to midpoint of the DC voltage are measured.
</p>
<p>
The RMS of the first harmonic of the first of these voltages is calculated.
Please note that the value of the first harmonic is valid after the first period (i.e. 0.5 s).
</p>
<p>
Furthermore, these three voltages are transformed to the corresponding space phasor.
Note that the zero component is not zero, indicating the shift of the neutral with respect to the midpoint of the DC voltage.
</p>
<p>
The space phasor is rotated to the coordinate system rotating with 2*&pi;*2 Hz.
To suppress the influence of switching, real and imaginary part of the rotated phasor are filtered.
The polar representation of this rotated and filtered phasor are calculated.
</p>
<p>
Please note that the filter has a settle time depending on the filter parameters.
</p>
</html>"),
      Diagram);
  end PWM1;

  model CurrentController "Current controller in dq coordinate system"
    import Modelica.Constants.pi;
    constant Integer m = 3 "Number of phases";
    parameter Integer p = 3 "Number of pole pairs";
    parameter Boolean useRMS = true "If true, inputs dq are multiplied by sqrt(2)";
    parameter Modelica.Units.SI.Frequency fsNominal "Nominal frequency";
    parameter Modelica.Units.SI.Voltage VsOpenCircuit "Open circuit RMS voltage per phase @ fsNominal";
    parameter Modelica.Units.SI.Resistance Rs "Stator resistance per phase";
    parameter Modelica.Units.SI.Inductance Ld "Inductance in d-axis";
    parameter Modelica.Units.SI.Inductance Lq "Inductance in q-axis";
    //Decoupling
    parameter Boolean decoupling = false "Use decoupling network";
    final parameter Modelica.Units.SI.MagneticFlux psiM = sqrt(2)*VsOpenCircuit/(2*pi*fsNominal) "Approximation of magnetic flux linkage";
    Modelica.Units.SI.AngularVelocity omega = p*der(phi);
    Modelica.Units.SI.Voltage Vd = sqrt(2)*(Rs*id - omega*Lq*iq);
    Modelica.Units.SI.Voltage Vq = sqrt(2)*(Rs*iq + omega*Ld*id) + omega*psiM;
    extends Modelica.Blocks.Interfaces.MO(final nout = 3);
    Modelica.Blocks.Interfaces.RealOutput y[2] annotation(
      Placement(transformation(extent = {{100, -10}, {120, 10}}), iconTransformation(extent = {{100, -10}, {120, 10}})));
    Modelica.Blocks.Interfaces.RealInput id annotation(
      Placement(transformation(extent = {{-140, 40}, {-100, 80}}), iconTransformation(extent = {{-140, 40}, {-100, 80}})));
    Modelica.Blocks.Interfaces.RealInput iq annotation(
      Placement(transformation(extent = {{-140, -80}, {-100, -40}}), iconTransformation(extent = {{-140, -80}, {-100, -40}})));
    Modelica.Blocks.Interfaces.RealInput phi(unit = "rad") annotation(
      Placement(transformation(origin = {60, -120}, extent = {{20, -20}, {-20, 20}}, rotation = 270), iconTransformation(origin = {60, -120}, extent = {{20, -20}, {-20, 20}}, rotation = 270)));
    Modelica.Blocks.Interfaces.RealInput iActual[m](each unit = "A") annotation(
      Placement(transformation(origin = {-60, -120}, extent = {{20, -20}, {-20, 20}}, rotation = 270), iconTransformation(origin = {-60, -120}, extent = {{20, -20}, {-20, 20}}, rotation = 270)));
    Modelica.Blocks.Math.Gain toPeak_d(final k = if useRMS then sqrt(2) else 1) annotation(
      Placement(transformation(origin = {-70, 60}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Blocks.Math.Gain toPeak_q(final k = if useRMS then sqrt(2) else 1) annotation(
      Placement(transformation(origin = {-70, 0}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Blocks.Math.Feedback feedback_d annotation(
      Placement(transformation(extent = {{-38, 50}, {-18, 70}})));
    Modelica.Blocks.Math.Feedback feedback_q annotation(
      Placement(transformation(extent = {{-40, -10}, {-20, 10}})));
    Modelica.Blocks.Math.Add add[2](final k1 = fill(+1, 2), final k2 = fill(if decoupling then +1 else 0, 2)) annotation(
      Placement(transformation(extent = {{32, -10}, {52, 10}})));
    Modelica.Blocks.Sources.RealExpression deCoupling[2](y = {Vd, Vq}) annotation(
      Placement(transformation(extent = {{-10, -40}, {10, -20}})));
    Transformation transformation(m = 3, p = 1) annotation(
      Placement(transformation(origin = {-60, -54}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
    Modelica.Blocks.Discrete.TransferFunction PI_d(b = {0, 0.003}, a = {1, -1}, samplePeriod = 0.0001) annotation(
      Placement(transformation(origin = {8, 60}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Blocks.Discrete.TransferFunction PI_q(b = {0, 0.003}, a = {1, -1}, samplePeriod = 0.0001, x(each start = 0)) annotation(
      Placement(transformation(origin = {2, 0}, extent = {{-10, -10}, {10, 10}})));
    InverseTransformation inverse annotation(
      Placement(transformation(origin = {84, 0}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Blocks.Nonlinear.Limiter limiter(uMax = VsOpenCircuit) annotation(
      Placement(transformation(origin = {64, 44}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Blocks.Nonlinear.Limiter limiter2(uMax = VsOpenCircuit) annotation(
      Placement(transformation(origin = {62, -34}, extent = {{-10, -10}, {10, 10}})));
    Antiwindup antiwindup annotation(
      Placement(transformation(origin = {-22, 90}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Blocks.Sources.Constant const(k = VsOpenCircuit) annotation(
      Placement(transformation(origin = {-70, 90}, extent = {{-10, -10}, {10, 10}})));
    Antiwindup antiwindup1 annotation(
      Placement(transformation(origin = {-20, -68}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Blocks.Sources.Constant const1(k = VsOpenCircuit) annotation(
      Placement(transformation(origin = {-80, -82}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Blocks.Discrete.UnitDelay unitDelay(samplePeriod = 0.0001) annotation(
      Placement(transformation(origin = {60, 80}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));
    Modelica.Blocks.Discrete.UnitDelay unitDelay1(samplePeriod = 0.0001) annotation(
      Placement(transformation(origin = {30, -86}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));
  protected
    constant Modelica.Units.SI.Resistance unitResistance = 1 annotation(
      Placement(visible = false, transformation(extent = {{0, 0}, {0, 0}})));
  equation
    connect(toPeak_d.u, id) annotation(
      Line(points = {{-82, 60}, {-120, 60}}, color = {0, 0, 127}));
    connect(toPeak_q.u, iq) annotation(
      Line(points = {{-82, 0}, {-90, 0}, {-90, -60}, {-120, -60}}, color = {0, 0, 127}));
    connect(toPeak_q.y, feedback_q.u1) annotation(
      Line(points = {{-59, 0}, {-38, 0}}, color = {0, 0, 127}));
    connect(toPeak_d.y, feedback_d.u1) annotation(
      Line(points = {{-59, 60}, {-36, 60}}, color = {0, 0, 127}));
    connect(deCoupling.y, add.u2) annotation(
      Line(points = {{11, -30}, {20, -30}, {20, -6}, {30, -6}}, color = {0, 0, 127}));
    connect(transformation.u, iActual) annotation(
      Line(points = {{-60, -66}, {-60, -120}}, color = {0, 0, 127}, thickness = 0.5));
    connect(transformation.y[1], feedback_d.u2) annotation(
      Line(points = {{-60, -43}, {-60, -24}, {-48, -24}, {-48, 36}, {-28, 36}, {-28, 52}}, color = {0, 0, 127}));
    connect(transformation.y[2], feedback_q.u2) annotation(
      Line(points = {{-60, -43}, {-60, -34}, {-30, -34}, {-30, -8}}, color = {0, 0, 127}));
    connect(phi, transformation.angle) annotation(
      Line(points = {{60, -120}, {60, -54}, {-48, -54}}, color = {0, 0, 127}));
    connect(inverse.y, y) annotation(
      Line(points = {{95, 0}, {110, 0}}, color = {0, 0, 127}, thickness = 0.5));
    connect(inverse.angle, phi) annotation(
      Line(points = {{84, -12}, {84, -54}, {60, -54}, {60, -120}}, color = {0, 0, 127}));
    connect(limiter.y, inverse.u[1]) annotation(
      Line(points = {{76, 44}, {76, 20}, {64, 20}, {64, 0}, {72, 0}}, color = {0, 0, 127}));
    connect(limiter2.y, inverse.u[2]) annotation(
      Line(points = {{74, -34}, {76, -34}, {76, -12}, {64, -12}, {64, 0}, {72, 0}}, color = {0, 0, 127}));
    connect(add[1].y, limiter.u) annotation(
      Line(points = {{54, 0}, {54, 24}, {44, 24}, {44, 44}, {52, 44}}, color = {0, 0, 127}));
    connect(limiter2.u, add[2].y) annotation(
      Line(points = {{50, -34}, {44, -34}, {44, -16}, {54, -16}, {54, 0}}, color = {0, 0, 127}));
    connect(feedback_d.y, antiwindup.error) annotation(
      Line(points = {{-18, 60}, {-16, 60}, {-16, 74}, {-44, 74}, {-44, 84}, {-32, 84}}, color = {0, 0, 127}));
    connect(antiwindup.y, PI_d.u) annotation(
      Line(points = {{-12, 90}, {-10, 90}, {-10, 60}, {-4, 60}}, color = {0, 0, 127}));
    connect(const.y, antiwindup.saturation) annotation(
      Line(points = {{-58, 90}, {-32, 90}}, color = {0, 0, 127}));
    connect(antiwindup1.y, PI_q.u) annotation(
      Line(points = {{-10, -68}, {-16, -68}, {-16, 0}, {-10, 0}}, color = {0, 0, 127}));
    connect(feedback_q.y, antiwindup1.error) annotation(
      Line(points = {{-20, 0}, {-26, 0}, {-26, -46}, {-44, -46}, {-44, -74}, {-30, -74}}, color = {0, 0, 127}));
    connect(const1.y, antiwindup1.saturation) annotation(
      Line(points = {{-68, -82}, {-48, -82}, {-48, -68}, {-30, -68}}, color = {0, 0, 127}));
    connect(PI_d.y, add[1].u1) annotation(
      Line(points = {{20, 60}, {22, 60}, {22, 6}, {30, 6}}, color = {0, 0, 127}));
    connect(PI_q.y, add[2].u1) annotation(
      Line(points = {{14, 0}, {22, 0}, {22, 6}, {30, 6}}, color = {0, 0, 127}));
    connect(unitDelay.u, y[1]) annotation(
      Line(points = {{72, 80}, {110, 80}, {110, 0}}, color = {0, 0, 127}));
    connect(unitDelay1.u, y[2]) annotation(
      Line(points = {{42, -86}, {110, -86}, {110, 0}}, color = {0, 0, 127}));
    connect(unitDelay.y, antiwindup.signal) annotation(
      Line(points = {{50, 80}, {6, 80}, {6, 96}, {-32, 96}}, color = {0, 0, 127}));
    connect(unitDelay1.y, antiwindup1.signal) annotation(
      Line(points = {{20, -86}, {8, -86}, {8, -62}, {-30, -62}}, color = {0, 0, 127}));
    annotation(
      Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}})),
      Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}})));
  end CurrentController;

  model SMPM_VoltagePWM "Test example: PermanentMagnetSynchronousMachine fed by FOC"
    extends Modelica.Icons.Example;
    import Modelica.Constants.pi;
    constant Integer m = 3 "Number of phases";
    parameter Modelica.Units.SI.Current Idq[2] = {0, 84.6} "Desired d- and q-current";
    parameter Modelica.Units.SI.AngularVelocity wNominal = 2*pi*smpmData.fsNominal/smpmData.p "Nominal speed";
    parameter Modelica.Units.SI.Torque TLoad = 181.4 "Nominal load torque";
    parameter Modelica.Units.SI.Inertia JLoad = 0.29 "Load's moment of inertia";
    Modelica.Electrical.Machines.BasicMachines.SynchronousMachines.SM_PermanentMagnet smpm(phiMechanical(start = 0, fixed = true), wMechanical(start = 0, fixed = true), useSupport = false, useThermalPort = false, p = smpmData.p, fsNominal = smpmData.fsNominal, Rs = smpmData.Rs, TsRef = smpmData.TsRef, Lszero = smpmData.Lszero, Lssigma = smpmData.Lssigma, Jr = smpmData.Jr, Js = smpmData.Js, frictionParameters = smpmData.frictionParameters, statorCoreParameters = smpmData.statorCoreParameters, strayLoadParameters = smpmData.strayLoadParameters, VsOpenCircuit = smpmData.VsOpenCircuit, Lmd = smpmData.Lmd, Lmq = smpmData.Lmq, useDamperCage = smpmData.useDamperCage, Lrsigmad = smpmData.Lrsigmad, Lrsigmaq = smpmData.Lrsigmaq, Rrd = smpmData.Rrd, Rrq = smpmData.Rrq, TrRef = smpmData.TrRef, permanentMagnetLossParameters = smpmData.permanentMagnetLossParameters, TsOperational = 293.15, alpha20s = smpmData.alpha20s, TrOperational = 293.15, alpha20r = smpmData.alpha20r) annotation(
      Placement(transformation(extent = {{-20, -50}, {0, -30}})));
    Modelica.Electrical.Analog.Basic.Ground ground annotation(
      Placement(transformation(origin = {-90, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 270)));
    Modelica.Electrical.Machines.Utilities.TerminalBox terminalBox(terminalConnection = "Y") annotation(
      Placement(transformation(extent = {{-20, -34}, {0, -14}})));
    Modelica.Mechanics.Rotational.Components.Inertia inertiaLoad(J = JLoad) annotation(
      Placement(transformation(extent = {{60, -50}, {80, -30}})));
    Modelica.Mechanics.Rotational.Sources.QuadraticSpeedDependentTorque quadraticSpeedDependentTorque(tau_nominal = -TLoad, w_nominal(displayUnit = "rad/s") = wNominal) annotation(
      Placement(transformation(extent = {{100, -50}, {80, -30}})));
    Modelica.Electrical.Polyphase.Sensors.CurrentSensor currentSensor(m = m) annotation(
      Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 270, origin = {-10, 0})));
    Modelica.Mechanics.Rotational.Sensors.MultiSensor multiSensor annotation(
      Placement(transformation(origin = {40, -40}, extent = {{10, 10}, {-10, -10}}, rotation = 180)));
    Modelica.Electrical.Machines.Sensors.RotorDisplacementAngle rotorDisplacementAngle(p = smpm.p) annotation(
      Placement(transformation(origin = {30, -18}, extent = {{-10, 10}, {10, -10}})));
    Modelica.Electrical.Analog.Basic.Ground groundM annotation(
      Placement(transformation(origin = {-80, -28}, extent = {{-10, -10}, {10, 10}}, rotation = 270)));
    Modelica.Electrical.Polyphase.Basic.Star starM(final m = m) annotation(
      Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 180, origin = {-60, -10})));
    Modelica.Electrical.Machines.Sensors.VoltageQuasiRMSSensor voltageQuasiRMSSensor annotation(
      Placement(transformation(origin = {-30, -10}, extent = {{-10, 10}, {10, -10}}, rotation = 180)));
    parameter Modelica.Electrical.Machines.Utilities.ParameterRecords.SM_PermanentMagnetData smpmData(useDamperCage = false) "Synchronous machine data" annotation(
      Placement(transformation(extent = {{-20, -80}, {0, -60}})));
    Modelica.Electrical.Machines.Sensors.CurrentQuasiRMSSensor currentQuasiRMSSensor annotation(
      Placement(transformation(origin = {-10, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 270)));
    Modelica.Electrical.PowerConverters.DCAC.Polyphase2Level inverter annotation(
      Placement(transformation(origin = {70, 90}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Electrical.PowerConverters.DCAC.Control.PWM pwm(f = 2000, uMax = 400) annotation(
      Placement(transformation(origin = {42, 58}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Blocks.Sources.Ramp ramp(height = 50, duration = 1.2) annotation(
      Placement(transformation(origin = {-68, 58}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Electrical.Machines.Utilities.VfController vfController(VNominal = 400, fNominal = 50) annotation(
      Placement(transformation(origin = {-30, 58}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Electrical.Machines.SpacePhasors.Blocks.ToSpacePhasor toSpacePhasor annotation(
      Placement(transformation(origin = {6, 58}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Electrical.Analog.Sources.ConstantVoltage constantVoltage(V = 400) annotation(
      Placement(transformation(origin = {-16, 96}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  initial equation
    smpm.is[1:2] = zeros(2);
  equation
    connect(terminalBox.plug_sn, smpm.plug_sn) annotation(
      Line(points = {{-16, -30}, {-16, -30}}, color = {0, 0, 255}));
    connect(terminalBox.plug_sp, smpm.plug_sp) annotation(
      Line(points = {{-4, -30}, {-4, -30}}, color = {0, 0, 255}));
    connect(quadraticSpeedDependentTorque.flange, inertiaLoad.flange_b) annotation(
      Line(points = {{80, -40}, {80, -40}}));
    connect(currentSensor.plug_n, terminalBox.plugSupply) annotation(
      Line(points = {{-10, -10}, {-10, -28}}, color = {0, 0, 255}));
    connect(inertiaLoad.flange_a, multiSensor.flange_b) annotation(
      Line(points = {{60, -40}, {50, -40}}));
    connect(multiSensor.flange_a, smpm.flange) annotation(
      Line(points = {{30, -40}, {0, -40}}));
    connect(rotorDisplacementAngle.flange, smpm.flange) annotation(
      Line(points = {{30, -28}, {6, -28}, {6, -40}, {0, -40}}));
    connect(rotorDisplacementAngle.plug_p, smpm.plug_sp) annotation(
      Line(points = {{20, -24}, {3, -24}, {3, -30}, {-4, -30}}, color = {0, 0, 255}));
    connect(rotorDisplacementAngle.plug_n, smpm.plug_sn) annotation(
      Line(points = {{20, -12}, {-16, -12}, {-16, -30}}, color = {0, 0, 255}));
    connect(voltageQuasiRMSSensor.plug_p, currentSensor.plug_n) annotation(
      Line(points = {{-20, -10}, {-10, -10}}, color = {0, 0, 255}));
    connect(starM.plug_p, voltageQuasiRMSSensor.plug_n) annotation(
      Line(points = {{-50, -10}, {-40, -10}}, color = {0, 0, 255}));
    connect(groundM.p, starM.pin_n) annotation(
      Line(points = {{-70, -28}, {-70, -10}}, color = {0, 0, 255}));
    connect(currentQuasiRMSSensor.plug_n, currentSensor.plug_p) annotation(
      Line(points = {{-10, 10}, {-10, 10}}, color = {0, 0, 255}));
    connect(currentQuasiRMSSensor.plug_p, inverter.ac) annotation(
      Line(points = {{-10, 30}, {-10, 42}, {96, 42}, {96, 90}, {80, 90}}, color = {0, 0, 255}));
    connect(pwm.fire_p, inverter.fire_p) annotation(
      Line(points = {{53, 64}, {64, 64}, {64, 78}}, color = {255, 0, 255}, thickness = 0.5));
    connect(pwm.fire_n, inverter.fire_n) annotation(
      Line(points = {{53, 52}, {76, 52}, {76, 78}}, color = {255, 0, 255}, thickness = 0.5));
    connect(ramp.y, vfController.u) annotation(
      Line(points = {{-56, 58}, {-42, 58}}, color = {0, 0, 127}));
    connect(vfController.y, toSpacePhasor.u) annotation(
      Line(points = {{-18, 58}, {-6, 58}}, color = {0, 0, 127}, thickness = 0.5));
    connect(toSpacePhasor.y, pwm.u) annotation(
      Line(points = {{18, 58}, {30, 58}}, color = {0, 0, 127}, thickness = 0.5));
    connect(constantVoltage.p, inverter.dc_p) annotation(
      Line(points = {{-6, 96}, {60, 96}}, color = {0, 0, 255}));
    connect(constantVoltage.n, ground.p) annotation(
      Line(points = {{-26, 96}, {-70, 96}, {-70, 90}, {-80, 90}}, color = {0, 0, 255}));
    connect(inverter.dc_n, ground.p) annotation(
      Line(points = {{60, 84}, {-70, 84}, {-70, 90}, {-80, 90}}, color = {0, 0, 255}));
    annotation(
      experiment(StopTime = 2.0, Interval = 1E-4, Tolerance = 1e-06),
      Documentation(info = "<html>
    <p>
    A synchronous machine with permanent magnets accelerates a quadratic speed dependent load from standstill.
    The rms values of d- and q-current in rotor fixed coordinate system are controlled by the dqCurrentController,
    and the output voltages fed to the machine. The result shows that the torque is influenced by the q-current,
    whereas the stator voltage is influenced by the d-current.</p>
    
    <p>Default machine parameters are used.</p>
    </html>"),
      Diagram);
  end SMPM_VoltagePWM;

  block Transformation "Conversion of polyphase instantaneous values to space phasors"
    extends Modelica.Blocks.Interfaces.MIMO(final nin = m, final nout = 2);
    parameter Integer m(min = 1) = 3 "Number of phases" annotation(
      Evaluate = true);
    parameter Integer p(min = 1) = 1 "Number of phases" annotation(
      Evaluate = true);
    // Input: Rotor electrical angle
    Modelica.Blocks.Interfaces.RealInput angle(unit = "rad") annotation(
      Placement(transformation(origin = {0, -120}, extent = {{-20, -20}, {20, 20}}, rotation = 90)));
  protected
    Real ialpha;
    Real ibeta;
  equation
// Clarke Transformation
    ialpha = u[1];
    ibeta = 1/sqrt(3)*u[1] + 2/sqrt(3)*u[2];
// Park Transformation
    y[1] = ialpha*cos(p*angle) + ibeta*sin(p*angle);
// id
    y[2] = -ialpha*sin(p*angle) + ibeta*cos(p*angle);
// iq
    annotation(
      Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics = {Line(points = {{0, 0}, {80, 80}, {60, 72}, {72, 60}, {80, 80}}, color = {0, 0, 255}), Line(points = {{0, 0}, {80, -80}, {72, -60}, {60, -72}, {80, -80}}, color = {0, 0, 255}), Line(points = {{-80, 0}, {-73.33, 10}, {-66.67, 17.32}, {-60, 20}, {-53.33, 17.32}, {-46.67, 10}, {-40, 0}, {-33.33, -10}, {-26.67, -17.32}, {-20, -20}, {-13.33, -17.32}, {-6.67, -10}, {0, 0}}, color = {0, 0, 255}, smooth = Smooth.Bezier), Line(points = {{-90, 0}, {-83.33, 10}, {-76.67, 17.32}, {-70, 20}, {-63.33, 17.32}, {-56.67, 10}, {-50, 0}, {-43.33, -10}, {-36.67, -17.32}, {-30, -20}, {-23.33, -17.32}, {-16.67, -10}, {-10, 0}}, color = {0, 0, 255}, smooth = Smooth.Bezier), Line(points = {{-70, 0}, {-63.33, 10}, {-56.67, 17.32}, {-50, 20}, {-43.33, 17.32}, {-36.67, 10}, {-30, 0}, {-23.33, -10}, {-16.67, -17.32}, {-10, -20}, {-3.33, -17.32}, {3.33, -10}, {10, 0}}, color = {0, 0, 255}, smooth = Smooth.Bezier)}),
      Documentation(info = "<html>
Transformation of polyphase values (of voltages or currents) to space phasor and zero sequence value.
</html>"));
  end Transformation;

  model InverseTransformation
    extends Modelica.Blocks.Interfaces.MIMO(final nin = 2, final nout = m);
    parameter Integer m(min = 1) = 3 "Number of phases" annotation(
      Evaluate = true);
    Modelica.Blocks.Interfaces.RealInput angle(unit = "rad") annotation(
      Placement(transformation(origin = {0, -120}, extent = {{-20, -20}, {20, 20}}, rotation = 90)));
  protected
    Real ialpha;
    Real ibeta;
  public
    Modelica.Blocks.Interfaces.RealInput u annotation(
      Placement(transformation(extent = {{-140, -20}, {-100, 20}}), iconTransformation(extent = {{-140, -20}, {-100, 20}})));
    Modelica.Blocks.Interfaces.RealOutput y annotation(
      Placement(transformation(extent = {{100, -10}, {120, 10}}), iconTransformation(extent = {{100, -10}, {120, 10}})));
  equation
// Inverse Park Transformation
    ialpha = u[1]*cos(angle) - u[2]*sin(angle);
// alpha
    ibeta = u[1]*sin(angle) + u[2]*cos(angle);
// beta
// Inverse Clarke Transformation
    y[1] = ialpha;
    y[2] = -0.5*ialpha + sqrt(3)/2*ibeta;
    y[3] = -0.5*ialpha - sqrt(3)/2*ibeta;
    annotation(
      Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics = {Rectangle(lineColor = {0, 0, 127}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, -100}, {100, 100}}), Text(textColor = {0, 0, 255}, extent = {{-150, 150}, {150, 110}}, textString = "%name"), Line(points = {{0, 0}, {-80, 80}, {-60, 72}, {-72, 60}, {-80, 80}}, color = {0, 0, 255}), Line(points = {{0, 0}, {-80, -80}, {-72, -60}, {-60, -72}, {-80, -80}}, color = {0, 0, 255}), Line(points = {{0, 0}, {6.67, 10}, {13.33, 17.32}, {20, 20}, {26.67, 17.32}, {33.33, 10}, {40, 0}, {46.67, -10}, {53.33, -17.32}, {60, -20}, {66.67, -17.32}, {73.33, -10}, {80, 0}}, color = {0, 0, 255}, smooth = Smooth.Bezier), Line(points = {{-10, 0}, {-3.33, 10}, {3.33, 17.32}, {10, 20}, {16.67, 17.32}, {23.33, 10}, {30, 0}, {36.67, -10}, {43.33, -17.32}, {50, -20}, {56.67, -17.32}, {63.33, -10}, {70, 0}}, color = {0, 0, 255}, smooth = Smooth.Bezier), Line(points = {{10, 0}, {16.67, 10}, {23.33, 17.32}, {30, 20}, {36.67, 17.32}, {43.33, 10}, {50, 0}, {56.67, -10}, {63.33, -17.32}, {70, -20}, {76.67, -17.32}, {83.33, -10}, {90, 0}}, color = {0, 0, 255}, smooth = Smooth.Bezier)}),
      Documentation(info = "<html>
  Transformation of polyphase values (of voltages or currents) to space phasor and zero sequence value.
  </html>"));
  end InverseTransformation;

  block SVPWM "SpaceVector Pulse Width Modulation"
    parameter Modelica.Units.SI.Frequency f "Switching frequency";
    extends Modelica.Blocks.Interfaces.DiscreteBlock(final samplePeriod = 1/f);
    import Modelica.Constants.small;
    import Modelica.Constants.eps;
    import Modelica.Constants.pi;
    import Modelica.Math.atan2;
    constant Integer m = 3 "Number of phases";
    parameter Real uMax "Maximum length of space vector = half diagonal of hexagon";
    constant Boolean fire[6, m] = [true, false, false; true, true, false; false, true, false; false, true, true; false, false, true; true, false, true] "Switching patterns";
    Modelica.Blocks.Interfaces.RealInput u[3] "Reference space phasor" annotation(
      Placement(transformation(extent = {{-140, -20}, {-100, 20}})));
    Modelica.Blocks.Interfaces.BooleanOutput fire_p[m] "Positive fire signal" annotation(
      Placement(transformation(extent = {{100, 50}, {120, 70}})));
    Modelica.Blocks.Interfaces.BooleanOutput fire_n[m] "Negative fire signal" annotation(
      Placement(transformation(extent = {{100, -70}, {120, -50}})));
  protected
    discrete Real uRef(start = 0, fixed = true) "Length of reference vector";
    discrete Modelica.Units.SI.Angle phiRef(start = 0, fixed = true) "Angle of reference vector within (-pi, +pi]";
    discrete Modelica.Units.SI.Angle phiPos(start = 0, fixed = true) "Angle of reference vector within [0, 2*pi)";
    Integer ka(start = 0, fixed = true), kb(start = 0, fixed = true) "Switching patterns limiting the sector";
    discrete Modelica.Units.SI.Angle phiSec(start = 0, fixed = true) "Angle of reference vector within sector within [0, pi/m)";
    discrete Real ta(start = 0, fixed = true), tb(start = 0, fixed = true), t0(start = samplePeriod, fixed = true) "Relative time spans of vectors a, b, and 0";
    discrete Modelica.Units.SI.Time T0(start = startTime, fixed = true) "Start time of switching interval";
  algorithm
    when sampleTrigger then
//Limited relative reference signal
      uRef := min(sqrt(u[1]^2 + u[2]^2)/(2/3*uMax), cos(pi/6));
//Determine angle of reference signal within (-pi, +pi]
      phiRef := if noEvent(uRef < small) then 0 else atan2(u[2], u[1]);
//Shift angle to [0, 2*pi)
      phiPos := max(phiRef + (if phiRef < -eps then 2*pi else 0), 0);
//Determine sector and neighbour sector
      ka := integer(phiPos/(pi/m));
      kb := if noEvent(ka >= 5) then 0 else ka + 1;
//Determine angle within sector in the range of [0, pi/m)
      phiSec := phiPos - ka*pi/m;
//Determine limited relative time spans
//uRef*cos(phiSec)=tb*cos(pi/m) + ta;
//uRef*sin(phiSec)=tb*sin(pi/m);
      tb := min(uRef*sin(phiSec)/sin(pi/m), 1);
      ta := min(uRef*cos(phiSec) - tb*cos(pi/m), 1);
      t0 := max(1 - ta - tb, 0);
//Remember start time of switching interval
      T0 := time;
    end when;
  equation
//Distribute switching patterns t0/4 + ta/2 + tb/2 + t0/2 + tb/2 + ta/2 + t0/4
    if time < startTime then
      fire_p = fill(true, m);
    elseif (time - T0)/samplePeriod < (t0/4) then
      fire_p = fill(false, m);
    elseif (time - T0)/samplePeriod < (t0/4 + ta/2) then
      fire_p = fire[ka + 1, :];
    elseif (time - T0)/samplePeriod < (t0/4 + ta/2 + tb/2) then
      fire_p = fire[kb + 1, :];
    elseif (time - T0)/samplePeriod < (t0/4 + ta/2 + tb/2 + t0/2) then
      fire_p = fill(true, m);
    elseif (time - T0)/samplePeriod < (t0/4 + ta/2 + tb/2 + t0/2 + tb/2) then
      fire_p = fire[kb + 1, :];
    elseif (time - T0)/samplePeriod < (t0/4 + ta/2 + tb/2 + t0/2 + tb/2 + ta/2) then
      fire_p = fire[ka + 1, :];
    else
      fire_p = fill(false, m);
    end if;
    fire_n = not fire_p;
    annotation(
      defaultComponentName = "svPWM",
      Icon(coordinateSystem(preserveAspectRatio = false), graphics = {Line(points = {{-80, 30}, {-36, 30}, {-36, 50}, {-10, 50}, {-10, 30}, {10, 30}, {10, 50}, {36, 50}, {36, 30}, {80, 30}}, color = {255, 0, 0}), Line(points = {{-80, -10}, {-70, -10}, {-70, 10}, {-36, 10}, {-36, -10}, {36, -10}, {36, 10}, {70, 10}, {70, -10}, {80, -10}}, color = {0, 0, 255}), Line(points = {{-80, -50}, {-80, -30}, {-70, -30}, {-70, -50}, {-10, -50}, {-10, -30}, {10, -30}, {10, -50}, {70, -50}, {70, -30}, {80, -30}, {80, -50}}, color = {0, 0, 0})}),
      Documentation(info = "<html>
  <p>
  For a three-phase system, 8 space vectors are available according to the following switching patterns:
  </p>
  <ul>
  <li>0 [0,0,0] length 0</li>
  <li>1 [1,0,0] 000&deg;</li>
  <li>2 [1,1,0] 060&deg;</li>
  <li>3 [0,1,0] 120&deg;</li>
  <li>4 [0,1,1] 180&deg;</li>
  <li>5 [0,0,1] 240&deg;</li>
  <li>6 [1,0,1] 300&deg;</li>
  <li>7 [1,1,1] length 0</li>
  </ul>
  <p>
  Vector 1..6 form a hexagon, vector 0 and 7 are of length 0.
  </p>
  <p>
  First, the space vector is limited,
  and the sector of the hexagon is determined where the input space vector <u>u</u> is located;
  then the angle of the space vector within this sector 0&le;&phi;&lt;60&deg; is calculated.
  </p>
  <p>
  The input space vector is averaged by <u>u</u> = t<sub>a</sub>*<u>u</u><sub>a</sub> + t<sub>b</sub>*<u>u</u><sub>b</sub> + t<sub>0</sub>*0,
  where <u>u</u><sub>a</sub> is the space vector at the left border of the sector
  and <u>u</u><sub>b</sub> is the space vector at the right border of the sector.
  If necessary, a zero length vector is applied additionally.
  </p>
  <p>
  The relative time spans for averaging over one switching period are determined by the following equations:
  </p>
  <ul>
  <li>Real part: <u>u</u>*cos(&phi;) = <u>u</u><sub>b</sub>*t<sub>b</sub>*cos(60&deg;) + <u>u</u><sub>a</sub>*t<sub>a</sub>*1</li>
  <li>Imag.part: <u>u</u>*sin(&phi;) = <u>u</u><sub>b</sub>*t<sub>b</sub>*sin(60&deg;)</li>
  <li>t<sub>a</sub> + t<sub>b</sub> + t<sub>0</sub> = 1</li>
  </ul>
  <p>
  To obtain the positive fire signal, the switching time spans are distributed symmetrically:
  t<sub>0</sub>/4 + t<sub>a</sub>/2 + t<sub>b</sub>/2 +t<sub>0</sub>/2 + t<sub>b</sub>/2 + t<sub>a</sub>/2 + t<sub>0</sub>/4
  </p>
  <p>
  The switching pattern of the negative fire signal is just the inverse of the positive fire signal.
  </p>
  </html>"));
  end SVPWM;

  block SVPWMcustom
    parameter Modelica.Units.SI.Frequency f = 2000 "Switching frequency";
    extends Modelica.Blocks.Interfaces.DiscreteBlock(final samplePeriod = 1/f);
    parameter Real startTime = 0 "Start time";
    parameter Real Udc = 400 "DC bus voltage (Must match inverter DC bus)";
    parameter Real deadTime = 5e-6 "Dead time to avoid overlap";
    Modelica.Blocks.Interfaces.RealInput u[3] "Three-phase reference voltages (ABC frame)" annotation(
      Placement(transformation(extent = {{-140, -20}, {-100, 20}})));
    Modelica.Blocks.Interfaces.BooleanOutput fire_p[3] "Positive fire signal" annotation(
      Placement(transformation(extent = {{100, 50}, {120, 70}})));
    Modelica.Blocks.Interfaces.BooleanOutput fire_n[3] "Negative fire signal" annotation(
      Placement(transformation(extent = {{100, -70}, {120, -50}})));
  protected
    Real t_norm;
    Integer sector;
    Real Ta, Tb, Tc;
    discrete Modelica.Units.SI.Time T0(start = startTime, fixed = true) "Start time of switching interval";
  algorithm
    when sampleTrigger then
      T0 := time;
    end when;
// Compute duty cycles (handling invalid values)
    if sector == 1 or sector == 4 then
      Ta := (1/(2*Udc))*(Udc + (u[1] - u[3]));
      Tb := (1/(2*Udc))*(Udc + (2*u[2] - u[1] - u[3]));
      Tc := (1/(2*Udc))*(Udc + (u[3] - u[1]));
    elseif sector == 2 or sector == 5 then
      Ta := (1/(2*Udc))*(Udc + (2*u[1] - u[2] - u[3]));
      Tb := (1/(2*Udc))*(Udc + (u[2] - u[3]));
      Tc := (1/(2*Udc))*(Udc + (u[3] - u[2]));
    else
// sector == 3 or sector == 6
      Ta := (1/(2*Udc))*(Udc + (u[1] - u[2]));
      Tb := (1/(2*Udc))*(Udc + (u[2] - u[1]));
      Tc := (1/(2*Udc))*(Udc + (2*u[3] - u[1] - u[2]));
    end if;
  equation
// Normalize time within one switching period
    t_norm = (time - T0)*f;
// Determine sector using phase voltage comparisons
    sector = if u[1] > u[2] and u[2] > u[3] then 1 else if u[2] > u[1] and u[1] > u[3] then 2 else if u[2] > u[3] and u[3] > u[1] then 3 else if u[3] > u[2] and u[2] > u[1] then 4 else if u[3] > u[1] and u[1] > u[2] then 5 else 6;
// Generate PWM signals
    fire_p[1] = noEvent(t_norm < Ta);
    fire_p[2] = noEvent(t_norm < Tb);
    fire_p[3] = noEvent(t_norm < Tc);
// Ensure complementary switching
    fire_n[1] = not fire_p[1];
    fire_n[2] = not fire_p[2];
    fire_n[3] = not fire_p[3];
  end SVPWMcustom;

  model IMC_InverterDrive "Test example: InductionMachineSquirrelCage inverter drive"
    extends Modelica.Icons.Example;
    import Modelica.Constants.pi;
    import Modelica.Electrical.Polyphase.Functions.factorY2DC;
    constant Integer m = 3 "Number of phases";
    parameter Modelica.Units.SI.Voltage VNominal = 400 "Nominal RMS voltage per phase";
    parameter Modelica.Units.SI.Frequency fNominal = 50 "Nominal frequency";
    parameter Modelica.Units.SI.Resistance RGrid = 10e-3 "Grid choke resistance";
    parameter Modelica.Units.SI.Inductance LGrid = 500e-6 "Grid choke inductance";
    parameter Modelica.Units.SI.Voltage VDC = factorY2DC(m)*VNominal/sqrt(3) "Theoretical DC voltage";
    parameter Modelica.Units.SI.Capacitance CDC = 5e-3 "DC capacitor";
    parameter Modelica.Units.SI.Torque TLoad = 161.4 "Nominal load torque";
    parameter Modelica.Units.SI.AngularVelocity wLoad = 1440.45*2*pi/60 "Nominal load speed";
    parameter Modelica.Units.SI.Inertia JLoad = 0.29 "Load's moment of inertia";
    Modelica.Electrical.Analog.Basic.Ground ground annotation(
      Placement(transformation(origin = {-58, 34}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Electrical.PowerConverters.DCAC.Polyphase2Level inverter annotation(
      Placement(transformation(extent = {{-10, -10}, {10, 10}}, origin = {0, 70})));
    Modelica.Electrical.Machines.Sensors.CurrentQuasiRMSSensor machineCurrent annotation(
      Placement(transformation(extent = {{-10, 10}, {10, -10}}, rotation = 270, origin = {30, 0})));
    Modelica.Mechanics.Rotational.Components.Inertia loadInertia(J = JLoad) annotation(
      Placement(transformation(extent = {{50, -40}, {70, -20}})));
    Modelica.Blocks.Sources.Ramp ramp(height = fNominal, startTime = 0, duration = 1.2) annotation(
      Placement(transformation(extent = {{-10, -10}, {10, 10}}, origin = {-20, -50})));
    Modelica.Electrical.Machines.Utilities.VfController vfController(final m = m, VNominal = 200, fNominal = fNominal, EconomyMode = true) annotation(
      Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 90, origin = {0, -20})));
    SVPWMcustom pwm(Udc = 200) annotation(
      Placement(transformation(origin = {0, 34}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
    Modelica.Electrical.Analog.Sources.ConstantVoltage constantVoltage(V = 200) annotation(
      Placement(transformation(origin = {-58, 66}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
    parameter Modelica.Electrical.Machines.Utilities.ParameterRecords.SM_PermanentMagnetData smpmData annotation(
      Placement(transformation(origin = {20, -72}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Electrical.Machines.Utilities.TerminalBox terminalBox(terminalConnection = "Y") annotation(
      Placement(transformation(origin = {30, -14}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Electrical.Machines.BasicMachines.SynchronousMachines.SM_PermanentMagnet smpm(p = smpmData.p, fsNominal = smpmData.fsNominal, TsOperational = 293.15, Rs = smpmData.Rs, Jr = smpmData.Jr, VsOpenCircuit = smpmData.VsOpenCircuit, TsRef = smpmData.TsRef, alpha20s = smpmData.alpha20s, Lssigma = smpmData.Lssigma, useDamperCage = false, Lmd = smpmData.Lmd, Lmq = smpmData.Lmd) annotation(
      Placement(transformation(origin = {30, -30}, extent = {{-10, -10}, {10, 10}})));
  equation
    connect(vfController.u, ramp.y) annotation(
      Line(points = {{0, -32}, {0, -50}, {-9, -50}}, color = {0, 0, 127}));
    connect(inverter.ac, machineCurrent.plug_p) annotation(
      Line(points = {{10, 70}, {30, 70}, {30, 10}}, color = {0, 0, 255}));
    connect(pwm.u, vfController.y) annotation(
      Line(points = {{0, 22}, {0, -8}}, color = {0, 0, 127}, thickness = 0.5));
    connect(pwm.fire_p, inverter.fire_p) annotation(
      Line(points = {{-6, 46}, {-6, 58}}, color = {255, 0, 255}, thickness = 0.5));
    connect(pwm.fire_n, inverter.fire_n) annotation(
      Line(points = {{6, 46}, {6, 58}}, color = {255, 0, 255}, thickness = 0.5));
    connect(constantVoltage.n, ground.p) annotation(
      Line(points = {{-58, 56}, {-58, 44}}, color = {0, 0, 255}));
    connect(constantVoltage.p, inverter.dc_p) annotation(
      Line(points = {{-58, 76}, {-10, 76}}, color = {0, 0, 255}));
    connect(inverter.dc_n, ground.p) annotation(
      Line(points = {{-10, 64}, {-38, 64}, {-38, 44}, {-58, 44}}, color = {0, 0, 255}));
    connect(terminalBox.plug_sp, smpm.plug_sp) annotation(
      Line(points = {{36, -20}, {36, -20}}, color = {0, 0, 255}));
    connect(terminalBox.plug_sn, smpm.plug_sn) annotation(
      Line(points = {{24, -20}, {24, -20}}, color = {0, 0, 255}));
    connect(machineCurrent.plug_n, terminalBox.plugSupply) annotation(
      Line(points = {{30, -10}, {30, -18}}, color = {0, 0, 255}));
    connect(smpm.flange, loadInertia.flange_a) annotation(
      Line(points = {{40, -30}, {50, -30}}));
    annotation(
      experiment(StopTime = 1.5, Interval = 5e-05, Tolerance = 1e-06),
      Documentation(info = "<html>
  <p>
  This is a model of a complete inverter drive comprising:
  </p>
  <ul>
  <li>a grid model and a line choke</li>
  <li><a href=\"modelica://Modelica.Electrical.PowerConverters.ACDC.DiodeBridge2mPulse\">a diode rectifier</a></li>
  <li>a buffer capacitor</li>
  <li><a href=\"modelica://Modelica.Electrical.PowerConverters.DCAC.Polyphase2Level\">a switching inverter</a></li>
  <li><a href=\"modelica://Modelica.Electrical.PowerConverters.DCAC.Control.PWM\">a pulse width modulation</a></li>
  <li><a href=\"modelica://Modelica.Electrical.Machines.Utilities.VfController\">a voltage/frequency characteristic</a></li>
  <li>the reference frequency ramped up</li>
  <li>an induction machine with squirrel cage</li>
  <li>a load inertia and quadratic speed dependent load torque (like a fan or pump)</li>
  </ul>
  <p>Please note: Be patient, two switching devices cause many event iterations which cost performance.</p>
  <p>Note that due to the voltage drop the voltage at the machine can't reach the full voltage which means torque reduction.</p>
  <p>Default machine parameters are adapted to nominal phase voltage 400 V and nominal phase current 25 A.</p>
  </html>"));
  end IMC_InverterDrive;

  model ControllerPWM "Test example: PermanentMagnetSynchronousMachine fed by FOC"
    extends Modelica.Icons.Example;
    import Modelica.Constants.pi;
    constant Integer m = 3 "Number of phases";
    parameter Modelica.Units.SI.Current Idq[2] = {-53.5, 84.6} "Desired d- and q-current";
    parameter Modelica.Units.SI.AngularVelocity wNominal = 2*pi*smpmData.fsNominal/smpmData.p "Nominal speed";
    parameter Modelica.Units.SI.Torque TLoad = 181.4 "Nominal load torque";
    parameter Modelica.Units.SI.Inertia JLoad = 0.29 "Load's moment of inertia";
    Modelica.Electrical.Machines.BasicMachines.SynchronousMachines.SM_PermanentMagnet smpm(phiMechanical(start = 0, fixed = true), wMechanical(start = 0, fixed = true), useSupport = false, useThermalPort = false, p = smpmData.p, fsNominal = smpmData.fsNominal, Rs = smpmData.Rs, TsRef = smpmData.TsRef, Lszero = smpmData.Lszero, Lssigma = smpmData.Lssigma, Jr = smpmData.Jr, Js = smpmData.Js, frictionParameters = smpmData.frictionParameters, statorCoreParameters = smpmData.statorCoreParameters, strayLoadParameters = smpmData.strayLoadParameters, VsOpenCircuit = smpmData.VsOpenCircuit, Lmd = smpmData.Lmd, Lmq = smpmData.Lmq, useDamperCage = smpmData.useDamperCage, Lrsigmad = smpmData.Lrsigmad, Lrsigmaq = smpmData.Lrsigmaq, Rrd = smpmData.Rrd, Rrq = smpmData.Rrq, TrRef = smpmData.TrRef, permanentMagnetLossParameters = smpmData.permanentMagnetLossParameters, TsOperational = 293.15, alpha20s = smpmData.alpha20s, TrOperational = 293.15, alpha20r = smpmData.alpha20r) annotation(
      Placement(transformation(extent = {{-20, -50}, {0, -30}})));
    Modelica.Electrical.Analog.Basic.Ground ground annotation(
      Placement(transformation(origin = {-90, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 270)));
    Modelica.Blocks.Sources.Constant id(k = 0) annotation(
      Placement(transformation(origin = {-4, -4}, extent = {{-90, 60}, {-70, 80}})));
    Modelica.Electrical.Machines.Utilities.TerminalBox terminalBox(terminalConnection = "Y") annotation(
      Placement(transformation(extent = {{-20, -34}, {0, -14}})));
    Modelica.Mechanics.Rotational.Components.Inertia inertiaLoad(J = JLoad) annotation(
      Placement(transformation(extent = {{60, -50}, {80, -30}})));
    Modelica.Electrical.Polyphase.Sensors.CurrentSensor currentSensor(m = m) annotation(
      Placement(transformation(origin = {-10, -14}, extent = {{-10, -10}, {10, 10}}, rotation = 270)));
    Modelica.Mechanics.Rotational.Sensors.MultiSensor multiSensor annotation(
      Placement(transformation(extent = {{10, 10}, {-10, -10}}, rotation = 180, origin = {40, -40})));
    parameter Modelica.Electrical.Machines.Utilities.ParameterRecords.SM_PermanentMagnetData smpmData(useDamperCage = false) "Synchronous machine data" annotation(
      Placement(transformation(origin = {96, -16}, extent = {{-20, -80}, {0, -60}})));
    Modelica.Electrical.Machines.Sensors.CurrentQuasiRMSSensor currentQuasiRMSSensor annotation(
      Placement(transformation(origin = {-10, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 270)));
    Modelica.Electrical.Machines.Sensors.SinCosResolver sinCosResolver(p = 1) annotation(
      Placement(transformation(extent = {{-10, 10}, {10, -10}}, rotation = 270, origin = {30, -20})));
    Modelica.Electrical.Machines.Utilities.SinCosEvaluation sinCosEvaluation annotation(
      Placement(transformation(extent = {{10, -10}, {-10, 10}}, rotation = 270, origin = {30, 10})));
    Modelica.Electrical.Analog.Sources.ConstantVoltage constantVoltage(V = 150) annotation(
      Placement(transformation(origin = {-24, 90}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));
    Modelica.Electrical.PowerConverters.DCAC.Polyphase2Level inverter annotation(
      Placement(transformation(origin = {52, 84}, extent = {{-10, -10}, {10, 10}})));
    CurrentController3 currentController3(p = smpm.p, useRMS = false, fsNominal = smpm.fsNominal, VsOpenCircuit = smpm.VsOpenCircuit, Rs = smpm.Rs, Ld = smpm.Lssigma + smpm.Lmd, Lq = smpm.Lssigma + smpm.Lmq, decoupling = true) annotation(
      Placement(transformation(origin = {-46, 50}, extent = {{-10, -10}, {10, 10}})));
    SVPWMcustom pwm(Udc = 150) annotation(
      Placement(transformation(origin = {10, 50}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Blocks.Math.Gain cur_ref(k = 1.5*smpm.p*0.505) annotation(
      Placement(transformation(origin = {-84, 12}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
    Modelica.Electrical.Machines.Examples.ControlledDCDrives.Utilities.LimitedPI PI_w(useI = true, Ti = 0.042, initType = Modelica.Blocks.Types.Init.InitialOutput) annotation(
      Placement(transformation(origin = {-84, -52}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
    Modelica.Blocks.Sources.Constant w_ref(k = 100) annotation(
      Placement(transformation(origin = {-84, -84}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  initial equation
    smpm.is[1:2] = zeros(2);
  equation
    connect(terminalBox.plug_sn, smpm.plug_sn) annotation(
      Line(points = {{-16, -30}, {-16, -30}}, color = {0, 0, 255}));
    connect(terminalBox.plug_sp, smpm.plug_sp) annotation(
      Line(points = {{-4, -30}, {-4, -30}}, color = {0, 0, 255}));
    connect(currentSensor.plug_n, terminalBox.plugSupply) annotation(
      Line(points = {{-10, -24}, {-10, -28}}, color = {0, 0, 255}));
    connect(inertiaLoad.flange_a, multiSensor.flange_b) annotation(
      Line(points = {{60, -40}, {50, -40}}));
    connect(multiSensor.flange_a, smpm.flange) annotation(
      Line(points = {{30, -40}, {0, -40}}));
    connect(currentQuasiRMSSensor.plug_n, currentSensor.plug_p) annotation(
      Line(points = {{-10, 10}, {-10, -4}}, color = {0, 0, 255}));
    connect(smpm.flange, sinCosResolver.flange) annotation(
      Line(points = {{0, -40}, {30, -40}, {30, -30}}, color = {0, 0, 0}));
    connect(sinCosResolver.y, sinCosEvaluation.u) annotation(
      Line(points = {{30, -9}, {30, -2}}, color = {0, 0, 127}));
    connect(ground.p, constantVoltage.n) annotation(
      Line(points = {{-80, 90}, {-34, 90}}, color = {0, 0, 255}));
    connect(constantVoltage.p, inverter.dc_p) annotation(
      Line(points = {{-14, 90}, {42, 90}}, color = {0, 0, 255}));
    connect(inverter.dc_n, ground.p) annotation(
      Line(points = {{42, 78}, {-58, 78}, {-58, 90}, {-80, 90}}, color = {0, 0, 255}));
    connect(inverter.ac, currentQuasiRMSSensor.plug_p) annotation(
      Line(points = {{62, 84}, {88, 84}, {88, 33.5}, {-10, 33.5}, {-10, 30}}, color = {0, 0, 255}));
    connect(id.y, currentController3.id) annotation(
      Line(points = {{-73, 66}, {-64, 66}, {-64, 56}, {-58, 56}}, color = {0, 0, 127}));
    connect(sinCosEvaluation.phi, currentController3.phi) annotation(
      Line(points = {{30, 22}, {30, 36}, {-40, 36}, {-40, 38}}, color = {0, 0, 127}));
    connect(currentController3.y, pwm.u) annotation(
      Line(points = {{-34, 50}, {-2, 50}}, color = {0, 0, 127}, thickness = 0.5));
    connect(pwm.fire_p, inverter.fire_p) annotation(
      Line(points = {{22, 56}, {46, 56}, {46, 72}}, color = {255, 0, 255}, thickness = 0.5));
    connect(pwm.fire_n, inverter.fire_n) annotation(
      Line(points = {{22, 44}, {58, 44}, {58, 72}}, color = {255, 0, 255}, thickness = 0.5));
    connect(currentController3.iq, cur_ref.y) annotation(
      Line(points = {{-58, 44}, {-84, 44}, {-84, 24}}, color = {0, 0, 127}));
    connect(PI_w.y, cur_ref.u) annotation(
      Line(points = {{-84, -41}, {-84, 0}}, color = {0, 0, 127}));
    connect(multiSensor.w, PI_w.u_m) annotation(
      Line(points = {{46, -50}, {46, -58}, {-72, -58}}, color = {0, 0, 127}));
    connect(w_ref.y, PI_w.u) annotation(
      Line(points = {{-84, -72}, {-84, -64}}, color = {0, 0, 127}));
  connect(currentSensor.i, currentController3.iActual) annotation(
      Line(points = {{-20, -14}, {-52, -14}, {-52, 38}}, color = {0, 0, 127}, thickness = 0.5));
    annotation(
      experiment(StopTime = 2.0, Interval = 1E-4, Tolerance = 1e-06),
      Documentation(info = "<html>
  <p>
  A synchronous machine with permanent magnets accelerates a quadratic speed dependent load from standstill.
  The rms values of d- and q-current in rotor fixed coordinate system are controlled by the dqCurrentController,
  and the output voltages fed to the machine. The result shows that the torque is influenced by the q-current,
  whereas the stator voltage is influenced by the d-current.</p>
  
  <p>Default machine parameters are used.</p>
  </html>"),
      Diagram);
  end ControllerPWM;

  model ControllerPWM2 "Test example: PermanentMagnetSynchronousMachine fed by FOC"
    extends Modelica.Icons.Example;
    import Modelica.Constants.pi;
    constant Integer m = 3 "Number of phases";
    parameter Modelica.Units.SI.Current Idq[2] = {0, 5} "Desired d- and q-current";
    parameter Modelica.Units.SI.AngularVelocity wNominal = 2*pi*smpmData.fsNominal/smpmData.p "Nominal speed";
    parameter Modelica.Units.SI.Torque TLoad = 181.4 "Nominal load torque";
    parameter Modelica.Units.SI.Inertia JLoad = 0.29 "Load's moment of inertia";
    Modelica.Electrical.Machines.BasicMachines.SynchronousMachines.SM_PermanentMagnet smpm(phiMechanical(start = 0, fixed = true), wMechanical(start = 0, fixed = true), useSupport = false, useThermalPort = false, p = smpmData.p, fsNominal = smpmData.fsNominal, Rs = smpmData.Rs, TsRef = smpmData.TsRef, Lszero = smpmData.Lszero, Lssigma = smpmData.Lssigma, Jr = smpmData.Jr, Js = smpmData.Js, frictionParameters = smpmData.frictionParameters, statorCoreParameters = smpmData.statorCoreParameters, strayLoadParameters = smpmData.strayLoadParameters, VsOpenCircuit = smpmData.VsOpenCircuit, Lmd = smpmData.Lmd, Lmq = smpmData.Lmq, useDamperCage = smpmData.useDamperCage, Lrsigmad = smpmData.Lrsigmad, Lrsigmaq = smpmData.Lrsigmaq, Rrd = smpmData.Rrd, Rrq = smpmData.Rrq, TrRef = smpmData.TrRef, permanentMagnetLossParameters = smpmData.permanentMagnetLossParameters, TsOperational = 293.15, alpha20s = smpmData.alpha20s, TrOperational = 293.15, alpha20r = smpmData.alpha20r) annotation(
      Placement(transformation(extent = {{-20, -50}, {0, -30}})));
    Modelica.Electrical.Analog.Basic.Ground ground annotation(
      Placement(transformation(origin = {-90, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 270)));
    Modelica.Blocks.Sources.Constant id(k = 0) annotation(
      Placement(transformation(extent = {{-90, 60}, {-70, 80}})));
    Modelica.Electrical.Machines.Utilities.TerminalBox terminalBox(terminalConnection = "Y") annotation(
      Placement(transformation(extent = {{-20, -34}, {0, -14}})));
    Modelica.Mechanics.Rotational.Components.Inertia inertiaLoad(J = JLoad) annotation(
      Placement(transformation(extent = {{60, -50}, {80, -30}})));
    Modelica.Electrical.Polyphase.Sensors.CurrentSensor currentSensor(m = m) annotation(
      Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 270, origin = {-10, 0})));
    Modelica.Mechanics.Rotational.Sensors.MultiSensor multiSensor annotation(
      Placement(transformation(origin = {40, -54}, extent = {{-10, 10}, {10, -10}}, rotation = -180)));
    Modelica.Electrical.Analog.Basic.Ground groundM annotation(
      Placement(transformation(origin = {-80, -28}, extent = {{-10, -10}, {10, 10}}, rotation = 270)));
    Modelica.Electrical.Polyphase.Basic.Star starM(final m = m) annotation(
      Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 180, origin = {-60, -10})));
    Modelica.Electrical.Machines.Sensors.VoltageQuasiRMSSensor voltageQuasiRMSSensor annotation(
      Placement(transformation(extent = {{-10, 10}, {10, -10}}, rotation = 180, origin = {-30, -10})));
    parameter Modelica.Electrical.Machines.Utilities.ParameterRecords.SM_PermanentMagnetData smpmData(useDamperCage = false) "Synchronous machine data" annotation(
      Placement(transformation(extent = {{-20, -80}, {0, -60}})));
    Modelica.Electrical.Machines.Sensors.CurrentQuasiRMSSensor currentQuasiRMSSensor annotation(
      Placement(transformation(origin = {-10, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 270)));
    Modelica.Electrical.Machines.Sensors.SinCosResolver sinCosResolver(p = 2) annotation(
      Placement(transformation(extent = {{-10, 10}, {10, -10}}, rotation = 270, origin = {30, -20})));
    Modelica.Electrical.Machines.Utilities.SinCosEvaluation sinCosEvaluation annotation(
      Placement(transformation(extent = {{10, -10}, {-10, 10}}, rotation = 270, origin = {30, 10})));
    Modelica.Electrical.Analog.Sources.ConstantVoltage constantVoltage(V = 150) annotation(
      Placement(transformation(origin = {-24, 90}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));
    Modelica.Electrical.PowerConverters.DCAC.Polyphase2Level inverter annotation(
      Placement(transformation(origin = {52, 84}, extent = {{-10, -10}, {10, 10}})));
    CurrentController currentController(fsNominal = 50, VsOpenCircuit = smpm.VsOpenCircuit, Rs = smpm.Rs, Ld = smpm.Lssigma + smpm.Lmd, Lq = smpm.Lssigma + smpm.Lmq, decoupling = true, p = smpm.p, useRMS = false) annotation(
      Placement(transformation(origin = {-40, 52}, extent = {{-10, -10}, {10, 10}})));
    SVPWMcustom pwm(Udc = 150, f = 2000) annotation(
      Placement(transformation(origin = {2, 52}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Blocks.Math.Gain gain(k = 1.5*smpm.p*0.505) annotation(
      Placement(transformation(origin = {-90, 34}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Electrical.Machines.Examples.ControlledDCDrives.Utilities.LimitedPI PI_q(k = 1, useI = true, Ti = 0.042, initType = Modelica.Blocks.Types.Init.InitialOutput) annotation(
      Placement(transformation(origin = {-134, 34}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Blocks.Sources.Constant const(k = 100) annotation(
      Placement(transformation(origin = {-174, 34}, extent = {{-10, -10}, {10, 10}})));
  initial equation
    smpm.is[1:2] = zeros(2);
  equation
    connect(terminalBox.plug_sn, smpm.plug_sn) annotation(
      Line(points = {{-16, -30}, {-16, -30}}, color = {0, 0, 255}));
    connect(terminalBox.plug_sp, smpm.plug_sp) annotation(
      Line(points = {{-4, -30}, {-4, -30}}, color = {0, 0, 255}));
    connect(currentSensor.plug_n, terminalBox.plugSupply) annotation(
      Line(points = {{-10, -10}, {-10, -28}}, color = {0, 0, 255}));
    connect(voltageQuasiRMSSensor.plug_p, currentSensor.plug_n) annotation(
      Line(points = {{-20, -10}, {-10, -10}}, color = {0, 0, 255}));
    connect(starM.plug_p, voltageQuasiRMSSensor.plug_n) annotation(
      Line(points = {{-50, -10}, {-40, -10}}, color = {0, 0, 255}));
    connect(groundM.p, starM.pin_n) annotation(
      Line(points = {{-70, -28}, {-70, -10}}, color = {0, 0, 255}));
    connect(currentQuasiRMSSensor.plug_n, currentSensor.plug_p) annotation(
      Line(points = {{-10, 10}, {-10, 10}}, color = {0, 0, 255}));
    connect(smpm.flange, sinCosResolver.flange) annotation(
      Line(points = {{0, -40}, {30, -40}, {30, -30}}, color = {0, 0, 0}));
    connect(sinCosResolver.y, sinCosEvaluation.u) annotation(
      Line(points = {{30, -9}, {30, -2}}, color = {0, 0, 127}));
    connect(ground.p, constantVoltage.n) annotation(
      Line(points = {{-80, 90}, {-34, 90}}, color = {0, 0, 255}));
    connect(constantVoltage.p, inverter.dc_p) annotation(
      Line(points = {{-14, 90}, {42, 90}}, color = {0, 0, 255}));
    connect(inverter.dc_n, ground.p) annotation(
      Line(points = {{42, 78}, {-58, 78}, {-58, 90}, {-80, 90}}, color = {0, 0, 255}));
    connect(inverter.ac, currentQuasiRMSSensor.plug_p) annotation(
      Line(points = {{62, 84}, {88, 84}, {88, 42}, {86, 42}, {86, 35}, {48, 35}, {48, 33.5}, {-10, 33.5}, {-10, 30}}, color = {0, 0, 255}));
    connect(id.y, currentController.id) annotation(
      Line(points = {{-68, 70}, {-60, 70}, {-60, 58}, {-52, 58}}, color = {0, 0, 127}));
    connect(currentSensor.i, currentController.iActual) annotation(
      Line(points = {{-20, 0}, {-46, 0}, {-46, 40}}, color = {0, 0, 127}, thickness = 0.5));
    connect(sinCosEvaluation.phi, currentController.phi) annotation(
      Line(points = {{30, 22}, {28, 22}, {28, 34}, {-34, 34}, {-34, 40}}, color = {0, 0, 127}));
    connect(currentController.y, pwm.u) annotation(
      Line(points = {{-28, 52}, {-10, 52}}, color = {0, 0, 127}, thickness = 0.5));
    connect(pwm.fire_p, inverter.fire_p) annotation(
      Line(points = {{14, 58}, {46, 58}, {46, 72}}, color = {255, 0, 255}, thickness = 0.5));
    connect(pwm.fire_n, inverter.fire_n) annotation(
      Line(points = {{14, 46}, {58, 46}, {58, 72}}, color = {255, 0, 255}, thickness = 0.5));
    connect(gain.y, currentController.iq) annotation(
      Line(points = {{-78, 34}, {-68, 34}, {-68, 46}, {-52, 46}}, color = {0, 0, 127}));
    connect(PI_q.y, gain.u) annotation(
      Line(points = {{-122, 34}, {-102, 34}}, color = {0, 0, 127}));
    connect(multiSensor.w, PI_q.u_m) annotation(
      Line(points = {{34, -65}, {34, -84}, {-140, -84}, {-140, 22}}, color = {0, 0, 127}));
    connect(const.y, PI_q.u) annotation(
      Line(points = {{-162, 34}, {-146, 34}}, color = {0, 0, 127}));
    connect(multiSensor.flange_a, inertiaLoad.flange_a) annotation(
      Line(points = {{50, -54}, {52, -54}, {52, -40}, {60, -40}}));
    connect(multiSensor.flange_b, smpm.flange) annotation(
      Line(points = {{30, -54}, {18, -54}, {18, -40}, {0, -40}}));
    annotation(
      experiment(StopTime = 2.0, Interval = 1E-4, Tolerance = 1e-06),
      Documentation(info = "<html>
    <p>
    A synchronous machine with permanent magnets accelerates a quadratic speed dependent load from standstill.
    The rms values of d- and q-current in rotor fixed coordinate system are controlled by the dqCurrentController,
    and the output voltages fed to the machine. The result shows that the torque is influenced by the q-current,
    whereas the stator voltage is influenced by the d-current.</p>
    
    <p>Default machine parameters are used.</p>
    </html>"));
  end ControllerPWM2;

  model ControllerPWM3 "Test example: PermanentMagnetSynchronousMachine fed by FOC"
    extends Modelica.Icons.Example;
    import Modelica.Constants.pi;
    constant Integer m = 3 "Number of phases";
    parameter Modelica.Units.SI.Current Idq[2] = {-53.5, 84.6} "Desired d- and q-current";
    parameter Modelica.Units.SI.AngularVelocity wNominal = 2*pi*smpmData.fsNominal/smpmData.p "Nominal speed";
    parameter Modelica.Units.SI.Torque TLoad = 181.4 "Nominal load torque";
    parameter Modelica.Units.SI.Inertia JLoad = 0.29 "Load's moment of inertia";
    Modelica.Electrical.Machines.BasicMachines.SynchronousMachines.SM_PermanentMagnet smpm(phiMechanical(start = 0, fixed = true), wMechanical(start = 0, fixed = true), useSupport = false, useThermalPort = false, p = smpmData.p, fsNominal = smpmData.fsNominal, Rs = smpmData.Rs, TsRef = smpmData.TsRef, Lszero = smpmData.Lszero, Lssigma = smpmData.Lssigma, Jr = smpmData.Jr, Js = smpmData.Js, frictionParameters = smpmData.frictionParameters, statorCoreParameters = smpmData.statorCoreParameters, strayLoadParameters = smpmData.strayLoadParameters, VsOpenCircuit = smpmData.VsOpenCircuit, Lmd = smpmData.Lmd, Lmq = smpmData.Lmq, useDamperCage = false, Lrsigmad = smpmData.Lrsigmad, Lrsigmaq = smpmData.Lrsigmaq, Rrd = smpmData.Rrd, Rrq = smpmData.Rrq, TrRef = smpmData.TrRef, permanentMagnetLossParameters = smpmData.permanentMagnetLossParameters, TsOperational = 293.15, alpha20s = smpmData.alpha20s, TrOperational = 293.15, alpha20r = smpmData.alpha20r) annotation(
      Placement(transformation(extent = {{-20, -50}, {0, -30}})));
    Modelica.Electrical.Analog.Basic.Ground ground annotation(
      Placement(transformation(origin = {-90, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 270)));
    Modelica.Electrical.Machines.Utilities.TerminalBox terminalBox(terminalConnection = "Y") annotation(
      Placement(transformation(extent = {{-20, -34}, {0, -14}})));
    Modelica.Mechanics.Rotational.Components.Inertia inertiaLoad(J = JLoad) annotation(
      Placement(transformation(extent = {{60, -50}, {80, -30}})));
    Modelica.Electrical.Polyphase.Sensors.CurrentSensor currentSensor(m = m) annotation(
      Placement(transformation(origin = {-10, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 270)));
    Modelica.Mechanics.Rotational.Sensors.MultiSensor multiSensor annotation(
      Placement(transformation(extent = {{10, 10}, {-10, -10}}, rotation = 180, origin = {40, -40})));
    Modelica.Electrical.Analog.Basic.Ground groundM annotation(
      Placement(transformation(origin = {-80, -28}, extent = {{-10, -10}, {10, 10}}, rotation = 270)));
    Modelica.Electrical.Polyphase.Basic.Star starM(final m = m) annotation(
      Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 180, origin = {-60, -10})));
    parameter Modelica.Electrical.Machines.Utilities.ParameterRecords.SM_PermanentMagnetData smpmData(useDamperCage = false) "Synchronous machine data" annotation(
      Placement(transformation(extent = {{-20, -80}, {0, -60}})));
    Modelica.Electrical.Machines.Sensors.CurrentQuasiRMSSensor currentQuasiRMSSensor annotation(
      Placement(transformation(origin = {-10, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 270)));
    Modelica.Electrical.Machines.Sensors.SinCosResolver sinCosResolver(p = smpm.p) annotation(
      Placement(transformation(extent = {{-10, 10}, {10, -10}}, rotation = 270, origin = {30, -20})));
    Modelica.Electrical.Machines.Utilities.SinCosEvaluation sinCosEvaluation annotation(
      Placement(transformation(extent = {{10, -10}, {-10, 10}}, rotation = 270, origin = {30, 10})));
    Modelica.Electrical.Analog.Sources.ConstantVoltage constantVoltage(V = 150) annotation(
      Placement(transformation(origin = {-24, 90}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));
    Modelica.Electrical.PowerConverters.DCAC.Polyphase2Level inverter annotation(
      Placement(transformation(origin = {52, 84}, extent = {{-10, -10}, {10, 10}})));
    CurrentController2 currentController2(p = smpm.p, fsNominal = smpm.fsNominal, VsOpenCircuit = smpm.VsOpenCircuit, Rs = smpm.Rs, Ld = smpm.Lssigma + smpm.Lmd, Lq = smpm.Lssigma + smpm.Lmq, decoupling = true, useRMS = false) annotation(
      Placement(transformation(origin = {-42, 50}, extent = {{-10, -10}, {10, 10}})));
    SVPWMcustom pwm(f = 2000, Udc = 150) annotation(
      Placement(transformation(origin = {0, 50}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Electrical.Polyphase.Sensors.VoltageSensor voltageSensor annotation(
      Placement(transformation(origin = {-34, -10}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));
    Modelica.Blocks.Sources.Constant id(k = 0) annotation(
      Placement(transformation(origin = {-90, 60}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Electrical.Machines.Examples.ControlledDCDrives.Utilities.LimitedPI PI_q(k = 1, Ti = 0.042, initType = Modelica.Blocks.Types.Init.InitialOutput) annotation(
      Placement(transformation(origin = {-126, 26}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Blocks.Sources.Constant wm_ref(k = 100) annotation(
      Placement(transformation(origin = {-172, 26}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Blocks.Math.Gain gain1(k = 1.5*smpm.p*0.505) annotation(
      Placement(transformation(origin = {-84, 26}, extent = {{-10, -10}, {10, 10}})));
  initial equation
    smpm.is[1:2] = zeros(2);
  equation
    connect(terminalBox.plug_sn, smpm.plug_sn) annotation(
      Line(points = {{-16, -30}, {-16, -30}}, color = {0, 0, 255}));
    connect(terminalBox.plug_sp, smpm.plug_sp) annotation(
      Line(points = {{-4, -30}, {-4, -30}}, color = {0, 0, 255}));
    connect(currentSensor.plug_n, terminalBox.plugSupply) annotation(
      Line(points = {{-10, -10}, {-10, -28}}, color = {0, 0, 255}));
    connect(inertiaLoad.flange_a, multiSensor.flange_b) annotation(
      Line(points = {{60, -40}, {50, -40}}));
    connect(multiSensor.flange_a, smpm.flange) annotation(
      Line(points = {{30, -40}, {0, -40}}));
    connect(groundM.p, starM.pin_n) annotation(
      Line(points = {{-70, -28}, {-70, -10}}, color = {0, 0, 255}));
    connect(currentQuasiRMSSensor.plug_n, currentSensor.plug_p) annotation(
      Line(points = {{-10, 10}, {-10, 10}}, color = {0, 0, 255}));
    connect(smpm.flange, sinCosResolver.flange) annotation(
      Line(points = {{0, -40}, {30, -40}, {30, -30}}, color = {0, 0, 0}));
    connect(sinCosResolver.y, sinCosEvaluation.u) annotation(
      Line(points = {{30, -9}, {30, -2}}, color = {0, 0, 127}));
    connect(ground.p, constantVoltage.n) annotation(
      Line(points = {{-80, 90}, {-34, 90}}, color = {0, 0, 255}));
    connect(constantVoltage.p, inverter.dc_p) annotation(
      Line(points = {{-14, 90}, {42, 90}}, color = {0, 0, 255}));
    connect(inverter.dc_n, ground.p) annotation(
      Line(points = {{42, 78}, {-58, 78}, {-58, 90}, {-80, 90}}, color = {0, 0, 255}));
    connect(inverter.ac, currentQuasiRMSSensor.plug_p) annotation(
      Line(points = {{62, 84}, {88, 84}, {88, 42}, {86, 42}, {86, 35}, {48, 35}, {48, 33.5}, {-10, 33.5}, {-10, 30}}, color = {0, 0, 255}));
    connect(sinCosEvaluation.phi, currentController2.phi) annotation(
      Line(points = {{30, 22}, {30, 28}, {-36, 28}, {-36, 38}}, color = {0, 0, 127}));
    connect(currentController2.y, pwm.u) annotation(
      Line(points = {{-30, 50}, {-12, 50}}, color = {0, 0, 127}, thickness = 0.5));
    connect(pwm.fire_p, inverter.fire_p) annotation(
      Line(points = {{12, 56}, {46, 56}, {46, 72}}, color = {255, 0, 255}, thickness = 0.5));
    connect(pwm.fire_n, inverter.fire_n) annotation(
      Line(points = {{12, 44}, {58, 44}, {58, 72}}, color = {255, 0, 255}, thickness = 0.5));
    connect(voltageSensor.plug_n, starM.plug_p) annotation(
      Line(points = {{-44, -10}, {-50, -10}}, color = {0, 0, 255}));
    connect(voltageSensor.plug_p, currentSensor.plug_n) annotation(
      Line(points = {{-24, -10}, {-10, -10}}, color = {0, 0, 255}));
    connect(id.y, currentController2.id) annotation(
      Line(points = {{-78, 60}, {-68, 60}, {-68, 56}, {-54, 56}}, color = {0, 0, 127}));
    connect(multiSensor.w, PI_q.u_m) annotation(
      Line(points = {{46, -50}, {46, -92}, {-132, -92}, {-132, 14}}, color = {0, 0, 127}));
    connect(currentSensor.i, currentController2.iActual) annotation(
      Line(points = {{-20, 0}, {-48, 0}, {-48, 38}}, color = {0, 0, 127}, thickness = 0.5));
    connect(gain1.y, currentController2.iq) annotation(
      Line(points = {{-72, 26}, {-68, 26}, {-68, 44}, {-54, 44}}, color = {0, 0, 127}));
    connect(PI_q.y, gain1.u) annotation(
      Line(points = {{-114, 26}, {-96, 26}}, color = {0, 0, 127}));
    connect(wm_ref.y, PI_q.u) annotation(
      Line(points = {{-160, 26}, {-138, 26}}, color = {0, 0, 127}));
    annotation(
      experiment(StopTime = 2.0, Interval = 1E-4, Tolerance = 1e-06),
      Documentation(info = "<html>
    <p>
    A synchronous machine with permanent magnets accelerates a quadratic speed dependent load from standstill.
    The rms values of d- and q-current in rotor fixed coordinate system are controlled by the dqCurrentController,
    and the output voltages fed to the machine. The result shows that the torque is influenced by the q-current,
    whereas the stator voltage is influenced by the d-current.</p>
    
    <p>Default machine parameters are used.</p>
    </html>"));
  end ControllerPWM3;

  model SMPM_VoltageSource2 "Test example: PermanentMagnetSynchronousMachine fed by FOC"
    extends Modelica.Icons.Example;
    import Modelica.Constants.pi;
    constant Integer m = 3 "Number of phases";
    parameter Modelica.Units.SI.Current Idq[2] = {-53.5, 84.6} "Desired d- and q-current";
    parameter Modelica.Units.SI.AngularVelocity wNominal = 2*pi*smpmData.fsNominal/smpmData.p "Nominal speed";
    parameter Modelica.Units.SI.Torque TLoad = 181.4 "Nominal load torque";
    parameter Modelica.Units.SI.Inertia JLoad = 0.29 "Load's moment of inertia";
    Modelica.Electrical.Machines.BasicMachines.SynchronousMachines.SM_PermanentMagnet smpm(phiMechanical(start = 0, fixed = true), wMechanical(start = 0, fixed = true), useSupport = false, useThermalPort = false, p = smpmData.p, fsNominal = smpmData.fsNominal, Rs = smpmData.Rs, TsRef = smpmData.TsRef, Lszero = smpmData.Lszero, Lssigma = smpmData.Lssigma, Jr = smpmData.Jr, Js = smpmData.Js, frictionParameters = smpmData.frictionParameters, statorCoreParameters = smpmData.statorCoreParameters, strayLoadParameters = smpmData.strayLoadParameters, VsOpenCircuit = smpmData.VsOpenCircuit, Lmd = smpmData.Lmd, Lmq = smpmData.Lmq, useDamperCage = smpmData.useDamperCage, Lrsigmad = smpmData.Lrsigmad, Lrsigmaq = smpmData.Lrsigmaq, Rrd = smpmData.Rrd, Rrq = smpmData.Rrq, TrRef = smpmData.TrRef, permanentMagnetLossParameters = smpmData.permanentMagnetLossParameters, TsOperational = 293.15, alpha20s = smpmData.alpha20s, TrOperational = 293.15, alpha20r = smpmData.alpha20r) annotation(
      Placement(transformation(extent = {{-20, -50}, {0, -30}})));
    Modelica.Electrical.Polyphase.Sources.SignalVoltage signalVoltage(final m = m) annotation(
      Placement(transformation(origin = {-10, 50}, extent = {{10, 10}, {-10, -10}}, rotation = 270)));
    Modelica.Electrical.Polyphase.Basic.Star star(final m = m) annotation(
      Placement(transformation(extent = {{-50, 80}, {-70, 100}})));
    Modelica.Electrical.Analog.Basic.Ground ground annotation(
      Placement(transformation(origin = {-90, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 270)));
    Modelica.Blocks.Sources.Constant iq(k = Idq[2]) annotation(
      Placement(transformation(extent = {{-90, 20}, {-70, 40}})));
    Modelica.Blocks.Sources.Constant id(k = 0) annotation(
      Placement(transformation(extent = {{-90, 60}, {-70, 80}})));
    Modelica.Electrical.Machines.Utilities.TerminalBox terminalBox(terminalConnection = "Y") annotation(
      Placement(transformation(extent = {{-20, -34}, {0, -14}})));
    Modelica.Mechanics.Rotational.Components.Inertia inertiaLoad(J = JLoad) annotation(
      Placement(transformation(extent = {{60, -50}, {80, -30}})));
    Modelica.Mechanics.Rotational.Sources.QuadraticSpeedDependentTorque quadraticSpeedDependentTorque(tau_nominal = -TLoad, w_nominal(displayUnit = "rad/s") = wNominal) annotation(
      Placement(transformation(extent = {{100, -50}, {80, -30}})));
    Modelica.Electrical.Polyphase.Sensors.CurrentSensor currentSensor(m = m) annotation(
      Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 270, origin = {-10, 0})));
    Modelica.Mechanics.Rotational.Sensors.MultiSensor multiSensor annotation(
      Placement(transformation(extent = {{10, 10}, {-10, -10}}, rotation = 180, origin = {40, -40})));
    Modelica.Electrical.Machines.Sensors.RotorDisplacementAngle rotorDisplacementAngle(p = smpm.p) annotation(
      Placement(transformation(origin = {10, -20}, extent = {{-10, 10}, {10, -10}})));
    Modelica.Electrical.Analog.Basic.Ground groundM annotation(
      Placement(transformation(origin = {-80, -28}, extent = {{-10, -10}, {10, 10}}, rotation = 270)));
    Modelica.Electrical.Polyphase.Basic.Star starM(final m = m) annotation(
      Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 180, origin = {-60, -10})));
    Modelica.Electrical.Machines.Sensors.VoltageQuasiRMSSensor voltageQuasiRMSSensor annotation(
      Placement(transformation(extent = {{-10, 10}, {10, -10}}, rotation = 180, origin = {-30, -10})));
    parameter Modelica.Electrical.Machines.Utilities.ParameterRecords.SM_PermanentMagnetData smpmData(useDamperCage = false) "Synchronous machine data" annotation(
      Placement(transformation(extent = {{-20, -80}, {0, -60}})));
    Modelica.Electrical.Machines.Sensors.CurrentQuasiRMSSensor currentQuasiRMSSensor annotation(
      Placement(transformation(origin = {-10, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 270)));
    Modelica.Electrical.Machines.Sensors.SinCosResolver sinCosResolver(p = 2) annotation(
      Placement(transformation(extent = {{-10, 10}, {10, -10}}, rotation = 270, origin = {30, -20})));
    Modelica.Electrical.Machines.Utilities.SinCosEvaluation sinCosEvaluation annotation(
      Placement(transformation(extent = {{10, -10}, {-10, 10}}, rotation = 270, origin = {30, 10})));
    CurrentController currentController(fsNominal = smpm.fsNominal, VsOpenCircuit = smpm.VsOpenCircuit, Rs = smpm.Rs, Ld = smpm.Lssigma + smpm.Lmd, Lq = smpm.Lssigma + smpm.Lmq, decoupling = true, useRMS = false) annotation(
      Placement(transformation(origin = {-42, 50}, extent = {{-10, -10}, {10, 10}})));
  initial equation
    smpm.is[1:2] = zeros(2);
  equation
    connect(star.pin_n, ground.p) annotation(
      Line(points = {{-70, 90}, {-80, 90}}, color = {0, 0, 255}));
    connect(terminalBox.plug_sn, smpm.plug_sn) annotation(
      Line(points = {{-16, -30}, {-16, -30}}, color = {0, 0, 255}));
    connect(terminalBox.plug_sp, smpm.plug_sp) annotation(
      Line(points = {{-4, -30}, {-4, -30}}, color = {0, 0, 255}));
    connect(quadraticSpeedDependentTorque.flange, inertiaLoad.flange_b) annotation(
      Line(points = {{80, -40}, {80, -40}}));
    connect(star.plug_p, signalVoltage.plug_n) annotation(
      Line(points = {{-50, 90}, {-10, 90}, {-10, 60}}, color = {0, 0, 255}));
    connect(currentSensor.plug_n, terminalBox.plugSupply) annotation(
      Line(points = {{-10, -10}, {-10, -28}}, color = {0, 0, 255}));
    connect(inertiaLoad.flange_a, multiSensor.flange_b) annotation(
      Line(points = {{60, -40}, {50, -40}}));
    connect(multiSensor.flange_a, smpm.flange) annotation(
      Line(points = {{30, -40}, {0, -40}}));
    connect(rotorDisplacementAngle.flange, smpm.flange) annotation(
      Line(points = {{10, -30}, {6, -30}, {6, -40}, {0, -40}}));
    connect(rotorDisplacementAngle.plug_p, smpm.plug_sp) annotation(
      Line(points = {{0, -26}, {6, -26}, {6, -30}, {-4, -30}}, color = {0, 0, 255}));
    connect(rotorDisplacementAngle.plug_n, smpm.plug_sn) annotation(
      Line(points = {{0, -14}, {0, -20}, {-16, -20}, {-16, -30}}, color = {0, 0, 255}));
    connect(voltageQuasiRMSSensor.plug_p, currentSensor.plug_n) annotation(
      Line(points = {{-20, -10}, {-10, -10}}, color = {0, 0, 255}));
    connect(starM.plug_p, voltageQuasiRMSSensor.plug_n) annotation(
      Line(points = {{-50, -10}, {-40, -10}}, color = {0, 0, 255}));
    connect(groundM.p, starM.pin_n) annotation(
      Line(points = {{-70, -28}, {-70, -10}}, color = {0, 0, 255}));
    connect(currentQuasiRMSSensor.plug_n, currentSensor.plug_p) annotation(
      Line(points = {{-10, 10}, {-10, 10}}, color = {0, 0, 255}));
    connect(signalVoltage.plug_p, currentQuasiRMSSensor.plug_p) annotation(
      Line(points = {{-10, 40}, {-10, 30}}, color = {0, 0, 255}));
    connect(smpm.flange, sinCosResolver.flange) annotation(
      Line(points = {{0, -40}, {30, -40}, {30, -30}}, color = {0, 0, 0}));
    connect(sinCosResolver.y, sinCosEvaluation.u) annotation(
      Line(points = {{30, -9}, {30, -2}}, color = {0, 0, 127}));
    connect(id.y, currentController.id) annotation(
      Line(points = {{-68, 70}, {-64, 70}, {-64, 56}, {-54, 56}}, color = {0, 0, 127}));
    connect(iq.y, currentController.iq) annotation(
      Line(points = {{-68, 30}, {-62, 30}, {-62, 44}, {-54, 44}}, color = {0, 0, 127}));
    connect(currentSensor.i, currentController.iActual) annotation(
      Line(points = {{-20, 0}, {-48, 0}, {-48, 38}}, color = {0, 0, 127}, thickness = 0.5));
    connect(sinCosEvaluation.phi, currentController.phi) annotation(
      Line(points = {{30, 22}, {30, 30}, {-36, 30}, {-36, 38}}, color = {0, 0, 127}));
    connect(currentController.y, signalVoltage.v) annotation(
      Line(points = {{-30, 50}, {-22, 50}}, color = {0, 0, 127}, thickness = 0.5));
    annotation(
      experiment(StopTime = 2.0, Interval = 1E-4, Tolerance = 1e-06),
      Documentation(info = "<html>
  <p>
  A synchronous machine with permanent magnets accelerates a quadratic speed dependent load from standstill.
  The rms values of d- and q-current in rotor fixed coordinate system are controlled by the dqCurrentController,
  and the output voltages fed to the machine. The result shows that the torque is influenced by the q-current,
  whereas the stator voltage is influenced by the d-current.</p>
  
  <p>Default machine parameters are used.</p>
  </html>"));
  end SMPM_VoltageSource2;

  model CurrentController2 "Current controller in dq coordinate system"
    import Modelica.Constants.pi;
    constant Integer m = 3 "Number of phases";
    parameter Integer p = 3 "Number of pole pairs";
    parameter Boolean useRMS = true "If true, inputs dq are multiplied by sqrt(2)";
    parameter Modelica.Units.SI.Frequency fsNominal "Nominal frequency";
    parameter Modelica.Units.SI.Voltage VsOpenCircuit "Open circuit RMS voltage per phase @ fsNominal";
    parameter Modelica.Units.SI.Resistance Rs "Stator resistance per phase";
    parameter Modelica.Units.SI.Inductance Ld "Inductance in d-axis";
    parameter Modelica.Units.SI.Inductance Lq "Inductance in q-axis";
    //Decoupling
    parameter Boolean decoupling = false "Use decoupling network";
    final parameter Modelica.Units.SI.MagneticFlux psiM = sqrt(2)*VsOpenCircuit/(2*pi*fsNominal) "Approximation of magnetic flux linkage";
    Modelica.Units.SI.AngularVelocity omega = p*der(phi);
    Modelica.Units.SI.Voltage Vd = sqrt(2)*(Rs*id - omega*Lq*iq);
    Modelica.Units.SI.Voltage Vq = sqrt(2)*(Rs*iq + omega*Ld*id) + omega*psiM;
    //Modelica.Units.SI.Voltage Vd = -omega*Lq*iq;
    //Modelica.Units.SI.Voltage Vq = omega*Ld*id + omega*psiM;
    extends Modelica.Blocks.Interfaces.MO(final nout = 3);
    Modelica.Blocks.Interfaces.RealOutput y[3] annotation(
      Placement(transformation(extent = {{100, -10}, {120, 10}}), iconTransformation(extent = {{100, -10}, {120, 10}})));
    Modelica.Blocks.Interfaces.RealInput id annotation(
      Placement(transformation(extent = {{-140, 40}, {-100, 80}}), iconTransformation(extent = {{-140, 40}, {-100, 80}})));
    Modelica.Blocks.Interfaces.RealInput iq annotation(
      Placement(transformation(extent = {{-140, -80}, {-100, -40}}), iconTransformation(extent = {{-140, -80}, {-100, -40}})));
    Modelica.Blocks.Interfaces.RealInput phi(unit = "rad") annotation(
      Placement(transformation(origin = {60, -120}, extent = {{20, -20}, {-20, 20}}, rotation = 270), iconTransformation(origin = {60, -120}, extent = {{20, -20}, {-20, 20}}, rotation = 270)));
    Modelica.Blocks.Interfaces.RealInput iActual[m](each unit = "A") annotation(
      Placement(transformation(origin = {-60, -120}, extent = {{20, -20}, {-20, 20}}, rotation = 270), iconTransformation(origin = {-60, -120}, extent = {{20, -20}, {-20, 20}}, rotation = 270)));
    Modelica.Blocks.Math.Gain toPeak_d(final k = if useRMS then sqrt(2) else 1) annotation(
      Placement(transformation(origin = {-70, 60}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Blocks.Math.Gain toPeak_q(final k = if useRMS then sqrt(2) else 1) annotation(
      Placement(transformation(origin = {-70, 0}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Blocks.Sources.RealExpression deCoupling[2](y = {Vd, Vq}) annotation(
      Placement(transformation(extent = {{-10, -40}, {10, -20}})));
    Transformation transformation(m = 3) annotation(
      Placement(transformation(origin = {-60, -54}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
    InverseTransformation inverse annotation(
      Placement(transformation(origin = {78, 0}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Electrical.Machines.Examples.ControlledDCDrives.Utilities.LimitedPI pi_d(useFF = true, Ti = Ld/Rs, yMax = VsOpenCircuit, initType = Modelica.Blocks.Types.Init.InitialOutput) annotation(
      Placement(transformation(origin = {2, 60}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Electrical.Machines.Examples.ControlledDCDrives.Utilities.LimitedPI pi_q(useFF = true, Ti = Ld/Rs, yMax = VsOpenCircuit, initType = Modelica.Blocks.Types.Init.InitialOutput) annotation(
      Placement(transformation(origin = {2, 0}, extent = {{-10, -10}, {10, 10}})));
  protected
    constant Modelica.Units.SI.Resistance unitResistance = 1 annotation(
      Placement(visible = false, transformation(extent = {{0, 0}, {0, 0}})));
  equation
    connect(toPeak_d.u, id) annotation(
      Line(points = {{-82, 60}, {-120, 60}}, color = {0, 0, 127}));
    connect(toPeak_q.u, iq) annotation(
      Line(points = {{-82, 0}, {-90, 0}, {-90, -60}, {-120, -60}}, color = {0, 0, 127}));
    connect(transformation.u, iActual) annotation(
      Line(points = {{-60, -66}, {-60, -120}}, color = {0, 0, 127}, thickness = 0.5));
    connect(phi, transformation.angle) annotation(
      Line(points = {{60, -120}, {60, -54}, {-48, -54}}, color = {0, 0, 127}));
    connect(inverse.y, y) annotation(
      Line(points = {{90, 0}, {110, 0}}, color = {0, 0, 127}, thickness = 0.5));
    connect(inverse.angle, phi) annotation(
      Line(points = {{78, -12}, {60, -12}, {60, -120}}, color = {0, 0, 127}));
    connect(transformation.y[1], pi_d.u_m) annotation(
      Line(points = {{-60, -42}, {-60, -12}, {-42, -12}, {-42, 36}, {-4, 36}, {-4, 48}}, color = {0, 0, 127}));
    connect(toPeak_d.y, pi_d.u) annotation(
      Line(points = {{-58, 60}, {-10, 60}}, color = {0, 0, 127}));
    connect(deCoupling[1].y, pi_d.feedForward) annotation(
      Line(points = {{12, -30}, {30, -30}, {30, 48}, {2, 48}}, color = {0, 0, 127}));
    connect(pi_d.y, inverse.u[1]) annotation(
      Line(points = {{14, 60}, {60, 60}, {60, 0}, {66, 0}}, color = {0, 0, 127}));
    connect(transformation.y[2], pi_q.u_m) annotation(
      Line(points = {{-60, -42}, {-60, -20}, {-4, -20}, {-4, -12}}, color = {0, 0, 127}));
    connect(toPeak_q.y, pi_q.u) annotation(
      Line(points = {{-58, 0}, {-10, 0}}, color = {0, 0, 127}));
    connect(pi_q.y, inverse.u[2]) annotation(
      Line(points = {{14, 0}, {66, 0}}, color = {0, 0, 127}));
    connect(deCoupling[2].y, pi_q.feedForward) annotation(
      Line(points = {{12, -30}, {18, -30}, {18, -18}, {2, -18}, {2, -12}}, color = {0, 0, 127}));
  end CurrentController2;

  model SMPM_VoltageSource3 "Test example: PermanentMagnetSynchronousMachine fed by FOC"
    extends Modelica.Icons.Example;
    import Modelica.Constants.pi;
    constant Integer m = 3 "Number of phases";
    parameter Modelica.Units.SI.Current Idq[2] = {-53.5, 84.6} "Desired d- and q-current";
    parameter Modelica.Units.SI.AngularVelocity wNominal = 2*pi*smpmData.fsNominal/smpmData.p "Nominal speed";
    parameter Modelica.Units.SI.Torque TLoad = 181.4 "Nominal load torque";
    parameter Modelica.Units.SI.Inertia JLoad = 0.29 "Load's moment of inertia";
    Modelica.Electrical.Machines.BasicMachines.SynchronousMachines.SM_PermanentMagnet smpm(phiMechanical(start = 0, fixed = true), wMechanical(start = 0, fixed = true), useSupport = false, useThermalPort = false, p = smpmData.p, fsNominal = smpmData.fsNominal, Rs = smpmData.Rs, TsRef = smpmData.TsRef, Lszero = smpmData.Lszero, Lssigma = smpmData.Lssigma, Jr = smpmData.Jr, Js = smpmData.Js, frictionParameters = smpmData.frictionParameters, statorCoreParameters = smpmData.statorCoreParameters, strayLoadParameters = smpmData.strayLoadParameters, VsOpenCircuit = smpmData.VsOpenCircuit, Lmd = smpmData.Lmd, Lmq = smpmData.Lmq, useDamperCage = smpmData.useDamperCage, Lrsigmad = smpmData.Lrsigmad, Lrsigmaq = smpmData.Lrsigmaq, Rrd = smpmData.Rrd, Rrq = smpmData.Rrq, TrRef = smpmData.TrRef, permanentMagnetLossParameters = smpmData.permanentMagnetLossParameters, TsOperational = 293.15, alpha20s = smpmData.alpha20s, TrOperational = 293.15, alpha20r = smpmData.alpha20r) annotation(
      Placement(transformation(extent = {{-20, -50}, {0, -30}})));
    Modelica.Electrical.Polyphase.Sources.SignalVoltage signalVoltage(final m = m) annotation(
      Placement(transformation(origin = {-10, 50}, extent = {{10, 10}, {-10, -10}}, rotation = 270)));
    Modelica.Electrical.Polyphase.Basic.Star star(final m = m) annotation(
      Placement(transformation(extent = {{-50, 80}, {-70, 100}})));
    Modelica.Electrical.Analog.Basic.Ground ground annotation(
      Placement(transformation(origin = {-90, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 270)));
    Modelica.Blocks.Sources.Constant iq(k = 10) annotation(
      Placement(transformation(extent = {{-90, 20}, {-70, 40}})));
    Modelica.Blocks.Sources.Constant id(k = 0) annotation(
      Placement(transformation(extent = {{-90, 60}, {-70, 80}})));
    Modelica.Electrical.Machines.Utilities.TerminalBox terminalBox(terminalConnection = "Y") annotation(
      Placement(transformation(extent = {{-20, -34}, {0, -14}})));
    Modelica.Mechanics.Rotational.Components.Inertia inertiaLoad(J = JLoad) annotation(
      Placement(transformation(extent = {{60, -50}, {80, -30}})));
    Modelica.Mechanics.Rotational.Sources.QuadraticSpeedDependentTorque quadraticSpeedDependentTorque(tau_nominal = -TLoad, w_nominal(displayUnit = "rad/s") = wNominal) annotation(
      Placement(transformation(extent = {{100, -50}, {80, -30}})));
    Modelica.Electrical.Polyphase.Sensors.CurrentSensor currentSensor(m = m) annotation(
      Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 270, origin = {-10, 0})));
    Modelica.Electrical.Machines.Utilities.DQCurrentController dqCurrentController(p = smpm.p, Ld = smpm.Lssigma + smpm.Lmd, Lq = smpm.Lssigma + smpm.Lmq, Rs = Modelica.Electrical.Machines.Thermal.convertResistance(smpm.Rs, smpm.TsRef, smpm.alpha20s, smpm.TsOperational), fsNominal = smpm.fsNominal, VsOpenCircuit = smpm.VsOpenCircuit) annotation(
      Placement(transformation(extent = {{-50, 40}, {-30, 60}})));
    Modelica.Mechanics.Rotational.Sensors.MultiSensor multiSensor annotation(
      Placement(transformation(extent = {{10, 10}, {-10, -10}}, rotation = 180, origin = {40, -40})));
    Modelica.Electrical.Machines.Sensors.RotorDisplacementAngle rotorDisplacementAngle(p = smpm.p) annotation(
      Placement(transformation(origin = {10, -20}, extent = {{-10, 10}, {10, -10}})));
    Modelica.Electrical.Analog.Basic.Ground groundM annotation(
      Placement(transformation(origin = {-80, -28}, extent = {{-10, -10}, {10, 10}}, rotation = 270)));
    Modelica.Electrical.Polyphase.Basic.Star starM(final m = m) annotation(
      Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 180, origin = {-60, -10})));
    Modelica.Electrical.Machines.Sensors.VoltageQuasiRMSSensor voltageQuasiRMSSensor annotation(
      Placement(transformation(extent = {{-10, 10}, {10, -10}}, rotation = 180, origin = {-30, -10})));
    parameter Modelica.Electrical.Machines.Utilities.ParameterRecords.SM_PermanentMagnetData smpmData(useDamperCage = false) "Synchronous machine data" annotation(
      Placement(transformation(extent = {{-20, -80}, {0, -60}})));
    Modelica.Electrical.Machines.Sensors.CurrentQuasiRMSSensor currentQuasiRMSSensor annotation(
      Placement(transformation(origin = {-10, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 270)));
    Modelica.Electrical.Machines.Sensors.SinCosResolver sinCosResolver(p = 1) annotation(
      Placement(transformation(extent = {{-10, 10}, {10, -10}}, rotation = 270, origin = {30, -20})));
    Modelica.Electrical.Machines.Utilities.SinCosEvaluation sinCosEvaluation annotation(
      Placement(transformation(extent = {{10, -10}, {-10, 10}}, rotation = 270, origin = {30, 10})));
  initial equation
    smpm.is[1:2] = zeros(2);
  equation
    connect(star.pin_n, ground.p) annotation(
      Line(points = {{-70, 90}, {-80, 90}}, color = {0, 0, 255}));
    connect(terminalBox.plug_sn, smpm.plug_sn) annotation(
      Line(points = {{-16, -30}, {-16, -30}}, color = {0, 0, 255}));
    connect(terminalBox.plug_sp, smpm.plug_sp) annotation(
      Line(points = {{-4, -30}, {-4, -30}}, color = {0, 0, 255}));
    connect(quadraticSpeedDependentTorque.flange, inertiaLoad.flange_b) annotation(
      Line(points = {{80, -40}, {80, -40}}));
    connect(star.plug_p, signalVoltage.plug_n) annotation(
      Line(points = {{-50, 90}, {-10, 90}, {-10, 60}}, color = {0, 0, 255}));
    connect(currentSensor.plug_n, terminalBox.plugSupply) annotation(
      Line(points = {{-10, -10}, {-10, -28}}, color = {0, 0, 255}));
    connect(id.y, dqCurrentController.id) annotation(
      Line(points = {{-69, 70}, {-60, 70}, {-60, 56}, {-52, 56}}, color = {0, 0, 127}));
    connect(iq.y, dqCurrentController.iq) annotation(
      Line(points = {{-69, 30}, {-60, 30}, {-60, 44}, {-52, 44}}, color = {0, 0, 127}));
    connect(dqCurrentController.y, signalVoltage.v) annotation(
      Line(points = {{-29, 50}, {-22, 50}}, color = {0, 0, 127}));
    connect(currentSensor.i, dqCurrentController.iActual) annotation(
      Line(points = {{-21, 0}, {-46, 0}, {-46, 38}}, color = {0, 0, 127}));
    connect(inertiaLoad.flange_a, multiSensor.flange_b) annotation(
      Line(points = {{60, -40}, {50, -40}}));
    connect(multiSensor.flange_a, smpm.flange) annotation(
      Line(points = {{30, -40}, {0, -40}}));
    connect(rotorDisplacementAngle.flange, smpm.flange) annotation(
      Line(points = {{10, -30}, {6, -30}, {6, -40}, {0, -40}}));
    connect(rotorDisplacementAngle.plug_p, smpm.plug_sp) annotation(
      Line(points = {{0, -26}, {6, -26}, {6, -30}, {-4, -30}}, color = {0, 0, 255}));
    connect(rotorDisplacementAngle.plug_n, smpm.plug_sn) annotation(
      Line(points = {{0, -14}, {0, -20}, {-16, -20}, {-16, -30}}, color = {0, 0, 255}));
    connect(voltageQuasiRMSSensor.plug_p, currentSensor.plug_n) annotation(
      Line(points = {{-20, -10}, {-10, -10}}, color = {0, 0, 255}));
    connect(starM.plug_p, voltageQuasiRMSSensor.plug_n) annotation(
      Line(points = {{-50, -10}, {-40, -10}}, color = {0, 0, 255}));
    connect(groundM.p, starM.pin_n) annotation(
      Line(points = {{-70, -28}, {-70, -10}}, color = {0, 0, 255}));
    connect(currentQuasiRMSSensor.plug_n, currentSensor.plug_p) annotation(
      Line(points = {{-10, 10}, {-10, 10}}, color = {0, 0, 255}));
    connect(signalVoltage.plug_p, currentQuasiRMSSensor.plug_p) annotation(
      Line(points = {{-10, 40}, {-10, 30}}, color = {0, 0, 255}));
    connect(smpm.flange, sinCosResolver.flange) annotation(
      Line(points = {{0, -40}, {30, -40}, {30, -30}}, color = {0, 0, 0}));
    connect(sinCosResolver.y, sinCosEvaluation.u) annotation(
      Line(points = {{30, -9}, {30, -2}}, color = {0, 0, 127}));
    connect(sinCosEvaluation.phi, dqCurrentController.phi) annotation(
      Line(points = {{30, 21}, {30, 34}, {-34, 34}, {-34, 38}}, color = {0, 0, 127}));
    annotation(
      experiment(StopTime = 2.0, Interval = 1E-4, Tolerance = 1e-06),
      Documentation(info = "<html>
  <p>
  A synchronous machine with permanent magnets accelerates a quadratic speed dependent load from standstill.
  The rms values of d- and q-current in rotor fixed coordinate system are controlled by the dqCurrentController,
  and the output voltages fed to the machine. The result shows that the torque is influenced by the q-current,
  whereas the stator voltage is influenced by the d-current.</p>
  
  <p>Default machine parameters are used.</p>
  </html>"));
  end SMPM_VoltageSource3;

  block Antiwindup
    Modelica.Blocks.Interfaces.RealInput signal annotation(
      Placement(transformation(origin = {-108, 60}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-108, 60}, extent = {{-20, -20}, {20, 20}})));
    Modelica.Blocks.Interfaces.RealInput saturation annotation(
      Placement(transformation(origin = {-108, 0}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-108, 0}, extent = {{-20, -20}, {20, 20}})));
    Modelica.Blocks.Interfaces.RealInput error annotation(
      Placement(transformation(origin = {-108, -60}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-108, -60}, extent = {{-20, -20}, {20, 20}})));
    Modelica.Blocks.Interfaces.RealOutput y annotation(
      Placement(transformation(origin = {108, 0}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {108, 0}, extent = {{-10, -10}, {10, 10}})));
  equation
    if signal < saturation then
      y = error;
    else
      y = 0;
    end if;
  end Antiwindup;

  model CurrentController3 "Current controller in dq coordinate system"
    import Modelica.Constants.pi;
    constant Integer m = 3 "Number of phases";
    parameter Integer p = 3 "Number of pole pairs";
    parameter Boolean useRMS = true "If true, inputs dq are multiplied by sqrt(2)";
    parameter Modelica.Units.SI.Frequency fsNominal "Nominal frequency";
    parameter Modelica.Units.SI.Voltage VsOpenCircuit "Open circuit RMS voltage per phase @ fsNominal";
    parameter Modelica.Units.SI.Resistance Rs "Stator resistance per phase";
    parameter Modelica.Units.SI.Inductance Ld "Inductance in d-axis";
    parameter Modelica.Units.SI.Inductance Lq "Inductance in q-axis";
    //Decoupling
    parameter Boolean decoupling = false "Use decoupling network";
    final parameter Modelica.Units.SI.MagneticFlux psiM = sqrt(2)*VsOpenCircuit/(2*pi*fsNominal) "Approximation of magnetic flux linkage";
    Modelica.Units.SI.AngularVelocity omega = p*der(phi);
    Modelica.Units.SI.Voltage Vd = sqrt(2)*(Rs*id - omega*Lq*iq);
    Modelica.Units.SI.Voltage Vq = sqrt(2)*(Rs*iq + omega*Ld*id) + omega*psiM;
    //Modelica.Units.SI.Voltage Vd = -omega*Lq*iq;
    //Modelica.Units.SI.Voltage Vq = omega*Ld*id + omega*psiM;
    extends Modelica.Blocks.Interfaces.MO(final nout = 3);
    Modelica.Blocks.Interfaces.RealOutput y[3] annotation(
      Placement(transformation(extent = {{100, -10}, {120, 10}}), iconTransformation(extent = {{100, -10}, {120, 10}})));
    Modelica.Blocks.Interfaces.RealInput id annotation(
      Placement(transformation(extent = {{-140, 40}, {-100, 80}}), iconTransformation(extent = {{-140, 40}, {-100, 80}})));
    Modelica.Blocks.Interfaces.RealInput iq annotation(
      Placement(transformation(extent = {{-140, -80}, {-100, -40}}), iconTransformation(extent = {{-140, -80}, {-100, -40}})));
    Modelica.Blocks.Interfaces.RealInput phi(unit = "rad") annotation(
      Placement(transformation(origin = {60, -120}, extent = {{20, -20}, {-20, 20}}, rotation = 270), iconTransformation(origin = {60, -120}, extent = {{20, -20}, {-20, 20}}, rotation = 270)));
    Modelica.Blocks.Interfaces.RealInput iActual[m](each unit = "A") annotation(
      Placement(transformation(origin = {-60, -120}, extent = {{20, -20}, {-20, 20}}, rotation = 270), iconTransformation(origin = {-60, -120}, extent = {{20, -20}, {-20, 20}}, rotation = 270)));
    Modelica.Blocks.Math.Gain toPeak_d(final k = if useRMS then sqrt(2) else 1) annotation(
      Placement(transformation(origin = {-70, 60}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Blocks.Math.Gain toPeak_q(final k = if useRMS then sqrt(2) else 1) annotation(
      Placement(transformation(origin = {-70, 0}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Blocks.Sources.RealExpression deCoupling[2](y = {Vd, Vq}) annotation(
      Placement(transformation(extent = {{-10, -40}, {10, -20}})));
    Modelica.Electrical.Machines.Examples.ControlledDCDrives.Utilities.LimitedPI pi_d(useFF = true, Ti = Ld/Rs, yMax = VsOpenCircuit, initType = Modelica.Blocks.Types.Init.InitialOutput) annotation(
      Placement(transformation(origin = {2, 60}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Electrical.Machines.Examples.ControlledDCDrives.Utilities.LimitedPI pi_q(useFF = true, Ti = Ld/Rs, yMax = VsOpenCircuit, initType = Modelica.Blocks.Types.Init.InitialOutput) annotation(
      Placement(transformation(origin = {2, 0}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Electrical.Machines.Utilities.ToDQ toDQ(p = p) annotation(
      Placement(transformation(origin = {-60, -52}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
    Modelica.Electrical.Machines.Utilities.FromDQ fromDQ(p = p) annotation(
      Placement(transformation(origin = {66, 0}, extent = {{-10, -10}, {10, 10}})));
  protected
    constant Modelica.Units.SI.Resistance unitResistance = 1 annotation(
      Placement(visible = false, transformation(extent = {{0, 0}, {0, 0}})));
  equation
    connect(toPeak_d.u, id) annotation(
      Line(points = {{-82, 60}, {-120, 60}}, color = {0, 0, 127}));
    connect(toPeak_q.u, iq) annotation(
      Line(points = {{-82, 0}, {-90, 0}, {-90, -60}, {-120, -60}}, color = {0, 0, 127}));
    connect(toPeak_d.y, pi_d.u) annotation(
      Line(points = {{-58, 60}, {-10, 60}}, color = {0, 0, 127}));
    connect(deCoupling[1].y, pi_d.feedForward) annotation(
      Line(points = {{12, -30}, {30, -30}, {30, 48}, {2, 48}}, color = {0, 0, 127}));
    connect(toPeak_q.y, pi_q.u) annotation(
      Line(points = {{-58, 0}, {-10, 0}}, color = {0, 0, 127}));
    connect(deCoupling[2].y, pi_q.feedForward) annotation(
      Line(points = {{12, -30}, {18, -30}, {18, -18}, {2, -18}, {2, -12}}, color = {0, 0, 127}));
    connect(phi, toDQ.phi) annotation(
      Line(points = {{60, -120}, {60, -52}, {-48, -52}}, color = {0, 0, 127}));
    connect(toDQ.u, iActual) annotation(
      Line(points = {{-60, -64}, {-60, -120}}, color = {0, 0, 127}, thickness = 0.5));
    connect(toDQ.y[1], pi_d.u_m) annotation(
      Line(points = {{-60, -40}, {-60, -20}, {-34, -20}, {-34, 32}, {-4, 32}, {-4, 48}}, color = {0, 0, 127}));
    connect(toDQ.y[2], pi_q.u_m) annotation(
      Line(points = {{-60, -40}, {-60, -28}, {-22, -28}, {-22, -20}, {-4, -20}, {-4, -12}}, color = {0, 0, 127}));
    connect(fromDQ.y, y) annotation(
      Line(points = {{78, 0}, {110, 0}}, color = {0, 0, 127}, thickness = 0.5));
    connect(phi, fromDQ.phi) annotation(
      Line(points = {{60, -120}, {60, -32}, {66, -32}, {66, -12}}, color = {0, 0, 127}));
    connect(pi_q.y, fromDQ.u[2]) annotation(
      Line(points = {{14, 0}, {54, 0}}, color = {0, 0, 127}));
    connect(pi_d.y, fromDQ.u[1]) annotation(
      Line(points = {{14, 60}, {40, 60}, {40, 0}, {54, 0}}, color = {0, 0, 127}));
  end CurrentController3;

  model VoltageVectorMagnitude
    parameter Modelica.Units.SI.Voltage Vdc = 600 "DC bus voltage";
    // Clarke Transform Variables
    Real Valpha;
    Real Vbeta;
    Modelica.Blocks.Interfaces.RealInput u[3] annotation(
      Placement(transformation(origin = {-108, 0}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-108, 0}, extent = {{-20, -20}, {20, 20}})));
    Modelica.Blocks.Interfaces.RealOutput y annotation(
      Placement(transformation(origin = {108, 0}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {108, 0}, extent = {{-10, -10}, {10, 10}})));
  equation
// Clarke Transformation
    Valpha = (2/3)*(u[1] - Vdc/2 - 0.5*(u[2] - Vdc/2) - 0.5*(u[3] - Vdc/2));
    Vbeta = (sqrt(3)/3)*((u[2] - Vdc/2) - (u[3] - Vdc/2));
// Voltage Vector Magnitude Calculation
    y = sqrt(3)*sqrt(Valpha^2 + Vbeta^2)/Vdc;
    annotation(
      Diagram(graphics = {Text(origin = {-9, 10}, extent = {{-23, 14}, {23, -14}}, textString = "\\rho")}));
  end VoltageVectorMagnitude;

  model SMPM_VoltageSource "Test example: PermanentMagnetSynchronousMachine fed by FOC"
    extends Modelica.Icons.Example;
    import Modelica.Constants.pi;
    constant Integer m = 3 "Number of phases";
    parameter Modelica.Units.SI.Current Idq[2] = {-53.5, 84.6} "Desired d- and q-current";
    parameter Modelica.Units.SI.AngularVelocity wNominal = 2*pi*smpmData.fsNominal/smpmData.p "Nominal speed";
    parameter Modelica.Units.SI.Torque TLoad = 181.4 "Nominal load torque";
    parameter Modelica.Units.SI.Inertia JLoad = 0.29 "Load's moment of inertia";
    Modelica.Electrical.Machines.BasicMachines.SynchronousMachines.SM_PermanentMagnet smpm(phiMechanical(start = 0, fixed = true), wMechanical(start = 0, fixed = true), useSupport = false, useThermalPort = false, p = smpmData.p, fsNominal = smpmData.fsNominal, Rs = smpmData.Rs, TsRef = smpmData.TsRef, Lszero = smpmData.Lszero, Lssigma = smpmData.Lssigma, Jr = smpmData.Jr, Js = smpmData.Js, frictionParameters = smpmData.frictionParameters, statorCoreParameters = smpmData.statorCoreParameters, strayLoadParameters = smpmData.strayLoadParameters, VsOpenCircuit = smpmData.VsOpenCircuit, Lmd = smpmData.Lmd, Lmq = smpmData.Lmq, useDamperCage = smpmData.useDamperCage, Lrsigmad = smpmData.Lrsigmad, Lrsigmaq = smpmData.Lrsigmaq, Rrd = smpmData.Rrd, Rrq = smpmData.Rrq, TrRef = smpmData.TrRef, permanentMagnetLossParameters = smpmData.permanentMagnetLossParameters, TsOperational = 293.15, alpha20s = smpmData.alpha20s, TrOperational = 293.15, alpha20r = smpmData.alpha20r) annotation(
      Placement(transformation(extent = {{-20, -50}, {0, -30}})));
    Modelica.Electrical.Polyphase.Sources.SignalVoltage signalVoltage(final m = m) annotation(
      Placement(transformation(origin = {-10, 50}, extent = {{10, 10}, {-10, -10}}, rotation = 270)));
    Modelica.Electrical.Polyphase.Basic.Star star(final m = m) annotation(
      Placement(transformation(extent = {{-50, 80}, {-70, 100}})));
    Modelica.Electrical.Analog.Basic.Ground ground annotation(
      Placement(transformation(origin = {-90, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 270)));
    Modelica.Blocks.Sources.Constant iq(k = 60) annotation(
      Placement(transformation(extent = {{-90, 20}, {-70, 40}})));
    Modelica.Blocks.Sources.Constant id(k = -50) annotation(
      Placement(transformation(extent = {{-90, 60}, {-70, 80}})));
    Modelica.Electrical.Machines.Utilities.TerminalBox terminalBox(terminalConnection = "Y") annotation(
      Placement(transformation(extent = {{-20, -34}, {0, -14}})));
    Modelica.Mechanics.Rotational.Components.Inertia inertiaLoad(J = JLoad) annotation(
      Placement(transformation(extent = {{60, -50}, {80, -30}})));
    Modelica.Mechanics.Rotational.Sources.QuadraticSpeedDependentTorque quadraticSpeedDependentTorque(tau_nominal = -TLoad, w_nominal(displayUnit = "rad/s") = wNominal) annotation(
      Placement(transformation(extent = {{100, -50}, {80, -30}})));
    Modelica.Electrical.Polyphase.Sensors.CurrentSensor currentSensor(m = m) annotation(
      Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 270, origin = {-10, 0})));
    Modelica.Electrical.Machines.Utilities.DQCurrentController dqCurrentController(p = smpm.p, Ld = smpm.Lssigma + smpm.Lmd, Lq = smpm.Lssigma + smpm.Lmq, Rs = Modelica.Electrical.Machines.Thermal.convertResistance(smpm.Rs, smpm.TsRef, smpm.alpha20s, smpm.TsOperational), fsNominal = smpm.fsNominal, VsOpenCircuit = smpm.VsOpenCircuit) annotation(
      Placement(transformation(extent = {{-50, 40}, {-30, 60}})));
    Modelica.Mechanics.Rotational.Sensors.MultiSensor multiSensor annotation(
      Placement(transformation(extent = {{10, 10}, {-10, -10}}, rotation = 180, origin = {40, -40})));
    Modelica.Electrical.Machines.Sensors.RotorDisplacementAngle rotorDisplacementAngle(p = smpm.p) annotation(
      Placement(transformation(origin = {10, -20}, extent = {{-10, 10}, {10, -10}})));
    Modelica.Electrical.Analog.Basic.Ground groundM annotation(
      Placement(transformation(origin = {-80, -28}, extent = {{-10, -10}, {10, 10}}, rotation = 270)));
    Modelica.Electrical.Polyphase.Basic.Star starM(final m = m) annotation(
      Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 180, origin = {-60, -10})));
    Modelica.Electrical.Machines.Sensors.VoltageQuasiRMSSensor voltageQuasiRMSSensor annotation(
      Placement(transformation(extent = {{-10, 10}, {10, -10}}, rotation = 180, origin = {-30, -10})));
    parameter Modelica.Electrical.Machines.Utilities.ParameterRecords.SM_PermanentMagnetData smpmData(useDamperCage = false) "Synchronous machine data" annotation(
      Placement(transformation(extent = {{-20, -80}, {0, -60}})));
    Modelica.Electrical.Machines.Sensors.CurrentQuasiRMSSensor currentQuasiRMSSensor annotation(
      Placement(transformation(origin = {-10, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 270)));
    Modelica.Electrical.Machines.Sensors.SinCosResolver sinCosResolver(p = 1) annotation(
      Placement(transformation(extent = {{-10, 10}, {10, -10}}, rotation = 270, origin = {30, -20})));
    Modelica.Electrical.Machines.Utilities.SinCosEvaluation sinCosEvaluation annotation(
      Placement(transformation(extent = {{10, -10}, {-10, 10}}, rotation = 270, origin = {30, 10})));
  initial equation
    smpm.is[1:2] = zeros(2);
  equation
    connect(star.pin_n, ground.p) annotation(
      Line(points = {{-70, 90}, {-80, 90}}, color = {0, 0, 255}));
    connect(terminalBox.plug_sn, smpm.plug_sn) annotation(
      Line(points = {{-16, -30}, {-16, -30}}, color = {0, 0, 255}));
    connect(terminalBox.plug_sp, smpm.plug_sp) annotation(
      Line(points = {{-4, -30}, {-4, -30}}, color = {0, 0, 255}));
    connect(quadraticSpeedDependentTorque.flange, inertiaLoad.flange_b) annotation(
      Line(points = {{80, -40}, {80, -40}}));
    connect(star.plug_p, signalVoltage.plug_n) annotation(
      Line(points = {{-50, 90}, {-10, 90}, {-10, 60}}, color = {0, 0, 255}));
    connect(currentSensor.plug_n, terminalBox.plugSupply) annotation(
      Line(points = {{-10, -10}, {-10, -28}}, color = {0, 0, 255}));
    connect(id.y, dqCurrentController.id) annotation(
      Line(points = {{-69, 70}, {-60, 70}, {-60, 56}, {-52, 56}}, color = {0, 0, 127}));
    connect(iq.y, dqCurrentController.iq) annotation(
      Line(points = {{-69, 30}, {-60, 30}, {-60, 44}, {-52, 44}}, color = {0, 0, 127}));
    connect(dqCurrentController.y, signalVoltage.v) annotation(
      Line(points = {{-29, 50}, {-22, 50}}, color = {0, 0, 127}));
    connect(currentSensor.i, dqCurrentController.iActual) annotation(
      Line(points = {{-21, 0}, {-46, 0}, {-46, 38}}, color = {0, 0, 127}));
    connect(inertiaLoad.flange_a, multiSensor.flange_b) annotation(
      Line(points = {{60, -40}, {50, -40}}));
    connect(multiSensor.flange_a, smpm.flange) annotation(
      Line(points = {{30, -40}, {0, -40}}));
    connect(rotorDisplacementAngle.flange, smpm.flange) annotation(
      Line(points = {{10, -30}, {6, -30}, {6, -40}, {0, -40}}));
    connect(rotorDisplacementAngle.plug_p, smpm.plug_sp) annotation(
      Line(points = {{0, -26}, {6, -26}, {6, -30}, {-4, -30}}, color = {0, 0, 255}));
    connect(rotorDisplacementAngle.plug_n, smpm.plug_sn) annotation(
      Line(points = {{0, -14}, {0, -20}, {-16, -20}, {-16, -30}}, color = {0, 0, 255}));
    connect(voltageQuasiRMSSensor.plug_p, currentSensor.plug_n) annotation(
      Line(points = {{-20, -10}, {-10, -10}}, color = {0, 0, 255}));
    connect(starM.plug_p, voltageQuasiRMSSensor.plug_n) annotation(
      Line(points = {{-50, -10}, {-40, -10}}, color = {0, 0, 255}));
    connect(groundM.p, starM.pin_n) annotation(
      Line(points = {{-70, -28}, {-70, -10}}, color = {0, 0, 255}));
    connect(currentQuasiRMSSensor.plug_n, currentSensor.plug_p) annotation(
      Line(points = {{-10, 10}, {-10, 10}}, color = {0, 0, 255}));
    connect(signalVoltage.plug_p, currentQuasiRMSSensor.plug_p) annotation(
      Line(points = {{-10, 40}, {-10, 30}}, color = {0, 0, 255}));
    connect(smpm.flange, sinCosResolver.flange) annotation(
      Line(points = {{0, -40}, {30, -40}, {30, -30}}, color = {0, 0, 0}));
    connect(sinCosResolver.y, sinCosEvaluation.u) annotation(
      Line(points = {{30, -9}, {30, -2}}, color = {0, 0, 127}));
    connect(sinCosEvaluation.phi, dqCurrentController.phi) annotation(
      Line(points = {{30, 21}, {30, 34}, {-34, 34}, {-34, 38}}, color = {0, 0, 127}));
    annotation(
      experiment(StopTime = 2.0, Interval = 1E-4, Tolerance = 1e-06),
      Documentation(info = "<html>
  <p>
  A synchronous machine with permanent magnets accelerates a quadratic speed dependent load from standstill.
  The rms values of d- and q-current in rotor fixed coordinate system are controlled by the dqCurrentController,
  and the output voltages fed to the machine. The result shows that the torque is influenced by the q-current,
  whereas the stator voltage is influenced by the d-current.</p>
  
  <p>Default machine parameters are used.</p>
  </html>"));
  end SMPM_VoltageSource;
  annotation(
    uses(Modelica(version = "4.0.0"), PowerSystems(version = "2.0.0")));
end FOC;
