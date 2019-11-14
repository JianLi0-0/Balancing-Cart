#!/usr/bin/env wolframscript
(* ::Package:: *)

(eqns={
(2 Iw+(Mr1+Mr2+2 Mw) R^2) (\[Theta]^\[Prime]\[Prime])[t]+R(L Mr1+Mr2 R1)(\[CurlyPhi]^\[Prime]\[Prime])[t]+R H Mr2 (\[Phi]^\[Prime]\[Prime])[t]+Cw1 Derivative[1][\[Theta]][t]-Cw1 Derivative[1][\[CurlyPhi]][t]==2 Km u[t],R (L Mr1+Mr2 R1) (\[Theta]^\[Prime]\[Prime])[t]+(Ir1+L^2 Mr1+Mr2 R1^2) (\[CurlyPhi]^\[Prime]\[Prime])[t]+H Mr2 R1 (\[Phi]^\[Prime]\[Prime])[t]-Cw1 Derivative[1][\[Theta]][t]+Cw1 Derivative[1][\[CurlyPhi]][t]-g (L Mr1+Mr2 R1) \[CurlyPhi][t]==-2 Km u[t],
H Mr2 R (\[Theta]^\[Prime]\[Prime])[t]+H Mr2 R1 (\[CurlyPhi]^\[Prime]\[Prime])[t]+(H Mr2 H+Ir2 )(\[Phi]^\[Prime]\[Prime])[t]+C12 Derivative[1][\[Phi]][t]-g H Mr2 \[Phi][t]==0
})



g = 9.8;
Km = 0.3*187/3591; (*%\:8f6c\:77e9\:5e38\:6570 N*m/A*)
R = 0.095/2;(*%\:8f6e\:5b50\:534a\:5f84 m*)
Mw = 0.134;
Iw = 0.000020;
T = 0.005;(*%\:63a7\:5236\:5468\:671f*)
Cw1 = 0.000098;(*%\:7535\:673a\:963b\:5c3c\:7cfb\:6570*)

Mr1 = 0.821;(*\:7b2c\:4e00\:7ea7\:6446\:6746\:8d28\:91cf\:ff0c\:5373\:5c0f\:8f66\:5e95\:76d8\:8d28\:91cf*)
Ir1 = 0.000578;(*\:7b2c\:4e00\:7ea7\:6446\:6746\:8f6c\:52a8\:60ef\:91cf*)(*\:5f85\:8fa8\:8bc6*)
L = 0.006950;(*\:7b2c\:4e00\:7ea7\:6446\:6746\:8d28\:5fc3\:8ddd\:96f6\:52bf\:80fd\:9762\:9ad8\:5ea6*)(*\:5f85\:8fa8\:8bc6*)

Mr2 = 0.086;(*\:7b2c\:4e8c\:7ea7\:6446\:6746\:8d28\:91cf*)
R1 = 0.147;(*\:7b2c\:4e00\:7ea7\:6446\:6746\:957f\:5ea6\:ff0c\:5373\:7535\:673a\:8f74\:5230\:7535\:4f4d\:5668\:7684\:8ddd\:79bb*)
H = 0.178;(*\:7b2c\:4e8c\:7ea7\:6446\:6746\:8d28\:5fc3\:8ddd\:7535\:4f4d\:5668\:65cb\:8f6c\:8f74\:7684\:8ddd\:79bb*)
Ir2 = 0.00355;(*\:7b2c\:4e8c\:7ea7\:6446\:6746\:76f8\:5bf9\:7535\:4f4d\:5668\:65cb\:8f6c\:8f74\:8f6c\:52a8\:60ef\:91cf*)
C12 = 0.000005;(*%\:7535\:4f4d\:5668\:963b\:5c3c\:7cfb\:6570*)

model=StateSpaceModel[eqns,
{{\[Theta][t],0},{\[CurlyPhi][t],0},{\[Phi][t],0},{\[Theta]'[t],0},{\[CurlyPhi]'[t],0},{\[Phi]'[t],0}},
u[t],
{},
t]

Normal[model]















(* ::InheritFromParent:: *)
(*StateSpaceModel[{{{0.,0.,0.,1.,0.,0.},{0.,0.,0.,0.,1.,0.},{0.,0.,0.,0.,0.,1.},{0.,-27.6081,0.966339,-0.0561659,0.0561659,-0.0000322073},{0.,117.836,-32.5875,0.0792697,-0.0792697,0.00108612},{0.,-39.0589,35.4825,-0.0219192,0.0219192,-0.00118261}},{{0.},{0.},{0.},{17.9071},{-25.2731},{6.98836}},{},{}},{{\[Theta][t],0},{\[CurlyPhi][t],0},{\[Phi][t],0},Subscript[\[FormalX], 1],Subscript[\[FormalX], 2],Subscript[\[FormalX], 3]},{{u[t],0}},Automatic,t,SamplingPeriod->None,SystemsModelLabels->None]*)
