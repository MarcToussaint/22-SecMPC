Include: '../botop/rai-robotModels/scenarios/pandasTable-calibrated.g'

box (table){ joint: rigid, shape: ssBox, size: [.06,.08,.07,.01], Q:[-.0,-.0,.095] }

target (table) {joint: rigid, shape: ssBox, size: [.06,.06,.06,.01], Q:[-.4,.2,.1] }

marc_red(optitrack_base){ shape:marker, size:[.1] }
marc_green(optitrack_base){ shape:marker, size:[.1] }
obst_base(optitrack_base){}
obst(obst_base){ Q:[0 0 .25] shape:capsule, size:[.5, .02], color:[.9] }
HandStick(optitrack_base){ shape:marker, color:[1, 1, .5] size:[.02] }

camera_gl { shape:camera, X:<t(-.4 3 1.5) d(-90 1 0 0) d(180 0 0 1) d(-10 1 0 0)>,
height:600, width:600, focalLength:2. }
