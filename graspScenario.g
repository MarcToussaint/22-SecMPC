Include: '../rai-robotModels/scenarios/pandasTable-calibrated.g'

box (table){ joint: rigid, shape: ssBox, size: [.06,.08,.07,.01], Q:[-.0,-.0,.095] }

target (table) {joint: rigid, shape: ssBox, size: [.06,.06,.06,.01], Q:[-.4,.2,.1] }

b1(optitrack_base){ shape:marker, size:[.1] }
b2(optitrack_base){ shape:marker, size:[.1] }
green3(optitrack_base){ shape:marker, size:[.1] }
HandStick(optitrack_base){ shape:marker, size:[.1] }
