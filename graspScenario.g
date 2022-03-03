Include: '../rai-robotModels/scenarios/pandasTable-calibrated.g'

box (table){ joint: rigid, shape: ssBox, size: [.06,.08,.07,.01], Q:[-.0,-.0,.095] }

target (table) {joint: rigid, shape: ssBox, size: [.06,.06,.06,.01], Q:[-.4,.2,.1] }

marc_red(optitrack_base){ shape:marker, size:[.1] }
marc_green(optitrack_base){ shape:marker, size:[.1] }
obst(optitrack_base){ shape:marker, size:[.1] }
#HandStick(optitrack_base){ shape:marker, size:[.1] }
