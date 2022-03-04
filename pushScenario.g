Include: '../botop/rai-robotModels/scenarios/pandasTable-calibrated.g'

stick (l_gripper){
    Q:<d(23 1 0 0) t(0 0 -.141)>
    shape:ssBox, size:[.015 .015 .5 .005] }

        stickTip (stick) { Q:[0 0 -.245], shape:sphere, size:[.008], color[1 1 .6 .3] }

puck (table){ joint:rigid Q:[.0 .4 .08]
            shape:ssCylinder size:[.06 .06 .005] color:[1 1 .6] }

target (table){ Q:[-.6 .3 .08]
            shape:ssCylinder size:[.06 .06 .005] color:[.6 1 .6] }

marc_red(optitrack_base){ shape:marker, size:[.1] }
marc_green(optitrack_base){ shape:marker, size:[.1] }
obst(optitrack_base){ shape:capsule, size:[.1, .08], color:[.9] }
HandStick(optitrack_base){ shape:marker, color:[1, 1, .5] size:[.02] }
