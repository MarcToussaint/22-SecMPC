Include: '../botop/rai-robotModels/scenarios/pandasTable-calibrated.g'

stick (l_gripper){
    Q:<d(23 1 0 0) t(0 0 -.141)>
    shape:ssBox, size:[.015 .015 .5 .005] }

stickTip (stick) { Q:[-0.01 0.01 -.245], shape:sphere, size:[.008], color[1 1 .6 .3] }

puck (table){ joint:rigid Q:[.0 .4 .08]
            shape:ssCylinder size:[.06 .04 .005] color:[1 1 .6] }

target (table){ Q:[-.6 .3 .08]
            shape:ssCylinder size:[.06 .04 .005] color:[.6 1 .6] }


marc_red(optitrack_base){ shape:marker, size:[.1] }
marc_green(optitrack_base){ shape:marker, size:[.1] }
obst_base(optitrack_base){}
obst(obst_base){ Q:[0 0 .25] shape:capsule, size:[.5, .02], color:[.9] }
HandStick(optitrack_base){ shape:marker, color:[1, 1, .5] size:[.02] }
