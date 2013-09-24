Package to plan turning the wheel and do for drc_hubo with limited support for hubo_plus

    - Comps must be installed with OPENRAVE_PLUGINS added to you envirnoment http://sourceforge.net/projects/comps

To call the planner service :

rosservice call /hubo_planner/PlanningQuery '{valve_position: { position: { x: 0, y: 0 ,z: 1 }, orientation: {x: 0, y: 0, z: 0, w: 1 } } }'



