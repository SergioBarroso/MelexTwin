#!/bin/bash

sleep 2      

TERMINAL_ID_0=$(qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.terminalIdsForSessionId 0)
qdbus org.kde.yakuake /yakuake/tabs setTabTitle 0 "Coppelia"

qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.splitTerminalLeftRight "$TERMINAL_ID_0"

SESSION_ID_1=$(qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession)
TERMINAL_ID_1=$(qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.terminalIdsForSessionId 1)
qdbus org.kde.yakuake /yakuake/tabs setTabTitle 1 "DSR"

qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.splitTerminalLeftRight "$TERMINAL_ID_1"

SESSION_ID_2=$(qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession)
TERMINAL_ID_2=$(qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.terminalIdsForSessionId 2)
qdbus org.kde.yakuake /yakuake/tabs setTabTitle 2 "Componentes"
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.splitTerminalLeftRight "$TERMINAL_ID_2"
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.splitTerminalLeftRight "$TERMINAL_ID_2"
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.splitTerminalTopBottom "$TERMINAL_ID_2"
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.splitTerminalTopBottom "$TERMINAL_ID_2"

SESSION_ID_3=$(qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession)
TERMINAL_ID_3=$(qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.terminalIdsForSessionId 3)
qdbus org.kde.yakuake /yakuake/tabs setTabTitle 3 "Joystick"
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.splitTerminalLeftRight "$TERMINAL_ID_3"


#Coppelia
qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 0 "killall -9 python3"
#qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 0 "rcnode &"
qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 0 "cd ~/robocomp/components/melexcar/CONTROL/Sim_Car/components/pioneer_pyrep"
qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 0 "./run.sh"

qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 1 "cd ~/software/CoppeliaSim_Edu_V4_2_0_Ubuntu20_04"

#DSR
qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 2 "cd ~/robocomp/components/melexcar/CONTROL/Sim_Car/agents/idserver"
qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 2 "bin/idserver etc/config_pioneer"

qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 3 "cd ~/robocomp/components/melexcar/CONTROL/Sim_Car/agents/pioneer_dsr"
qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 3 "bin/pioneer_dsr etc/config_coppelia"

#Componentes
qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 4 "cd ~/robocomp/components/dsr-graph/components/path_planner_astar"
qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 4 "bin/path_planner_astar etc/config_exterior-robolab"

qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 5 "cd ~/robocomp/components/melexcar/CONTROL/Sim_Car/agents/mission_controller_melex"
qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 5 "bin/mission_controller_melex etc/config"

qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 6 "cd ~/robocomp/components/melexcar/CONTROL/Sim_Car/agents/path_follower_melex"
qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 6 "bin/path_follower etc/config_coppelia_pioneer "

qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 7 #"cd ~/robocomp/components/dsr-graph/components/elastic_band"

qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 8

#Joystick
qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 9 "cd robocomp/components/robocomp-robolab/components/hardware/external_control/joystickpublish"
#qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal 9 "bin/JoystickPublish etc/config_pioneer_coppelia"






