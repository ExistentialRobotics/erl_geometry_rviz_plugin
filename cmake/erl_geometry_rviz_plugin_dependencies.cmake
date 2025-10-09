if(ROS1_ACTIVATED)
    erl_config_qt5(Core Widgets)
    erl_config_ogre()
elseif(ROS2_ACTIVATED)
    erl_config_qt(Core Widgets) # Qt5 or Qt6
endif()
