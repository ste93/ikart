
yarp_prepare_plugin(iKartOdometry
        CATEGORY device
        TYPE IKartOdometry
        INCLUDE IKartOdometry.h
        DEPENDS "TARGET YARP::YARP_math"
        )

    yarp_add_plugin(yarp_iKartOdometry)

    target_sources(yarp_iKartOdometry
            PRIVATE
            IKartOdometry.h
            IKartOdometry.cpp
            )

    target_link_libraries(yarp_iKartOdometry
            PRIVATE
            YARP::YARP_os
            YARP::YARP_sig
            YARP::YARP_dev
            YARP::YARP_math
            )
    list(APPEND YARP_${YARP_PLUGIN_MASTER}_PRIVATE_DEPS
            YARP_os
            YARP_sig
            YARP_dev
            YARP_math
            )

    yarp_install(
            TARGETS yarp_iKartOdometry
            EXPORT YARP_${YARP_PLUGIN_MASTER}
            COMPONENT ${YARP_PLUGIN_MASTER}
            LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
            ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR}
            YARP_INI DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR}
    )

    set(YARP_${YARP_PLUGIN_MASTER}_PRIVATE_DEPS ${YARP_${YARP_PLUGIN_MASTER}_PRIVATE_DEPS} PARENT_SCOPE)

    set_property(TARGET yarp_iKartOdometry PROPERTY FOLDER "Plugins/Device/Fake")
