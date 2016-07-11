QT += widgets gui
QT *= opengl xml

QMAKE_CXXFLAGS  += -std=c++11

Thirdparty      = ../Thirdparty
UI              = ./UI
LSD             = ./LSDcore
PIL             = $${Thirdparty}/PIL/src


INCLUDEPATH += $${Thirdparty}
INCLUDEPATH += $${UI}
INCLUDEPATH += $${LSD}
INCLUDEPATH += $${Thirdparty}/eigen
INCLUDEPATH += /usr/include/suitesparse
INCLUDEPATH += $${PIL}

HEADERS += \
    UI/MainWidget.h \
    UI/PointCloudWidget.h \
    LSDcore/DataStructures/Frame.h \
    LSDcore/DataStructures/FrameMemory.h \
    LSDcore/DataStructures/FramePoseStruct.h \
    LSDcore/DepthEstimation/DepthMap.h \
    LSDcore/DepthEstimation/DepthMapPixelHypothesis.h \
    LSDcore/GlobalMapping/g2oTypeSim3Sophus.h \
    LSDcore/GlobalMapping/KeyFrameGraph.h \
    LSDcore/GlobalMapping/TrackableKeyFrameSearch.h \
    LSDcore/Tracking/LGSX.h \
    LSDcore/Tracking/Relocalizer.h \
    LSDcore/Tracking/SE3Tracker.h \
    LSDcore/Tracking/Sim3Tracker.h \
    LSDcore/Tracking/TrackingReference.h \
    LSDcore/util/EigenCoreInclude.h \
    LSDcore/util/globalFuncs.h \
    LSDcore/util/IndexThreadReduce.h \
    LSDcore/util/settings.h \
    LSDcore/util/SophusUtil.h \
    LSDcore/util/Undistorter.h \
    IOWrapper/Imgload.h \
    lsdSystem.h

SOURCES += \
    main.cpp \
    UI/MainWidget.cpp \
    UI/PointCloudWidget.cpp \
    LSDcore/DataStructures/Frame.cpp \
    LSDcore/DataStructures/FrameMemory.cpp \
    LSDcore/DataStructures/FramePoseStruct.cpp \
    LSDcore/DepthEstimation/DepthMap.cpp \
    LSDcore/DepthEstimation/DepthMapPixelHypothesis.cpp \
    LSDcore/GlobalMapping/g2oTypeSim3Sophus.cpp \
    LSDcore/GlobalMapping/KeyFrameGraph.cpp \
    LSDcore/GlobalMapping/TrackableKeyFrameSearch.cpp \
    LSDcore/Tracking/Relocalizer.cpp \
    LSDcore/Tracking/SE3Tracker.cpp \
    LSDcore/Tracking/Sim3Tracker.cpp \
    LSDcore/Tracking/TrackingReference.cpp \
    LSDcore/util/globalFuncs.cpp \
    LSDcore/util/settings.cpp \
    LSDcore/util/SophusUtil.cpp \
    LSDcore/util/Undistorter.cpp \
    IOWrapper/Imgload.cpp \
    lsdSystem.cpp

LIBQGLVIEWER = $${Thirdparty}/qglviewer/lib
LIBG2O       = $${Thirdparty}/g2o/lib
LIBBOOST     = $${Thirdparty}/boost

LIBS += -L$${LIBBOOST} -lboost_thread -lboost_system

LIBS += -L$${LIBQGLVIEWER}/ -lQGLViewer -lglut -lGLU

LIBS += -L$${LIBG2O} -lg2o_core -lg2o_cli -lg2o_csparse_extension -lcxsparse -lg2o_types_sba \
                     -lg2o_parser -lg2o_solver_eigen -lg2o_stuff -lg2o_types_data

CONFIG += link_pkgconfig
PKGCONFIG += opencv

