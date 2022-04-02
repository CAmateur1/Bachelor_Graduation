QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++11

# The following define makes your compiler emit warnings if you use
# any Qt feature that has been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    main.cpp \
    mainwindow.cpp \
    pointcloudmethod.cpp \
    prothread.cpp

HEADERS += \
    mainwindow.h \
    pointcloudmethod.h \
    prothread.h

FORMS += \
    mainwindow.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target


INCLUDEPATH += F:\PCL1.9.1\install\PCL1.9.1\include\pcl-1.9

INCLUDEPATH += F:\PCL1.9.1\install\PCL1.9.1\include\pcl-1.9\pcl

INCLUDEPATH += F:\PCL1.9.1\install\PCL1.9.1\3rdParty\Boost\include\boost-1_68

INCLUDEPATH += F:\PCL1.9.1\install\PCL1.9.1\3rdParty\Eigen\eigen3

INCLUDEPATH += F:\PCL1.9.1\install\PCL1.9.1\3rdParty\FLANN\include

INCLUDEPATH += F:\PCL1.9.1\install\PCL1.9.1\3rdParty\OpenNI2\Include

INCLUDEPATH += F:\PCL1.9.1\install\PCL1.9.1\3rdParty\Qhull\include

INCLUDEPATH += F:\PCL1.9.1\install\PCL1.9.1\3rdParty\VTK\include\vtk-8.1

CONFIG(debug)
{

LIBS += -LF:\PCL1.9.1\install\PCL1.9.1\lib\
        -lpcl_common_debug\
        -lpcl_kdtree_debug\
        -lpcl_octree_debug\
        -lpcl_search_debug\
        -lpcl_sample_consensus_debug\
        -lpcl_filters_debug\
        -lpcl_io_debug\
        -lpcl_io_ply_debug\
        -lpcl_features_debug\
        -lpcl_ml_debug\
        -lpcl_segmentation_debug\
        -lpcl_visualization_debug\
        -lpcl_surface_debug\
        -lpcl_registration_debug\
        -lpcl_keypoints_debug\
        -lpcl_tracking_debug\
        -lpcl_recognition_debug\
        -lpcl_stereo_debug\
        -lpcl_outofcore_debug\
        -lpcl_people_debug

LIBS += -LF:\PCL1.9.1\install\PCL1.9.1\3rdParty\Boost\lib\
        -llibboost_atomic-vc141-mt-gd-x64-1_68\
        -llibboost_bzip2-vc141-mt-gd-x64-1_68\
        -llibboost_chrono-vc141-mt-gd-x64-1_68\
        -llibboost_container-vc141-mt-gd-x64-1_68\
        -llibboost_context-vc141-mt-gd-x64-1_68\
        -llibboost_contract-vc141-mt-gd-x64-1_68\
        -llibboost_coroutine-vc141-mt-gd-x64-1_68\
        -llibboost_date_time-vc141-mt-gd-x64-1_68\
        -llibboost_exception-vc141-mt-gd-x64-1_68\
        -llibboost_fiber-vc141-mt-gd-x64-1_68\
        -llibboost_filesystem-vc141-mt-gd-x64-1_68\
        -llibboost_graph-vc141-mt-gd-x64-1_68\
        -llibboost_graph_parallel-vc141-mt-gd-x64-1_68\
        -llibboost_iostreams-vc141-mt-gd-x64-1_68\
        -llibboost_locale-vc141-mt-gd-x64-1_68\
        -llibboost_log-vc141-mt-gd-x64-1_68\
        -llibboost_log_setup-vc141-mt-gd-x64-1_68\
        -llibboost_math_c99-vc141-mt-gd-x64-1_68\
        -llibboost_math_c99f-vc141-mt-gd-x64-1_68\
        -llibboost_math_c99l-vc141-mt-gd-x64-1_68\
        -llibboost_math_tr1-vc141-mt-gd-x64-1_68\
        -llibboost_math_tr1f-vc141-mt-gd-x64-1_68\
        -llibboost_math_tr1l-vc141-mt-gd-x64-1_68\
        -llibboost_mpi-vc141-mt-gd-x64-1_68\
        -llibboost_numpy27-vc141-mt-gd-x64-1_68\
        -llibboost_numpy37-vc141-mt-gd-x64-1_68\
        -llibboost_prg_exec_monitor-vc141-mt-gd-x64-1_68\
        -llibboost_program_options-vc141-mt-gd-x64-1_68\
        -llibboost_python27-vc141-mt-gd-x64-1_68\
        -llibboost_python37-vc141-mt-gd-x64-1_68\
        -llibboost_random-vc141-mt-gd-x64-1_68\
        -llibboost_regex-vc141-mt-gd-x64-1_68\
        -llibboost_serialization-vc141-mt-gd-x64-1_68\
        -llibboost_signals-vc141-mt-gd-x64-1_68\
        -llibboost_stacktrace_noop-vc141-mt-gd-x64-1_68\
        -llibboost_stacktrace_windbg-vc141-mt-gd-x64-1_68\
        -llibboost_stacktrace_windbg_cached-vc141-mt-gd-x64-1_68\
        -llibboost_system-vc141-mt-gd-x64-1_68\
        -llibboost_test_exec_monitor-vc141-mt-gd-x64-1_68\
        -llibboost_thread-vc141-mt-gd-x64-1_68\
        -llibboost_timer-vc141-mt-gd-x64-1_68\
        -llibboost_type_erasure-vc141-mt-gd-x64-1_68\
        -llibboost_unit_test_framework-vc141-mt-gd-x64-1_68\
        -llibboost_wave-vc141-mt-gd-x64-1_68\
        -llibboost_wserialization-vc141-mt-gd-x64-1_68\
        -llibboost_zlib-vc141-mt-gd-x64-1_68

LIBS += -LF:\PCL1.9.1\install\PCL1.9.1\3rdParty\FLANN\lib\
        -lflann-gd\
        -lflann_cpp-gd\
        -lflann_cpp_s-gd\
        -lflann_s-gd

LIBS += -LF:\PCL1.9.1\install\PCL1.9.1\3rdParty\OpenNI2\Lib\
        -lOpenNI2

LIBS += -LF:\PCL1.9.1\install\PCL1.9.1\3rdParty\Qhull\lib\
        -lqhull_d\
        -lqhull_p_d\
        -lqhull_r_d\
        -lqhullcpp_d\
        -lqhullstatic_d\
        -lqhullstatic_r_d

LIBS += -LF:\PCL1.9.1\install\PCL1.9.1\3rdParty\VTK\lib\
        -lvtksys-8.1-gd\
        -lvtkCommonCore-8.1-gd\
        -lvtkCommonMath-8.1-gd\
        -lvtkCommonMisc-8.1-gd\
        -lvtkCommonSystem-8.1-gd\
        -lvtkCommonTransforms-8.1-gd\
        -lvtkCommonDataModel-8.1-gd\
        -lvtkCommonColor-8.1-gd\
        -lvtkCommonExecutionModel-8.1-gd\
        -lvtkCommonComputationalGeometry-8.1-gd\
        -lvtkFiltersCore-8.1-gd\
        -lvtkFiltersGeneral-8.1-gd\
        -lvtkImagingCore-8.1-gd\
        -lvtkImagingFourier-8.1-gd\
        -lvtkalglib-8.1-gd\
        -lvtkFiltersStatistics-8.1-gd\
        -lvtkFiltersExtraction-8.1-gd\
        -lvtkInfovisCore-8.1-gd\
        -lvtkFiltersGeometry-8.1-gd\
        -lvtkFiltersSources-8.1-gd\
        -lvtkRenderingCore-8.1-gd\
        -lvtkzlib-8.1-gd\
        -lvtkfreetype-8.1-gd\
        -lvtkRenderingFreeType-8.1-gd\
        -lvtkRenderingContext2D-8.1-gd\
        -lvtkChartsCore-8.1-gd\
        -lvtklz4-8.1-gd\
        -lvtkIOCore-8.1-gd\
        -lvtkIOLegacy-8.1-gd\
        -lvtkexpat-8.1-gd\
        -lvtkIOXMLParser-8.1-gd\
        -lvtkIOXML-8.1-gd\
        -lvtklibxml2-8.1-gd\
        -lvtkIOInfovis-8.1-gd\
        -lvtkglew-8.1-gd\
        -lvtkRenderingOpenGL2-8.1-gd\
        -lvtkRenderingContextOpenGL2-8.1-gd\
        -lvtkDICOMParser-8.1-gd\
        -lvtkmetaio-8.1-gd\
        -lvtkjpeg-8.1-gd\
        -lvtkpng-8.1-gd\
        -lvtktiff-8.1-gd\
        -lvtkIOImage-8.1-gd\
        -lvtkTestingRendering-8.1-gd\
        -lvtkImagingSources-8.1-gd\
        -lvtkFiltersHybrid-8.1-gd\
        -lvtkFiltersModeling-8.1-gd\
        -lvtkImagingColor-8.1-gd\
        -lvtkImagingGeneral-8.1-gd\
        -lvtkImagingHybrid-8.1-gd\
        -lvtkInteractionStyle-8.1-gd\
        -lvtkRenderingAnnotation-8.1-gd\
        -lvtkRenderingVolume-8.1-gd\
        -lvtkInteractionWidgets-8.1-gd\
        -lvtkViewsCore-8.1-gd\
        -lvtkViewsContext2D-8.1-gd\
        -lvtkFiltersGeneric-8.1-gd\
        -lvtkIOGeometry-8.1-gd\
        -lvtkTestingGenericBridge-8.1-gd\
        -lvtkDomainsChemistry-8.1-gd\
        -lvtkDomainsChemistryOpenGL2-8.1-gd\
        -lvtkParallelCore-8.1-gd\
        -lvtkFiltersAMR-8.1-gd\
        -lvtkhdf5-8.1-gd\
        -lvtkhdf5_hl-8.1-gd\
        -lvtkIOAMR-8.1-gd\
        -lvtkNetCDF-8.1-gd\
        -lvtkexoIIc-8.1-gd\
        -lvtkIOExodus-8.1-gd\
        -lvtkImagingMath-8.1-gd\
        -lvtkRenderingVolumeOpenGL2-8.1-gd\
        -lvtkFiltersFlowPaths-8.1-gd\
        -lvtkFiltersImaging-8.1-gd\
        -lvtkRenderingLabel-8.1-gd\
        -lvtkFiltersHyperTree-8.1-gd\
        -lvtkImagingStencil-8.1-gd\
        -lvtkFiltersParallel-8.1-gd\
        -lvtkFiltersParallelImaging-8.1-gd\
        -lvtkFiltersPoints-8.1-gd\
        -lvtkFiltersProgrammable-8.1-gd\
        -lvtkFiltersSMP-8.1-gd\
        -lvtkFiltersSelection-8.1-gd\
        -lvtkverdict-8.1-gd\
        -lvtkFiltersVerdict-8.1-gd\
        -lvtknetcdfcpp-8.1-gd\
        -lvtkIONetCDF-8.1-gd\
        -lvtkjsoncpp-8.1-gd\
        -lvtkIOParallel-8.1-gd\
        -lvtkFiltersTexture-8.1-gd\
        -lvtkFiltersTopology-8.1-gd\
        -lvtkGUISupportMFC-8.1-gd\
        -lvtkGUISupportQt-8.1-gd\
        -lvtkGUISupportQtOpenGL-8.1-gd\
        -lvtksqlite-8.1-gd\
        -lvtkIOSQL-8.1-gd\
        -lvtkGUISupportQtSQL-8.1-gd\
        -lvtkInfovisLayout-8.1-gd\
        -lvtkproj4-8.1-gd\
        -lvtkGeovisCore-8.1-gd\
        -lvtkIOEnSight-8.1-gd\
        -lvtkgl2ps-8.1-gd\
        -lvtkRenderingGL2PSOpenGL2-8.1-gd\
        -lvtklibharu-8.1-gd\
        -lvtkIOExport-8.1-gd\
        -lvtkIOExportOpenGL2-8.1-gd\
        -lvtkInteractionImage-8.1-gd\
        -lvtkIOImport-8.1-gd\
        -lvtkIOLSDyna-8.1-gd\
        -lvtkIOMINC-8.1-gd\
        -lvtkoggtheora-8.1-gd\
        -lvtkIOMovie-8.1-gd\
        -lvtkIOPLY-8.1-gd\
        -lvtkIOParallelXML-8.1-gd\
        -lvtkTestingIOSQL-8.1-gd\
        -lvtkIOTecplotTable-8.1-gd\
        -lvtkIOVideo-8.1-gd\
        -lvtkImagingStatistics-8.1-gd\
        -lvtkRenderingImage-8.1-gd\
        -lvtkImagingMorphological-8.1-gd\
        -lvtkRenderingLOD-8.1-gd\
        -lvtkRenderingQt-8.1-gd\
        -lvtkViewsInfovis-8.1-gd\
        -lvtkViewsQt-8.1-gd\
        -lvtkRenderingOpenGL-8.1-gd\
        -lvtkRenderingContextOpenGL-8.1-gd\
        -lvtkRenderingGL2PS-8.1-gd\
        -lvtkIOExportOpenGL-8.1-gd\
        -lvtkRenderingLIC-8.1-gd\
        -lvtkRenderingVolumeOpenGL-8.1-gd
}
