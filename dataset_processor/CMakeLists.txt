cmake_minimum_required (VERSION 3.5.1)
project (data_processor)
set (CMAKE_CXX_STANDARD 14)

set(OpenCV_INCLUDE_DIRS /usr/local/include/opencv4/opencv /usr/local/include/opencv4)
set(OpenCV_LIBS -lopencv_superres -lopencv_videostab -lopencv_gapi -lopencv_stitching -lopencv_saliency -lopencv_optflow -lopencv_structured_light -lopencv_dpm -lopencv_stereo -lopencv_bioinspired -lopencv_ximgproc -lopencv_tracking -lopencv_hfs -lopencv_img_hash -lopencv_xphoto -lopencv_bgsegm -lopencv_video -lopencv_ccalib -lopencv_phase_unwrapping -lopencv_reg -lopencv_freetype -lopencv_cvv -lopencv_xobjdetect -lopencv_fuzzy -lopencv_face -lopencv_objdetect -lopencv_photo -lopencv_aruco -lopencv_dnn_objdetect -lopencv_xfeatures2d -lopencv_shape -lopencv_surface_matching -lopencv_rgbd -lopencv_calib3d -lopencv_viz -lopencv_datasets -lopencv_text -lopencv_dnn -lopencv_ml -lopencv_hdf -lopencv_line_descriptor -lopencv_features2d -lopencv_highgui -lopencv_videoio -lopencv_imgcodecs -lopencv_flann -lopencv_plot -lopencv_imgproc -lopencv_core)
set(OpenCV_LIBRARY_DIRS /usr/local/lib)

include_directories(include)
include_directories(/usr/include/eigen3)

include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(ALL_LIBS ${OpenCV_LIBS} ${PCL_LIBRARIES})

add_executable(calculate_rt src/calculate_rt.cpp)
add_executable(mouse_click src/mouse_click.cpp)

target_link_libraries(calculate_rt ${ALL_LIBS})
target_link_libraries(mouse_click ${ALL_LIBS})
