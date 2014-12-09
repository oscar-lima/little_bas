cmake_minimum_required(VERSION 2.8)
project( face_recognition )
find_package( OpenCV REQUIRED )

add_executable(face_recognition 
	src/facerec_eigenfaces.cpp 
)

include_directories(
	${OpenCV_INCLUDE_DIRS}
	)

target_link_libraries(face_recognition ${OpenCV_LIBS})
