cmake_minimum_required(VERSION 4.0 FATAL_ERROR)
set (CMAKE_CXX_FLAGS "-O3 -g3 -std=c++17")
project(only_cpp)

# パッケージの探索
find_package (OpenCV REQUIRED)

# インクルードファイルの場所を指定
include_directories(
  ${PROJECT_SOURCE_DIR}/include
  /usr/local/include
  /usr/include/eigen3
  /usr/local/include/eigen3 	 	# for APPLE
  /opt/homebrew/include/eigen3 	# for M1 Mac
  /opt/homebrew/include         # for M1 Mac
  )

# リンクするライブラリの場所を指定
link_directories(
  ${PROJECT_SOURCE_DIR}/lib
  /usr/local/lib
  /opt/homebrew/lib
  )

# 実行ファイルの生成
add_executable (slam slam.cpp)

target_link_libraries(slam ${OpenCV_LIBRARIES})
