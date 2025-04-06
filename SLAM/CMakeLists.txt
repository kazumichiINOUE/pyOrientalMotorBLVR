cmake_minimum_required(VERSION 4.0 FATAL_ERROR)
set (CMAKE_CXX_FLAGS "-O3 -std=c++20")
project(only_cpp)

# パッケージの探索
find_package (OpenCV REQUIRED)
find_package(OpenMP REQUIRED)
find_package(SDL2 REQUIRED)
find_package(Curses REQUIRED)

# インクルードファイルの場所を指定
include_directories(
  /usr/local/include
  /usr/include/eigen3
  /opt/homebrew/opt/lua@5.4/include/lua
  /usr/local/include/eigen3 	 	# for APPLE
  /opt/homebrew/include/eigen3 	# for M1 Mac
  /opt/homebrew/include         # for M1 Mac
  )

# リンクするライブラリの場所を指定
link_directories(
  /usr/local/lib
  /opt/homebrew/lib
  /opt/homebrew/opt/lua@5.4/lib
  )

# 実行ファイルの生成
add_executable (slam slam.cpp)
add_executable (slam_adjust slam_adjust.cpp)

target_link_libraries(slam ${OpenCV_LIBRARIES})
target_link_libraries(slam_adjust PUBLIC ${OpenCV_LIBRARIES} lua ${CURSES_LIBRARIES} SDL2::SDL2)

