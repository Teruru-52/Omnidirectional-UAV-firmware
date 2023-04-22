message("================== LinearProgrammin Test ==================")

# CMakeのバージョンを設定
cmake_minimum_required(VERSION 3.16.3)

# プロジェクト名と使用する言語を設定
project(main C)

# 実行ファイルを作成
add_executable(main main.c)

# cmake -S . -B build
# cmake --build build