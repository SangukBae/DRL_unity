# CMakeLists.txt
#
# [역할]
# ROS2 패키지 빌드 설정 파일.
# Python 스크립트들을 ROS2 환경에 설치하고,
# config/launch 파일들을 share 디렉토리에 설치한다.
#
# [구현 내용]
# - cmake_minimum_required, project(hunter_se_drl) 선언
# - ament_cmake, ament_cmake_python find_package
# - ament_python_install_package(scripts)로 scripts 패키지 등록
# - install(PROGRAMS ...) 으로 각 실행 스크립트 등록
# - install(DIRECTORY config launch ...) 으로 리소스 파일 share에 설치
# - ament_package() 마무리
