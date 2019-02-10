#!/usr/bin/env python

import os
import ycm_core

flags = [
'-Wall',
'-Wextra',
'-Werror',
'-fexceptions',
'-DNDEBUG',
'-std=c++11',
'-x',
'c++',
'-isystem',
'/usr/include',
'-isystem',
'/usr/local/include',
'-isystem',
'/opt/ros/' + os.getenv('ROS_DISTRO') + '/include',
'-isystem',
'/home/mukhe027/catkin_ws/devel/include',
'-isystem',
'/home/mukhe027/catkin_ws/src/CMakeLists.txt/include',
'-isystem',
'/home/mukhe027/catkin_ws/src/Onboard-SDK-ROS/include',
'-isystem',
'/home/mukhe027/catkin_ws/src/OpenTLD/include',
'-isystem',
'/home/mukhe027/catkin_ws/src/aprilslam/include',
'-isystem',
'/home/mukhe027/catkin_ws/src/apriltags_ros/include',
'-isystem',
'/home/mukhe027/catkin_ws/src/catkin_simple/include',
'-isystem',
'/home/mukhe027/catkin_ws/src/gazebo_ros_pkgs/include',
'-isystem',
'/home/mukhe027/catkin_ws/src/glog_catkin/include',
'-isystem',
'/home/mukhe027/catkin_ws/src/hku_m100_gazebo/include',
'-isystem',
'/home/mukhe027/catkin_ws/src/image_pipeline/include',
'-isystem',
'/home/mukhe027/catkin_ws/src/mav_comm/include',
'-isystem',
'/home/mukhe027/catkin_ws/src/mavros/include',
'-isystem',
'/home/mukhe027/catkin_ws/src/opencv_contrib/include',
'-isystem',
'/home/mukhe027/catkin_ws/src/rotors_simulator/include',
'-isystem',
'/home/mukhe027/catkin_ws/src/rviz/include'
]

compilation_database_folder = ''

if os.path.exists( compilation_database_folder ):
  database = ycm_core.CompilationDatabase( compilation_database_folder )
else:
  database = None

SOURCE_EXTENSIONS = [ '.cpp', '.cxx', '.cc', '.c' ]

def DirectoryOfThisScript():
  return os.path.dirname( os.path.abspath( __file__ ) )


def MakeRelativePathsInFlagsAbsolute( flags, working_directory ):
  if not working_directory:
    return list( flags )
  new_flags = []
  make_next_absolute = False
  path_flags = [ '-isystem', '-I', '-iquote', '--sysroot=' ]
  for flag in flags:
    new_flag = flag

    if make_next_absolute:
      make_next_absolute = False
      if not flag.startswith( '/' ):
        new_flag = os.path.join( working_directory, flag )

    for path_flag in path_flags:
      if flag == path_flag:
        make_next_absolute = True
        break

      if flag.startswith( path_flag ):
        path = flag[ len( path_flag ): ]
        new_flag = path_flag + os.path.join( working_directory, path )
        break

    if new_flag:
      new_flags.append( new_flag )
  return new_flags


def IsHeaderFile( filename ):
  extension = os.path.splitext( filename )[ 1 ]
  return extension in [ '.h', '.hxx', '.hpp', '.hh' ]


def GetCompilationInfoForFile( filename ):
  if IsHeaderFile( filename ):
    basename = os.path.splitext( filename )[ 0 ]
    for extension in SOURCE_EXTENSIONS:
      replacement_file = basename + extension
      if os.path.exists( replacement_file ):
        compilation_info = database.GetCompilationInfoForFile(
          replacement_file )
        if compilation_info.compiler_flags_:
          return compilation_info
    return None
  return database.GetCompilationInfoForFile( filename )


def FlagsForFile( filename, **kwargs ):
  if database:
    compilation_info = GetCompilationInfoForFile( filename )
    if not compilation_info:
      return None

    final_flags = MakeRelativePathsInFlagsAbsolute(
      compilation_info.compiler_flags_,
      compilation_info.compiler_working_dir_ )
  else:
    relative_to = DirectoryOfThisScript()
    final_flags = MakeRelativePathsInFlagsAbsolute( flags, relative_to )

  return {
    'flags': final_flags,
    'do_cache': True
  }
