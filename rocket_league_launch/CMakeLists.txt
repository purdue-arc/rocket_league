cmake_minimum_required(VERSION 3.0.2)
project(rocket_league_launch)

# find catkin so we can use our macros
find_package(catkin REQUIRED)

# generate variables for installation
catkin_package()

#############
## Install ##
#############

# Mark launch files for installation
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)