# Filters CPP

This package provides C++ libraries for filters, including the Kalman filter.

## Build
```
cd build
cmake -S .. -B . -DCMAKE_BUILD_TYPE=Release
sudo cmake --build . --target install
```

## Linking this project to another library

### Kalman Filter
```
find_library(kalman_filter_library NAMES kalman_filter_library PATHS /usr/local/lib)
find_path(kalman_filter_library_INCLUDE_DIRS NAMES kalman_filter/kalman_filter.hpp PATHS usr/local/include)

link_directories(${kalman_filter_library})
include_directories(${kalman_filter_library_INCLUDE_DIRS})

add_executable(executable_name source_files)

target_link_libraries(executable_name ${kalman_filter_library})

set_target_properties(executable_name PROPERTIES
    INSTALL_RPATH_USE_LINK_PATH TRUE
    INSTALL_RPATH "${kalman_filter_library}")
```

### Moving average filter
```
find_path(moving_average_filter_library_INCLUDE_DIRS NAMES moving_average_filter/moving_average_filter.hpp PATHS /usr/local/include)

include_directories(${moving_average_filter_library_INCLUDE_DIRS})
```

### Low-pass filter
```
find_path(low_pass_filter_library_INCLUDE_DIRS NAMES low_pass_filter/low_pass_filter.hpp PATHS /usr/local/include)

include_directories(${low_pass_filter_library_INCLUDE_DIRS})
```
