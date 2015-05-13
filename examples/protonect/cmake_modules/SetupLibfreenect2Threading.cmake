INCLUDE(CheckCXXSourceCompiles)

SET (CMAKE_REQUIRED_FLAGS "-std=c++11")

CHECK_CXX_SOURCE_COMPILES("
  #include <thread>
  #include <mutex>
  #include <condition_variable>
  #include <chrono>

  int main(int argc, char** argv) {
    std::thread thread;
    std::mutex mutex;
    std::lock_guard<std::mutex> lock_guard(mutex);
    std::unique_lock<std::mutex> unique_lock(mutex);
    std::condition_variable condition_variable;
    thread_local int i;
    
    return 0;
  }
  " LIBFREENECT2_THREADING_STDLIB_CXX11)

IF(NOT LIBFREENECT2_THREADING_STDLIB_CXX11)

  SET (CMAKE_REQUIRED_FLAGS "-std=c++0x")

  CHECK_CXX_SOURCE_COMPILES("
    #include <thread>
    #include <mutex>
    #include <condition_variable>
    #include <chrono>

    int main(int argc, char** argv) {
      std::thread thread;
      std::mutex mutex;
      std::lock_guard<std::mutex> lock_guard(mutex);
      std::unique_lock<std::mutex> unique_lock(mutex);
      std::condition_variable condition_variable;
      thread_local int i;
      
      return 0;
    }
    " LIBFREENECT2_THREADING_STDLIB_CXX0X)
ENDIF()

IF(LIBFREENECT2_THREADING_STDLIB_CXX11 OR LIBFREENECT2_THREADING_STDLIB_CXX0X)
  SET(LIBFREENECT2_THREADING "stdlib")
  SET(LIBFREENECT2_THREADING_INCLUDE_DIR "")
  SET(LIBFREENECT2_THREADING_SOURCE "")
  SET(LIBFREENECT2_THREADING_LIBRARIES "")
  SET(LIBFREENECT2_THREADING_STDLIB 1)
ELSE()
  SET(LIBFREENECT2_THREADING "tinythread")
  SET(LIBFREENECT2_THREADING_INCLUDE_DIR "src/tinythread/")
  SET(LIBFREENECT2_THREADING_SOURCE "src/tinythread/tinythread.cpp")
  IF(NOT WIN32)
    SET(LIBFREENECT2_THREADING_LIBRARIES "pthread")
  ELSE(NOT WIN32)
    SET(LIBFREENECT2_THREADING_LIBRARIES "")
  ENDIF(NOT WIN32)
  SET(LIBFREENECT2_THREADING_TINYTHREAD 1)
ENDIF()

MESSAGE(STATUS "using ${LIBFREENECT2_THREADING} as threading library")
