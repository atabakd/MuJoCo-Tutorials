# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

#####set(BIN_NAME LQR-Acrobot)

# The following 5 lines come from drake-shambhala
option(RUN_X11_TESTS "Run tests that require X11" OFF)
set(drake_DIR "/opt/drake/lib/cmake/drake")
find_package(drake CONFIG REQUIRED)
find_package(PythonInterp 2.7 EXACT MODULE REQUIRED)
set(PYTHONPATH "${drake_PYTHON_DIRS}:$ENV{PYTHONPATH}")

list(APPEND CMAKE_MODULE_PATH "${MUJOCO_PY_MJPRO_PATH}/cmake/modules")

#####add_executable(${BIN_NAME} ${BIN_NAME}.cpp)
#file(GLOB APP_SOURCES *.cxx)
file(GLOB APP_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.cpp)
foreach(testsourcefile ${APP_SOURCES})

    string(REPLACE ".cpp" "" BIN_NAME ${testsourcefile})
    add_executable(${BIN_NAME} ${testsourcefile} LQR-Acrobot.h)
    target_link_libraries(${BIN_NAME} ${LIB_MUJOCO})

    # Standard libraries for GL
    target_link_libraries(${BIN_NAME} GL GLU glut )

    # Additional libraries from mujoco package
    target_link_libraries(${BIN_NAME} libglew.so libglfw.so libglewegl.so libglewosmesa.so)

    # Drake stuff
    target_link_libraries(${BIN_NAME} drake::drake)

endforeach( testsourcefile ${APP_SOURCES} )
