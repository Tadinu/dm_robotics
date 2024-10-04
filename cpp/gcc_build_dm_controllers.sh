mkdir -p build_gcc
pushd build_gcc

/home/ducthan/CMAKE/3.30.3/bin/cmake .. -DCMAKE_INSTALL_PREFIX=../release -DCMAKE_BUILD_TYPE:STRING=Release

cmake --build . -j8
cmake --install .
popd

