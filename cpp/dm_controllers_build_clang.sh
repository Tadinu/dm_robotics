#https://github.com/google-deepmind/mujoco_mpc/blob/main/.github/workflows/build.yml

#sudo apt update
#sudo apt install libc++-17-dev
#sudo apt install libc++abi-17-dev
mkdir -p build
pushd build

/home/ducthan/CMAKE/3.30.3/bin/cmake .. -G Ninja \
      -DCMAKE_C_COMPILER:STRING=clang-18 -DCMAKE_CXX_COMPILER:STRING=clang++-18 \
		  -DCMAKE_INSTALL_PREFIX=../release \
		  -DCMAKE_BUILD_TYPE:STRING=Release
      #-DCMAKE_CXX_FLAGS:STRING="-stdlib=libc++" \
      #-DCMAKE_EXE_LINKER_FLAGS:STRING="-Wl,--no-as-needed -stdlib=libc++" \

cmake --build . -j8
cmake --install .
popd

# Copy mjpc release (libs + headers) to UE's RRSimBase
#source ./copy_mjpcrelease_to_rrsim_base.sh

