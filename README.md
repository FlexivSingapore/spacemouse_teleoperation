# Teleoperation of Flexiv Robots using Spacemouse

## Build and install depenencies
1. Please install flexiv_rdk via https://github.com/flexivrobotics/flexiv_rdk. Follow the README.md in flexiv_rdk to install the flexiv rdk library. Please checkout version v1.8 of the flexiv_rdk.

2. Install relevant dependencies

        sudo apt-get install libusb-1.0-0-dev

2. In the current repository, create a build folder
   
        mkdir build && cd build

3. Configure cmake via

        cmake .. -DCMAKE_PREFIX_PATH=~/rdk_install

4. Build the application

        make -j 5

5. Run the application

        ./flexiv_teleoperation_app

## Build and install spacemouse drivers
Please follow the instructions in README_Spacemouse.txt to install the spacemouse drivers. 