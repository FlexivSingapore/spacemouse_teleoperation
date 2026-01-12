##################################################
# Assuming ~/spacemouse as base directory
##################################################

======================
Install spacenavd
======================
cd ~/spacemouse
git clone https://github.com/FreeSpacenav/spacenavd
cd spacenavd
git checkout v1.1
./configure
make -j 4
sudo make install

cd contrib/systemd
sudo cp spacenavd.service /etc/systemd/system

======================
Install libspnav
======================
cd ~/spacemouse
git clone https://github.com/FreeSpacenav/libspnav
cd libspnav
git checkout v1.1
./configure
make -j 4
sudo make install

======================
Activating space mouse
======================
sudo systemctl start spacenavd

======================
Testing space mouse
======================
cd ~/Desktop/spacemouse/libspnav/examples/fly
make -j 4
./fly
