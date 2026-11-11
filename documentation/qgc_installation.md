```
// Dependencies 
sudo apt-get update
sudo apt-get install -y fuse libfuse2 // Ubuntu 22.04

// AppImage installation
cd ~/Downloads
wget https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage

// Execute 
chmod +x QGroundControl.AppImage

// Start
./QGroundControl.AppImage

(optional) Test with USB cable on PI -> Flight controller
sudo usermod -a -G dialout $USER
sudo usermod -a -G plugdev $USER

```