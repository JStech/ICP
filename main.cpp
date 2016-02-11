#include <iostream>
//#include <HAL/Camera/CameraDevice.h>
//#include <calibu/Calibu.h>

int main(int argc, char *argv[])
{
  // check arguments
  if (argc != 3) {
    std::cout << "Usage: " << argv[0] << " /path/to/cameras.xml 'scheme:///path/to/stereo/data'" << std::endl;
    return 1;
  }

  // read cameras.xml
  //std::shared_ptr<calibu::Rig<double>> rig =
    //calibu::ReadXmlRig(argv[1]);


  return 0;
}
