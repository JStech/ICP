#include <HAL/Camera/CameraDevice.h>
#include <SceneGraph/SceneGraph.h>
#include <archive.h>
#include <archive_entry.h>
#include <calibu/Calibu.h>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <pangolin/pangolin.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#define BUFF_SZ 8192

template <typename T>
class PointSky {
    struct archive *a;
    struct archive_entry *e;
    const char *filename;
    int n;
  public:
    // create a sky
    PointSky(char *filename): filename(filename), a(NULL), e(NULL), n(0) {}

    // add a cloud to the sky
    void WriteCloud(typename pcl::PointCloud<T>::Ptr cloud) {
      char buff[BUFF_SZ]; char tmpfilename[L_tmpnam];

      // create new archive and entry if necessary
      if (a==NULL) {
        a = archive_write_new();
        archive_write_add_filter_gzip(a);
        archive_write_set_format_ustar(a);
        archive_write_open_filename(a, filename);
      }
      if (e==NULL) {
        e = archive_entry_new();
      }

      // write point cloud to temporary file
      if (NULL == tmpnam(tmpfilename)) {
        exit(1);
      }
      pcl::io::savePCDFileBinary(tmpfilename, cloud);

      // generate filename, file attributes for archive
      snprintf(buff, BUFF_SZ, "%d.pcd", n);
      n++;
      archive_entry_set_pathname(e, buff);
      struct stat st;
      stat(tmpfilename, &st);
      archive_entry_copy_stat(e, st);
      archive_entry_set_filetype(e, AE_IFREG);
      archive_entry_set_perm(e, 0644);
      archive_write_header(a, e);

      // read data and write to archive
      FILE* f = fopen(tmpfilename, "rb");
      int len = fread(buff, 1, BUFF_SZ, f);
      while (len > 0) {
        archive_write_data(a, buff, len);
        len = fread(buff, 1, BUFF_SZ, f);
      }
      fclose(f);
      remove(tmpfilename);

      archive_entry_clear(e);
      return;
    }
};

int main(int argc, char *argv[])
{
  // check arguments
  if (argc != 3) {
    std::cout << "Usage: " << argv[0] << " /path/to/cameras.xml 'scheme:///path/to/stereo/data'" << std::endl;
    return 1;
  }

  // read cameras.xml, get baseline
  std::shared_ptr<calibu::Rig<double>> rig =
    calibu::ReadXmlRig(argv[1]);
  float focal_length = rig->cameras_[1]->GetParams()[0];

  // open image data
  hal::Camera camera(argv[2]);
  if (camera.NumChannels() != 2) {
    std::cerr << "Expecting video channel and depth channel." << std::endl;
    return 1;
  }
  const unsigned w = camera.Width();
  const unsigned h = camera.Height();

  // start Pangolin, create displays
  pangolin::CreateWindowAndBind("Main", 2*w, h);
  pangolin::View& base_view = pangolin::DisplayBase();

  // video, depth views
  SceneGraph::ImageView video_view(true, false);
  SceneGraph::ImageView depth_view(true, false);

  video_view.SetBounds(0.0, 1.0, 0.0, 0.5);
  depth_view.SetBounds(0.0, 1.0, 0.5, 1.0);

  base_view.AddDisplay(video_view);
  base_view.AddDisplay(depth_view);

  // memory stuff
  std::shared_ptr<hal::ImageArray> imgs = hal::ImageArray::Create();

  FILE* pcf = fopen("point_cloud.bin", "wb");

  Eigen::Vector2d pixel;
  Eigen::Vector3d point;

  // main loop
  for (unsigned frame_number = 0; !pangolin::ShouldQuit(); frame_number++) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    if (camera.Capture(*imgs)) {
      const unsigned char* v_img = (*imgs)[0]->data();
      const unsigned short* d_img = (unsigned short*) (*imgs)[1]->data();

      video_view.SetImage(v_img, w, h);
      depth_view.SetImage(d_img, w, h, GL_LUMINANCE, GL_LUMINANCE, GL_UNSIGNED_SHORT);

      float p[3];
      for (unsigned i=0; i<h*w; i++) {
        if (d_img[i] == 0) continue;
        // unproject depth
        pixel(0) = (float) (i/w);
        pixel(1) = (float) (i%w);
        float depth = d_img[i] / 10000.0;
        depth *= std::sqrt(focal_length*focal_length + (i/w - h/2)*(i/w - h/2)
            + (i%w - w/2)*(i%w - w/2));
        depth /= 10.;
        point = depth * rig->cameras_[1]->Unproject(pixel);

        // append to point cloud
        // TODO: do this more efficiently
        p[0] = point(0);
        p[1] = point(1);
        p[2] = point(2);
        fwrite(p, sizeof(float), 3, pcf);
      }
      p[0] = p[1] = p[2] = 0;
      fwrite(p, sizeof(float), 3, pcf);
    }
    pangolin::FinishFrame();
  }
  fclose(pcf);

  return 0;
}
