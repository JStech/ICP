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

class PointSkyReader {
    struct archive *a;
    struct archive_entry *e;
    const char *filename;
    int n;

  public:
    PointSkyReader(const char fn[]): a(NULL), e(NULL), filename(fn), n(0) {

      // open file
      a = archive_read_new();
      archive_read_support_filter_all(a);
      archive_read_support_format_all(a);
      archive_read_support_compression_all(a);
      int r = archive_read_open_filename(a, filename, 10240);
      if (r != ARCHIVE_OK) {
        std::cerr << "Can't open file " << filename << std::endl;
        exit(1);
      }

    }

    // read clouds
    pcl::PointCloud<pcl::PointXYZ>::Ptr ReadNext() {
      char tmpfilename[L_tmpnam];

      if (archive_read_next_header(a, &e) != ARCHIVE_OK) {
        return NULL;
      }

      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud
        (new pcl::PointCloud<pcl::PointXYZ>);
      if (NULL == tmpnam(tmpfilename)) {
        exit(1);
      }
      int tmpfd = open(tmpfilename, O_RDONLY);
      if (archive_read_data_into_fd(a, tmpfd) != ARCHIVE_OK) {
        std::cerr << "Error unpacking cloud." << std::endl;
        close(tmpfd);
        std::remove(tmpfilename);
        exit(1);
      }
      close(tmpfd);
      if (pcl::io::loadPCDFile<pcl::PointXYZ>(tmpfilename, *cloud) == -1) {
        std::cerr << "Error reading cloud." << std::endl;
        std::remove(tmpfilename);
        exit(1);
      }
      std::remove(tmpfilename);
      return cloud;
    }
};

class PointSkyWriter {
    struct archive *a;
    struct archive_entry *e;
    const char *filename;
    int n;
  public:
    // create a sky
    PointSkyWriter(const char fn[]): a(NULL), e(NULL), filename(fn), n(0) {}

    // add a cloud to the sky
    void WriteCloud(pcl::PointCloud<pcl::PointXYZ> cloud) {
      char buff[BUFF_SZ];
      char tmpfilename[L_tmpnam];

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
      snprintf(buff, BUFF_SZ, "sky/%d.pcd", n);
      n++;
      archive_entry_set_pathname(e, buff);
      struct stat st;
      stat(tmpfilename, &st);
      archive_entry_copy_stat(e, &st);
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

    // finish and close
    void Close() {
      archive_write_close(a);
      archive_write_free(a);
      archive_entry_free(e);
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

  Eigen::Vector2d pixel;
  Eigen::Vector3d point;

  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.width = w*h;
  cloud.height = 1;
  cloud.is_dense = false;
  cloud.points.resize(w*h);

  PointSkyWriter sky("point_sky.tar.gz");

  // main loop
  for (unsigned frame_number = 0; !pangolin::ShouldQuit(); frame_number++) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    if (camera.Capture(*imgs)) {
      const unsigned char* v_img = (*imgs)[0]->data();
      const unsigned short* d_img = (unsigned short*) (*imgs)[1]->data();

      video_view.SetImage(v_img, w, h);
      depth_view.SetImage(d_img, w, h, GL_LUMINANCE, GL_LUMINANCE, GL_UNSIGNED_SHORT);

      for (unsigned i=0; i<h*w; i++) {
        if (d_img[i] == 0) {
          cloud.points[i].x = cloud.points[i].y = cloud.points[i].z =
            std::numeric_limits<float>::quiet_NaN();
          continue;
        }
        // unproject depth
        pixel(0) = (float) (i/w);
        pixel(1) = (float) (i%w);
        float depth = d_img[i] / 10000.0;
        depth *= std::sqrt(focal_length*focal_length + (i/w - h/2)*(i/w - h/2)
            + (i%w - w/2)*(i%w - w/2));
        depth /= 10.;
        point = depth * rig->cameras_[1]->Unproject(pixel);

        // append to point cloud
        cloud.points[i].x = point(0);
        cloud.points[i].y = point(1);
        cloud.points[i].z = point(2);
      }
      sky.WriteCloud(cloud);
    }
    pangolin::FinishFrame();
  }
  sky.Close();

  return 0;
}
