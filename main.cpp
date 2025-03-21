#include "Object.h"
#include "Camera.h"
#include "Light.h"
#include "read_json.h"
#include "write_ppm.h"
#include "viewing_ray.h"
#include "raycolor.h"
#include <Eigen/Core>
#include <Eigen/src/Core/Matrix.h>
#include <vector>
#include <iostream>
#include <memory>
#include <limits>
#include <functional>


int main(int argc, char * argv[])
{
  Camera camera;
  std::vector< std::shared_ptr<Object> > objects;
  std::vector< std::shared_ptr<Light> > lights;
  // Read a camera and scene description from given .json file
  read_json(
    argc<=1?"../data/sphere-and-plane.json":argv[1],
    camera,
    objects,
    lights);

  int width =  1920;
  int height = 1080;
  std::vector<unsigned char> rgb_image(3*width*height);

    int c = 0;
    double max_c = height * width;

    // For each pixel (i,j)
    for (unsigned i = 0; i < height; ++i)
    {
        for (unsigned j = 0; j < width; ++j)
        {
            c += 1;

            // Set background color
            Eigen::Vector3d rgb(0, 0, 0);

            // Compute viewing ray
            Ray ray;
            viewing_ray(camera, i, j, width, height, ray);

            raycolor(ray, 1.0, objects, lights, 0, rgb);

            // Write double precision color into image
            auto clamp = [](double s)
            { return std::max(std::min(s, 1.0), 0.0); };
            rgb_image[0 + 3 * (j + width * i)] = 255.0 * clamp(rgb(0));
            rgb_image[1 + 3 * (j + width * i)] = 255.0 * clamp(rgb(1));
            rgb_image[2 + 3 * (j + width * i)] = 255.0 * clamp(rgb(2));

            printf("[%f / 100]\r", 100 * (c / max_c));
            fflush(stdout);
        }
    }

  write_ppm("rgb.ppm",rgb_image,width,height,3);
}
