#include "raycolor.h"
#include "first_hit.h"
#include "blinn_phong_shading.h"
#include "reflect.h"

bool raycolor(
  const Ray & ray, 
  const double min_t,
  const std::vector< std::shared_ptr<Object> > & objects,
  const std::vector< std::shared_ptr<Light> > & lights,
  const int num_recursive_calls,
  Eigen::Vector3d & rgb)
{

  int recursive_cutoff = 11;
  
  if (num_recursive_calls == recursive_cutoff){
      return false;
  }

  int idx_of_hit_obj; 
  double parametric_distance_of_hit;
  Eigen::Vector3d normal_at_hit;

  bool ray_hit_something = first_hit(ray, min_t, objects, idx_of_hit_obj, parametric_distance_of_hit, normal_at_hit);

  if (not ray_hit_something) {
    return false;
  }

  rgb = blinn_phong_shading(ray, idx_of_hit_obj, parametric_distance_of_hit, normal_at_hit, objects, lights);

  Ray reflected_ray;
  Eigen::Vector3d recursive_reflected_rgb;

  reflected_ray.origin = ray.origin + parametric_distance_of_hit * ray.direction;;
  reflected_ray.direction = reflect(ray.direction.normalized(), normal_at_hit);

  bool rec_had_hit = raycolor(reflected_ray, 1.0e-6, objects, lights, num_recursive_calls+1, recursive_reflected_rgb);

  if (rec_had_hit) {
    auto hit_obj = objects[idx_of_hit_obj];
    rgb += hit_obj->material->km.cwiseProduct(recursive_reflected_rgb);
  }
  
  return true;
}
