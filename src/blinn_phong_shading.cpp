#include "blinn_phong_shading.h"
// Hint:
#include "first_hit.h"
#include <iostream>

Eigen::Vector3d blinn_phong_shading(
  const Ray & ray,
  const int & hit_id, 
  const double & t,
  const Eigen::Vector3d & n,
  const std::vector< std::shared_ptr<Object> > & objects,
  const std::vector<std::shared_ptr<Light> > & lights)
{

  Eigen::Vector3d accumulated_light(0,0,0);
  double ambient_intensity = 0.1;
  double epsilon = 1e-6;

  auto object_we_are_computing_light_for = objects[hit_id];
  auto objects_material = object_we_are_computing_light_for->material;

  Eigen::Vector3d diffuse_coef = objects_material->kd;
  Eigen::Vector3d specular_coef = objects_material->ks;
  Eigen::Vector3d ambient_coef = objects_material->ka;
  double phong_exponent = objects_material->phong_exponent;

  Eigen::Vector3d position_we_are_computing_light_at = ray.origin + t * ray.direction;

  for (auto &light : lights){
    Eigen::Vector3d direction_to_light_source;
    double parametric_distance_to_light_source;

    light->direction(position_we_are_computing_light_at, direction_to_light_source, parametric_distance_to_light_source);

    Ray ray_from_hit_to_light_source;
    ray_from_hit_to_light_source.origin = position_we_are_computing_light_at; 
    ray_from_hit_to_light_source.direction = direction_to_light_source;
     
    int throwaway_id; double parametric_distance_from_hit_to_object_towards_light; Eigen::Vector3d throwaway_n;

    bool ray_from_hit_to_light_source_hits_object = first_hit(ray_from_hit_to_light_source, epsilon, objects, throwaway_id, parametric_distance_from_hit_to_object_towards_light, throwaway_n);
    bool clear_path_from_hit_to_light_source = not ray_from_hit_to_light_source_hits_object or parametric_distance_from_hit_to_object_towards_light > parametric_distance_to_light_source;

    if (clear_path_from_hit_to_light_source){

      Eigen::Vector3d reversed_view_ray = -ray.direction.normalized();
      direction_to_light_source = direction_to_light_source.normalized();
      Eigen::Vector3d halfway = (reversed_view_ray + direction_to_light_source).normalized();

      accumulated_light += diffuse_coef.cwiseProduct(light->I) * fmax(0, n.dot(direction_to_light_source));
      accumulated_light += specular_coef.cwiseProduct(light->I) * std::pow(fmax(0, n.dot(halfway)), phong_exponent);
    }
  }

  accumulated_light += ambient_coef * ambient_intensity;

  return accumulated_light;

}
