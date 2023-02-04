#include <extremum_seeking/flux_lines.hpp>
#include <extremum_seeking/observer_based.hpp>
#include <extremum_seeking/extremum_seeking_bur.hpp>

int main(int argc, char **argv){
  ros::init(argc, argv, "extremum_seeking_node");
  ros::NodeHandle nh("~");

  //FluxLines fluxLinesMain(nh);
  //ObserverBased ObserverBasedMain(nh);
  ExtremumSeekingBUR ExtremumSeekingMain(nh);

  ros::spin();

  return 0;
}