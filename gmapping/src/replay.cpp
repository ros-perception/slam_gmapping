/* Author: Laurent George */

#include <ros/ros.h>

#include "slam_gmapping.h"
#include "boost/program_options.hpp"

int
main(int argc, char** argv)
{
    /** Define and parse the program options 
     */ 
    namespace po = boost::program_options; 
    po::options_description desc("Options"); 
    desc.add_options() 
    ("help", "Print help messages") 
    ("scan_topic",  po::value<std::string>()->default_value("/scan") ,"topic that contains the laserScan in the rosbag")
    ("bag_filename", po::value<std::string>()->required(), "ros bag filename") 
    ("seed", po::value<unsigned long int>()->default_value(0), "seed")
    ("max_duration_buffer", po::value<unsigned long int>()->default_value(99999), "max tf buffer duration") ;
    
    po::variables_map vm; 
    try 
    { 
        po::store(po::parse_command_line(argc, argv, desc),  
                  vm); // can throw 
        
        /** --help option 
         */ 
        if ( vm.count("help")  ) 
        { 
            std::cout << "Basic Command Line Parameter App" << std::endl 
            << desc << std::endl; 
            return 0; 
        } 
        
        po::notify(vm); // throws on error, so do after help in case 
        // there are any problems 
    } 
    catch(po::error& e) 
    { 
        std::cerr << "ERROR: " << e.what() << std::endl << std::endl; 
        std::cerr << desc << std::endl; 
        return -1; 
    } 
    
    std::string bag_fname = vm["bag_filename"].as<std::string>();
    std::string scan_topic = vm["scan_topic"].as<std::string>();
    unsigned long int seed = vm["seed"].as<unsigned long int>();
    unsigned long int max_duration_buffer = vm["max_duration_buffer"].as<unsigned long int>();
    
    ros::init(argc, argv, "slam_gmapping");
    SlamGMapping *  gn = new SlamGMapping(bag_fname, scan_topic, seed, max_duration_buffer) ;
    std::cout << "replay stop, press ctrl-c to quit"  << std::endl;
    ros::spin(); // wait so user can save the map
    return(0);
    
    
}


