#include <bitbots_ros_control/wolfgang_hardware_interface.h>

namespace bitbots_ros_control
{

  /**
   * This class provides a combination of multiple hardware interfaces to construct a complete Wolfgang robot.
   * It is similar to a CombinedRobotHw class as specified in ros_control, but it is changed a bit to make sharing of
   * a common bus driver over multiple hardware interfaces possible.
   */
  WolfgangHardwareInterface::WolfgangHardwareInterface(rclcpp::Node::SharedPtr nh) : servo_interface_(nh)
  {
    nh_ = nh;

    // get list of all bus devices
    rcl_interfaces::msg::ListParametersResult device_name_list = nh_->list_parameters({"device_info"}, 3);

    // Convert dxls to native type: a vector of tuples with name and id for sorting purposes
    std::vector<std::pair<std::string, int>> dxl_devices;
    for (const std::string &parameter_name : device_name_list.names)
    {
      // we get directly the parameters and not the groups. use id parameter to identify them
      if (parameter_name.find(".id") != std::string::npos)
      {
        int id = nh->get_parameter(parameter_name).as_int();
        // remove "device_info." and ".id"
        std::string device_name = parameter_name.substr(12, parameter_name.size() - 3 - 12);
        dxl_devices.emplace_back(device_name, id);
      }
    }

    // sort the devices by id. This way the devices will always be read and written in ID order later, making debug easier.
    std::sort(dxl_devices.begin(), dxl_devices.end(),
              [](std::pair<std::string, int> &a, std::pair<std::string, int> &b)
              { return a.second < b.second; });

    // create overall servo interface since we need a single interface for the controllers
    servo_interface_ = DynamixelServoHardwareInterface(nh);

    // try to ping all devices on the list, add them to the driver and create corresponding hardware interfaces
    // try until interruption to enable the user to turn on the power
    RCLCPP_ERROR_STREAM(nh_->get_logger(), "Found dx_devices count: " << std::to_string(dxl_devices.size()));
    RCLCPP_INFO_STREAM(nh_->get_logger(), "Found dx_devices count: " << std::to_string(rclcpp::ok()));

    while (rclcpp::ok())
    {
      if (create_interfaces(dxl_devices))
      {
        break;
      }
      nh_->get_clock()->sleep_for(rclcpp::Duration::from_seconds(3));
    }
  }

  bool WolfgangHardwareInterface::create_interfaces(std::vector<std::pair<std::string, int>> dxl_devices)
  {
    interfaces_ = std::vector<std::vector<bitbots_ros_control::HardwareInterface *>>();
    // init bus drivers
    std::vector<std::string> pinged;
    rcl_interfaces::msg::ListParametersResult port_list = nh_->list_parameters({"port_info"}, 3);
    RCLCPP_INFO_STREAM(nh_->get_logger(), "Hi") ;//<< std::to_string(parameter_name));

    for (const std::string &parameter_name : port_list.names)
    {
      // we get directly the parameters and not the groups. use id parameter to identify them
      //RCLCPP_INFO_STREAM(nh_->get_logger(), "Divice_file " << std::to_string(parameter_name.find(".device_file")
      //));

      if (parameter_name.find(".device_file") != std::string::npos)
      {
        std::string port_name = parameter_name.substr(10, parameter_name.size() - 11 - 11);
        // read bus driver specifications from config
        std::string device_file = nh_->get_parameter("port_info." + port_name + ".device_file").as_string();
        int baudrate = nh_->get_parameter("port_info." + port_name + ".baudrate").as_int();
        int protocol_version = nh_->get_parameter("port_info." + port_name + ".protocol_version").as_int();
        auto driver = std::make_shared<DynamixelDriver>();
        if (!driver->init(device_file.c_str(), uint32_t(baudrate)))
        {
          RCLCPP_ERROR(nh_->get_logger(), "Error opening serial port %s", device_file.c_str());
          sleep(1);
          exit(1);
        }
        // some interface seem to produce some gitter directly after connecting. wait or it will interfere with pings
        // uncomment the following line if you are using such an interface
        // sleep(1);
        driver->setPacketHandler(protocol_version);
        std::vector<bitbots_ros_control::HardwareInterface *> interfaces_on_port;
        // iterate over all devices and ping them to see what is connected to this bus
        std::vector<std::tuple<int, std::string, float, float>> servos_on_port;
        RCLCPP_ERROR_STREAM(nh_->get_logger(), "Found dx_devices count: " << std::to_string(dxl_devices.size()));
        for (std::pair<std::string, int> &device : dxl_devices)
        {
          // RCLCPP_INFO_STREAM(nh_->get_logger(), device.first);
          std::string name = device.first;
          int id = device.second;
          if (std::find(pinged.begin(), pinged.end(), device.first) != pinged.end())
          {
            // we already found this and don't have to search again
          }
          else
          {
            int model_number_specified;
            nh_->get_parameter("device_info." + name + ".model_number", model_number_specified);
            // some devices provide more than one type of interface, e.g. the IMU provides additionally buttons and LEDs
            std::string interface_type;
            nh_->get_parameter("device_info." + name + ".interface_type", interface_type);
            uint16_t model_number_specified_16 = uint16_t(model_number_specified);
            uint16_t *model_number_returned_16 = new uint16_t;
            //for (uint8_t i = 0; i < uint8_t(128); i++){
            //if (driver->ping(uint8_t(id), model_number_returned_16))
            //    RCLCPP_ERROR_STREAM(nh_->get_logger(), "Start servo init" << i );
            //}
            if (driver->ping(uint8_t(id), model_number_returned_16))
            {
              // check if the specified model number matches the actual model number of the device
              if (model_number_specified_16 != *model_number_returned_16)
              {
                RCLCPP_WARN(nh_->get_logger(), "Model number of id %d does not match", id);
              }
              // ping was successful, add device correspondingly
              // only add them if the mode is set correspondingly
              // TODO maybe move more of the parameter stuff in the init of the modules instead of doing everything here
              if ((model_number_specified == 311 || model_number_specified == 321 || model_number_specified == 320 ||
                   model_number_specified == 1100))
              {
                RCLCPP_ERROR_STREAM(nh_->get_logger(), "Start servo init" );

                // Servos
                // We need to add the tool to the driver for later reading and writing
                driver->setTools(model_number_specified_16, id);
                float mounting_offset;
                nh_->get_parameter_or("device_info." + name + ".mounting_offset", mounting_offset, 0.0f);
                float joint_offset;
                nh_->get_parameter_or("device_info." + name + ".joint_offset", joint_offset, 0.0f);
                servos_on_port.push_back(std::make_tuple(id, name, mounting_offset, joint_offset));
              
                RCLCPP_ERROR_STREAM(nh_->get_logger(), "End servo init" );

              }
              pinged.push_back(name);
            }
            else {
                RCLCPP_ERROR_STREAM(nh_->get_logger(), "Cand ping");

            }
            delete model_number_returned_16;
          }
        }
        // create a servo bus interface if there were servos found on this bus
        RCLCPP_INFO_STREAM(nh_->get_logger(), "Founded servos bus count" << std::to_string(servos_on_port.size())) ;//<< std::to_string(parameter_name));

        if (servos_on_port.size() > 0)
        {
          ServoBusInterface *interface = new ServoBusInterface(nh_, driver, servos_on_port);
          interfaces_on_port.push_back(interface);
          servo_interface_.addBusInterface(interface);
        }
        // add vector of interfaces on this port to overall collection of interfaces
        interfaces_.push_back(interfaces_on_port);
      }
    }

    // if (0 || pinged.size()!=dxl_devices.size()) {
    if (0)
    {

      // when we only have 1 or two devices it's only the core
      if (pinged.empty() || pinged.size() == 1 || pinged.size() == 2)
      {

        RCLCPP_ERROR_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 5000, "Could not start ros control. Power is off!");
      }
      else
      {
        if (first_ping_error_)
        {
          first_ping_error_ = false;
        }
        else
        {
          RCLCPP_ERROR(nh_->get_logger(), "Could not ping all devices!");
          // check which devices were not pinged successful
          for (std::pair<std::string, int> &device : dxl_devices)
          {
            if (std::find(pinged.begin(), pinged.end(), device.first) != pinged.end())
            {
            }
            else
            {
              RCLCPP_ERROR(nh_->get_logger(), "%s with id %d missing", device.first.c_str(), device.second);
            }
          }
        }
      }
      return false;
    }
    else
    {
      return true;
    }
  }

  void threaded_init(std::vector<HardwareInterface *> &port_interfaces, rclcpp::Node::SharedPtr &nh, int &success)
  {
    success = true;
    for (HardwareInterface *interface : port_interfaces)
    {
      success &= interface->init();
    }
  }

  bool WolfgangHardwareInterface::init()
  {
    // iterate through all ports
    std::vector<std::thread> threads;
    std::vector<int *> successes;
    int i = 0;
    for (std::vector<HardwareInterface *> &port_interfaces : interfaces_)
    {
      // iterate through all interfaces on this port
      // we use an int instead of bool, since std::ref can't handle bool
      int suc = 0;
      successes.push_back(&suc);
      threads.push_back(std::thread(threaded_init, std::ref(port_interfaces), std::ref(nh_), std::ref(suc)));
      i++;
    }
    // wait for all inits to finish
    for (std::thread &thread : threads)
    {
      thread.join();
    }
    // see if all inits were successful
    bool success = true;
    for (bool s : successes)
    {
      success &= s;
    }
    // init servo interface last after all servo busses are there
    success &= servo_interface_.init();
    return success;
  }

  void threaded_read(std::vector<HardwareInterface *> &port_interfaces,
                     const rclcpp::Time &t,
                     const rclcpp::Duration &dt)
  {
    for (HardwareInterface *interface : port_interfaces)
    {
      interface->read(t, dt);
    }
  }

  void WolfgangHardwareInterface::read(const rclcpp::Time &t, const rclcpp::Duration &dt)
  {

    // only read all hardware if power is on
    std::vector<std::thread> threads;
    // start all reads
    for (std::vector<HardwareInterface *> &port_interfaces : interfaces_)
    {
      threads.push_back(std::thread(threaded_read, std::ref(port_interfaces), std::ref(t), std::ref(dt)));
    }
    // wait for all reads to finish
    for (std::thread &thread : threads)
    {
      thread.join();
    }
    // aggregate all servo values for controller
    servo_interface_.read(t, dt);
  }

  void threaded_write(std::vector<HardwareInterface *> &port_interfaces,
                      const rclcpp::Time &t,
                      const rclcpp::Duration &dt)
  {
    for (HardwareInterface *interface : port_interfaces)
    {
      interface->write(t, dt);
    }
  }

  void WolfgangHardwareInterface::write(const rclcpp::Time &t, const rclcpp::Duration &dt)
  {

    // write all controller values to interfaces
    servo_interface_.write(t, dt);
    std::vector<std::thread> threads;
    // start all writes
    for (std::vector<HardwareInterface *> &port_interfaces : interfaces_)
    {
      threads.push_back(std::thread(threaded_write, std::ref(port_interfaces), std::ref(t), std::ref(dt)));
    }

    // wait for all writes to finish
    for (std::thread &thread : threads)
    {
      thread.join();
    }
  }
}